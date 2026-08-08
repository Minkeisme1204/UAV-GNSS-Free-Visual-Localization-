// test_util_callback — unit test for util::CallbackSlot<void(Args...)> and
// util::ThreadsafeQueue<T> (SystemManager steps S1 + S5b,
// .docs/designs/system_manager_design.md §7). Replaces test_util_pubsub: S5b
// removed util::Publisher<T> and unified every channel of the project on
// CallbackSlot, so the checks below are the S1 ones ported plus C7/C8, which
// cover exactly what the switch was made for.
//
// Pure unit test: no data files, no config, headless, exits non-zero on failure.
//
// CallbackSlot checks:
//   C1 several subscribers are called in registration order
//   C2 remove() removes; removing an unknown id returns false
//   C3 a subscriber calling add()/remove()/operator() from inside its own
//      callback must NOT deadlock (guarded by a hard deadline). This is the
//      defect of the previous debug_viewer::CallbackSlot, which held its mutex
//      across the call.
//   C4 an exception thrown by one subscriber does not stop the others
//   C5 clear() removes everything
//   C6 thread safety: concurrent emission + add/remove, no crash and no missed
//      delivery for a subscriber registered before the storm
//   C7 a MULTI-ARGUMENT signature — (double timestamp, const Payload&) — is
//      delivered verbatim, with no wrapper struct. This is the reason S5b
//      dropped Publisher<T>, which could only carry one payload parameter.
//   C8 remove() called from INSIDE a callback, on a worker with a deadline: the
//      removed subscriber still runs for the in-flight emission (documented
//      snapshot semantics) and never again afterwards.
//
// ThreadsafeQueue checks:
//   Q1 strict FIFO (and move-only element types)
//   Q2 push_blocking_if_full really blocks when full, then succeeds once the
//      consumer makes room (proved by measuring the elapsed time)
//   Q3 push_blocking_if_full times out: returns false and pushes NOTHING
//   Q4 push_dropping_if_full drops the OLDEST item, dropped_count is exact
//   Q5 shutdown() wakes a blocked pop_blocking AND a blocked push (deadline)
//   Q6 after shutdown() pop_blocking still drains the leftovers, then reports
//      false; pushes after shutdown are refused
//
// Every wait on a worker thread has a deadline — a hang is reported as a
// failure instead of blocking the test run forever.

#include "uavloc/util/callback_slot.h"
#include "uavloc/util/threadsafe_queue.h"

#include <spdlog/spdlog.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

using uavloc::util::CallbackSlot;
using uavloc::util::ThreadsafeQueue;

using Clock = std::chrono::steady_clock;
using Ms    = std::chrono::milliseconds;

//! Slot type used by most checks: one int argument.
using IntSlot = CallbackSlot<void(int)>;

// ── Test tunables (no magic numbers inline) ───────────────────────────────
//! Hard deadline for any worker thread we wait on. Generous: it only has to
//! separate "slow machine" from "deadlocked".
constexpr Ms THREAD_DEADLINE{5000};
//! How long the Q2 consumer sleeps before making room — must be long enough
//! that a non-blocking (buggy) push cannot accidentally look like a blocking one.
constexpr Ms Q2_CONSUMER_DELAY{200};
//! Lower bound accepted as proof that Q2's push really waited (scheduler slop).
constexpr Ms Q2_MIN_BLOCKED{100};
//! Timeout handed to the Q3 push that must expire.
constexpr Ms Q3_PUSH_TIMEOUT{150};
//! Lower bound accepted as proof that Q3's push waited for its full timeout.
constexpr Ms Q3_MIN_WAITED{100};
//! Delay before the main thread calls shutdown() in Q5, so the workers are
//! provably parked inside their wait when it happens.
constexpr Ms Q5_SHUTDOWN_DELAY{100};

constexpr int         C6_EMITTER_THREADS      = 4;
constexpr int         C6_EMISSIONS_PER_THREAD = 500;
constexpr int         C6_SUBSCRIBER_THREADS   = 3;
constexpr int         C6_SUB_CYCLES           = 200;
constexpr std::size_t Q_SMALL_CAPACITY        = 1;
constexpr std::size_t Q_MED_CAPACITY          = 2;
constexpr std::size_t Q_DROP_CAPACITY         = 3;
constexpr std::size_t Q_DRAIN_CAPACITY        = 4;

//! Payload of C7 — stands in for a real channel payload (VOData, FusionResult).
struct Payload {
    int         id = 0;
    std::string tag;
};

bool g_ok = true;

bool check(bool cond, const char* name) {
    if (cond) {
        spdlog::info("PASS: {}", name);
    } else {
        spdlog::error("FAIL: {}", name);
        g_ok = false;
    }
    return cond;
}

//! Waits for a worker's completion signal with a hard deadline. On timeout the
//! worker is stuck, and it still references objects on this test's stack —
//! unwinding would be undefined behaviour, so report and terminate at once.
void await_or_die(std::future<void>& fut, const char* name) {
    if (fut.wait_for(THREAD_DEADLINE) != std::future_status::ready) {
        spdlog::error("FAIL: {} — worker did not finish within {} ms (hang/deadlock)",
                      name, THREAD_DEADLINE.count());
        spdlog::default_logger()->flush();
        std::_Exit(EXIT_FAILURE);
    }
}

double elapsed_ms(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// ── CallbackSlot ──────────────────────────────────────────────────────────

void test_slot_order() {
    IntSlot          slot;
    std::vector<int> calls;
    int              seen_value = 0;

    slot.add([&](int v) { calls.push_back(1); seen_value = v; });
    slot.add([&](int) { calls.push_back(2); });
    slot.add([&](int) { calls.push_back(3); });

    slot(42);

    const bool ordered = calls == std::vector<int>{1, 2, 3};
    check(ordered && seen_value == 42 && slot.size() == 3,
          "C1 slot calls subscribers in registration order");

    // An empty callback is refused and does not become a subscriber.
    const auto bad_id = slot.add(IntSlot::Callback{});
    check(bad_id == IntSlot::INVALID_ID && slot.size() == 3,
          "C1b empty callback is rejected");

    // An empty slot must be callable and do nothing.
    IntSlot empty;
    empty(1);
    check(empty.size() == 0 && empty.empty(), "C1c an empty slot is safe to call");
}

void test_slot_remove() {
    IntSlot slot;
    int     a = 0, b = 0;

    const auto id_a = slot.add([&](int) { ++a; });
    const auto id_b = slot.add([&](int) { ++b; });

    slot(0);
    const bool removed     = slot.remove(id_a);
    const bool removed_2nd = slot.remove(id_a);            // already gone
    const bool removed_bad = slot.remove(id_b + 1000);     // never handed out
    const bool removed_inv = slot.remove(IntSlot::INVALID_ID);
    slot(0);

    check(removed && !removed_2nd && !removed_bad && !removed_inv && a == 1 && b == 2 &&
              slot.size() == 1,
          "C2 remove() removes; unknown id returns false");
}

void test_slot_reentrant() {
    // A subscriber that re-enters the slot would deadlock if operator() held the
    // mutex while calling back — the exact bug of the old debug_viewer copy.
    // Run it on a worker with a deadline: a hang is reported, not waited on.
    IntSlot            slot;
    std::promise<void> done;
    std::future<void>  fut = done.get_future();

    std::atomic<int>  inner_calls{0};
    std::atomic<int>  outer_calls{0};
    std::atomic<bool> nested_ok{false};

    auto self_id = std::make_shared<std::atomic<IntSlot::Id>>(IntSlot::INVALID_ID);

    std::thread worker([&] {
        // Subscriber 1: removes ITSELF from inside its own callback...
        const auto id = slot.add([&](int) {
            ++outer_calls;
            slot.remove(self_id->load());
            // ... and registers a new subscriber while the emission is in flight.
            slot.add([&](int) { ++inner_calls; });
        });
        self_id->store(id);

        slot(1);  // outer_calls 0→1, adds the inner subscriber
        slot(2);  // only the inner subscriber remains
        nested_ok.store(outer_calls.load() == 1 && inner_calls.load() == 1);
        done.set_value();
    });

    await_or_die(fut, "C3 re-entrant add/remove inside a callback");
    worker.join();

    check(nested_ok.load(), "C3 re-entrant add/remove inside a callback");
}

void test_slot_exception() {
    IntSlot slot;
    int     before = 0, after = 0;

    slot.add([&](int) { ++before; });
    slot.add([&](int) { throw std::runtime_error("subscriber blew up"); });
    slot.add([&](int) { ++after; });

    bool threw = false;
    try {
        slot(7);
    } catch (...) {
        threw = true;
    }

    check(!threw && before == 1 && after == 1,
          "C4 a throwing subscriber does not stop the following ones");
}

void test_slot_clear() {
    IntSlot    slot;
    int        hits = 0;
    const auto id   = slot.add([&](int) { ++hits; });

    slot.clear();
    slot(0);

    check(slot.size() == 0 && hits == 0 && !slot.remove(id),
          "C5 clear() removes every subscriber");
}

void test_slot_threading() {
    IntSlot slot;

    // Registered BEFORE any emission and never removed ⇒ it must observe every
    // single call. That is the only delivery invariant a racing test can assert.
    auto permanent_hits = std::make_shared<std::atomic<long long>>(0);
    slot.add([permanent_hits](int) { permanent_hits->fetch_add(1); });

    std::atomic<bool>        stop_churning{false};
    std::vector<std::thread> threads;

    for (int t = 0; t < C6_EMITTER_THREADS; ++t) {
        threads.emplace_back([&slot] {
            for (int i = 0; i < C6_EMISSIONS_PER_THREAD; ++i) {
                slot(i);
            }
        });
    }
    for (int t = 0; t < C6_SUBSCRIBER_THREADS; ++t) {
        threads.emplace_back([&slot, &stop_churning] {
            // The transient counter is owned by the callback (shared_ptr), so an
            // in-flight emission that still holds a snapshot stays safe after
            // the remove() returns.
            auto churn_hits = std::make_shared<std::atomic<int>>(0);
            for (int i = 0; i < C6_SUB_CYCLES && !stop_churning.load(); ++i) {
                const auto id = slot.add([churn_hits](int) { churn_hits->fetch_add(1); });
                slot.remove(id);
            }
        });
    }

    // Deadline for the whole storm.
    std::promise<void> done;
    std::future<void>  fut = done.get_future();
    std::thread        joiner([&] {
        for (auto& th : threads) {
            th.join();
        }
        done.set_value();
    });
    if (fut.wait_for(THREAD_DEADLINE) != std::future_status::ready) {
        stop_churning.store(true);
        spdlog::error("FAIL: C6 concurrent emission + add/remove — storm exceeded {} ms",
                      THREAD_DEADLINE.count());
        spdlog::default_logger()->flush();
        std::_Exit(EXIT_FAILURE);
    }
    joiner.join();

    const long long expected =
        static_cast<long long>(C6_EMITTER_THREADS) * C6_EMISSIONS_PER_THREAD;
    const long long got = permanent_hits->load();
    if (got != expected) {
        spdlog::error("C6: permanent subscriber saw {} calls, expected {}", got, expected);
    }
    check(got == expected, "C6 concurrent emission + add/remove is safe and lossless");
}

void test_slot_multi_argument() {
    // The shape every SystemManager channel uses: (timestamp, payload). No
    // wrapper struct, which is precisely what Publisher<T> could not express.
    CallbackSlot<void(double, const Payload&)> slot;

    double      seen_ts  = 0.0;
    int         seen_id  = 0;
    std::string seen_tag;
    int         hits = 0;

    slot.add([&](double ts, const Payload& p) {
        seen_ts  = ts;
        seen_id  = p.id;
        seen_tag = p.tag;
        ++hits;
    });

    const Payload p{7, "vo"};
    slot(1234.5, p);

    // A zero-argument signature must work too (no Args at all).
    CallbackSlot<void()> nullary;
    int                  nullary_hits = 0;
    nullary.add([&] { ++nullary_hits; });
    nullary();

    check(hits == 1 && seen_ts == 1234.5 && seen_id == 7 && seen_tag == "vo" &&
              nullary_hits == 1,
          "C7 a (timestamp, payload) signature is delivered verbatim");
}

void test_slot_remove_inside_callback() {
    // Documented snapshot semantics: a subscriber removed DURING an emission
    // may still run for THAT emission, and never for a later one. Runs on a
    // worker with a deadline, because a slot that held its mutex while calling
    // would hang here instead of failing.
    IntSlot            slot;
    std::promise<void> done;
    std::future<void>  fut = done.get_future();

    std::atomic<int>  victim_calls{0};
    std::atomic<int>  remover_calls{0};
    std::atomic<bool> removed_ok{false};
    std::atomic<bool> gone_after{false};

    auto victim_id = std::make_shared<std::atomic<IntSlot::Id>>(IntSlot::INVALID_ID);

    std::thread worker([&] {
        // Registered FIRST, so it is invoked before the remover on emission 1.
        victim_id->store(slot.add([&](int) { ++victim_calls; }));
        slot.add([&](int) {
            // Only the FIRST emission actually removes anything; on the second
            // one remove() correctly reports false (already gone), so the
            // result is latched instead of overwritten.
            const bool removed = slot.remove(victim_id->load());
            if (remover_calls.fetch_add(1) == 0) {
                removed_ok.store(removed);
            }
        });

        slot(1);                              // victim runs, then is removed
        const int after_first = victim_calls.load();
        slot(2);                              // victim must NOT run again
        gone_after.store(victim_calls.load() == after_first);
        done.set_value();
    });

    await_or_die(fut, "C8 remove() from inside a callback");
    worker.join();

    check(removed_ok.load() && remover_calls.load() == 2 && victim_calls.load() == 1 &&
              gone_after.load() && slot.size() == 1,
          "C8 remove() from inside a callback does not deadlock and takes effect "
          "from the next emission");
}

// ── ThreadsafeQueue ───────────────────────────────────────────────────────

void test_queue_fifo() {
    ThreadsafeQueue<int> q(Q_DRAIN_CAPACITY);
    bool                 pushed_all = true;
    for (int i = 1; i <= static_cast<int>(Q_DRAIN_CAPACITY); ++i) {
        pushed_all = pushed_all && q.push_blocking_if_full(i, THREAD_DEADLINE);
    }

    const bool sized = q.size() == Q_DRAIN_CAPACITY && !q.empty() &&
                       q.capacity() == Q_DRAIN_CAPACITY && q.dropped_count() == 0;

    bool ordered = true;
    for (int i = 1; i <= static_cast<int>(Q_DRAIN_CAPACITY); ++i) {
        int v = 0;
        ordered = ordered && q.pop_blocking(v) && v == i;
    }
    int  spare      = 0;
    const bool drained = !q.try_pop(spare) && q.empty();

    check(pushed_all && sized && ordered && drained, "Q1 strict FIFO order");

    // Move-only payloads must work (the queue takes T by value and moves).
    ThreadsafeQueue<std::unique_ptr<int>> mq(Q_MED_CAPACITY);
    mq.push_dropping_if_full(std::make_unique<int>(11));
    std::unique_ptr<int> out;
    check(mq.pop_blocking(out) && out && *out == 11, "Q1b move-only element type");
}

void test_queue_push_blocks() {
    ThreadsafeQueue<int> q(Q_MED_CAPACITY);
    q.push_blocking_if_full(1, THREAD_DEADLINE);
    q.push_blocking_if_full(2, THREAD_DEADLINE);  // now full

    std::promise<void> done;
    std::future<void>  fut = done.get_future();
    int                consumed = 0;
    std::thread        consumer([&] {
        std::this_thread::sleep_for(Q2_CONSUMER_DELAY);
        q.pop_blocking(consumed);
        done.set_value();
    });

    const auto t0      = Clock::now();
    const bool pushed  = q.push_blocking_if_full(3, THREAD_DEADLINE);
    const double waited = elapsed_ms(t0);

    await_or_die(fut, "Q2 blocking push");
    consumer.join();

    spdlog::info("Q2: push waited {:.1f} ms (consumer delay {} ms)", waited, Q2_CONSUMER_DELAY.count());
    check(pushed && consumed == 1 && waited >= static_cast<double>(Q2_MIN_BLOCKED.count()) &&
              q.size() == Q_MED_CAPACITY,
          "Q2 push_blocking_if_full blocks while full, succeeds when room appears");
}

void test_queue_push_timeout() {
    ThreadsafeQueue<int> q(Q_SMALL_CAPACITY);
    q.push_blocking_if_full(1, THREAD_DEADLINE);  // full

    const auto   t0     = Clock::now();
    const bool   pushed = q.push_blocking_if_full(99, Q3_PUSH_TIMEOUT);
    const double waited = elapsed_ms(t0);

    int  front = 0;
    const bool kept_original = q.pop_blocking(front) && front == 1 && q.empty();

    spdlog::info("Q3: push timed out after {:.1f} ms (timeout {} ms)", waited, Q3_PUSH_TIMEOUT.count());
    check(!pushed && waited >= static_cast<double>(Q3_MIN_WAITED.count()) && kept_original,
          "Q3 push_blocking_if_full times out and pushes nothing");
}

void test_queue_drop_oldest() {
    ThreadsafeQueue<int> q(Q_DROP_CAPACITY);
    bool no_loss = true;
    for (int i = 1; i <= static_cast<int>(Q_DROP_CAPACITY); ++i) {
        no_loss = no_loss && q.push_dropping_if_full(i);  // 1,2,3
    }
    const bool dropped_4 = !q.push_dropping_if_full(4);   // drops 1 → 2,3,4
    const bool dropped_5 = !q.push_dropping_if_full(5);   // drops 2 → 3,4,5

    const bool counted = q.dropped_count() == 2 && q.size() == Q_DROP_CAPACITY;

    bool contents_ok = true;
    for (int expected = 3; expected <= 5; ++expected) {
        int v = 0;
        contents_ok = contents_ok && q.try_pop(v) && v == expected;
    }

    check(no_loss && dropped_4 && dropped_5 && counted && contents_ok,
          "Q4 push_dropping_if_full drops the oldest and counts it");
}

void test_queue_shutdown_wakes_waiters() {
    // (a) a consumer parked in pop_blocking
    {
        ThreadsafeQueue<int> q(Q_MED_CAPACITY);
        std::promise<void>   done;
        std::future<void>    fut = done.get_future();
        std::atomic<bool>    popped{true};

        std::thread consumer([&] {
            int v = 0;
            popped.store(q.pop_blocking(v));
            done.set_value();
        });

        std::this_thread::sleep_for(Q5_SHUTDOWN_DELAY);
        q.shutdown();
        await_or_die(fut, "Q5 shutdown wakes a blocked pop_blocking");
        consumer.join();

        check(!popped.load() && q.is_shutdown(),
              "Q5 shutdown wakes a blocked pop_blocking");
    }

    // (b) a producer parked in push_blocking_if_full
    {
        ThreadsafeQueue<int> q(Q_SMALL_CAPACITY);
        q.push_blocking_if_full(1, THREAD_DEADLINE);  // full

        std::promise<void> done;
        std::future<void>  fut = done.get_future();
        std::atomic<bool>  pushed{true};

        std::thread producer([&] {
            // A timeout far beyond the deadline: only shutdown() can free it.
            pushed.store(q.push_blocking_if_full(2, THREAD_DEADLINE * 10));
            done.set_value();
        });

        std::this_thread::sleep_for(Q5_SHUTDOWN_DELAY);
        q.shutdown();
        await_or_die(fut, "Q5b shutdown wakes a blocked push_blocking_if_full");
        producer.join();

        check(!pushed.load() && q.size() == Q_SMALL_CAPACITY,
              "Q5b shutdown wakes a blocked push_blocking_if_full");
    }
}

void test_queue_drain_after_shutdown() {
    ThreadsafeQueue<int> q(Q_DRAIN_CAPACITY);
    for (int i = 1; i <= 3; ++i) {
        q.push_blocking_if_full(i, THREAD_DEADLINE);
    }
    q.shutdown();

    const bool refused_block = !q.push_blocking_if_full(99, THREAD_DEADLINE);
    const bool refused_drop  = !q.push_dropping_if_full(98);

    bool drained_in_order = true;
    for (int expected = 1; expected <= 3; ++expected) {
        int v = 0;
        drained_in_order = drained_in_order && q.pop_blocking(v) && v == expected;
    }
    int  v_after = 0;
    const bool closed = !q.pop_blocking(v_after) && !q.try_pop(v_after);

    check(refused_block && refused_drop && drained_in_order && closed,
          "Q6 pop_blocking drains leftovers after shutdown, then reports false");
}

}  // namespace

int main() {
    spdlog::set_level(spdlog::level::info);

    test_slot_order();
    test_slot_remove();
    test_slot_reentrant();
    test_slot_exception();
    test_slot_clear();
    test_slot_threading();
    test_slot_multi_argument();
    test_slot_remove_inside_callback();

    test_queue_fifo();
    test_queue_push_blocks();
    test_queue_push_timeout();
    test_queue_drop_oldest();
    test_queue_shutdown_wakes_waiters();
    test_queue_drain_after_shutdown();

    if (!g_ok) {
        spdlog::error("test_util_callback: FAIL");
        return 1;
    }
    spdlog::info("test_util_callback: PASS");
    return 0;
}
