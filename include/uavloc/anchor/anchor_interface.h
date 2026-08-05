#pragma once

// AnchorInterface — the request/response seam between the pipeline and whatever
// produces ABSOLUTE horizontal positions (the M1 groundtruth-driven FakeAnchor
// today, VPR + geo-referencing later).
//
// Modelled on kcb_slam's satcom interface (RequestPlaceRecognition + a result
// callback), with two of its traps deliberately not reproduced:
//
//   1. kcb's RequestPlaceRecognition() returns void although its own
//      documentation says the request is refused while the previous one is
//      still running — so the caller cannot tell a dropped request from an
//      accepted one. requestFix() returns a real bool.
//   2. kcb passes both a stamp and a position and then ignores them, because
//      the implementation reads the timestamp out of the keyframe itself. Here
//      AnchorQuery::timestamp_msec is the single path.
//
// Lifecycle mirrors every other component in this repo: setup() → start() →
// (requests flow) → stop(). core::SystemManager owns the implementation and
// sequences it against its own lifecycle.

#include "uavloc/anchor/absolute_fix.h"
#include "uavloc/anchor/anchor_query.h"

#include <functional>

namespace uavloc::anchor {

class AnchorInterface {
public:
    virtual ~AnchorInterface() = default;

    //! Build whatever the producer needs (load a database, open a model …).
    //! Must not spawn a thread and must not block on a device: a unit test
    //! calls it freely. Returns false when the producer is unusable.
    virtual bool setup() = 0;

    //! Start accepting requests (spawns the worker thread in asynchronous
    //! mode). Returns false when setup() has not succeeded.
    virtual bool start() = 0;

    //! Stop accepting requests and join whatever start() spawned. Idempotent.
    //! After it returns, no result callback can still be in flight — that is
    //! what lets the owner tear the consumer down safely.
    virtual void stop() = 0;

    using ResultCallback = std::function<void(const AbsoluteFix&)>;

    //! Register the ONE sink results are delivered to (kcb:
    //! SetSatcomResultCallback). Registering again replaces the previous sink.
    //! The callback may fire on the requesting thread (synchronous producer) or
    //! on the producer's own thread (asynchronous), so the sink must be
    //! thread-safe and must do as little as possible.
    virtual void setResultCallback(ResultCallback cb) = 0;

    //! Declare the ENU origin every produced AbsoluteFix::xy_enu must be
    //! expressed against — the SAME origin the fusion state X(0) is anchored
    //! at.
    //!
    //! ⚠ Why this is not an argument of setup(): the origin does not exist yet
    //! at setup() time. It is chosen by the pipeline at the first fused pose
    //! that coincides with usable telemetry, i.e. long after every component
    //! has been built. Letting a producer read an origin from its own config
    //! instead would create a second source of truth for the coordinate frame,
    //! which is exactly the class of silent-offset bug the S3 double-counted
    //! altitude was (see the alt0 = 0 note in core::SystemManager).
    //!
    //! A producer that has not been given an origin MUST NOT emit a fix.
    virtual void setEnuOrigin(double lat0_deg, double lon0_deg) = 0;

    //! Ask for an absolute fix for `q`. NON-BLOCKING in every implementation.
    //!
    //! Returns false when the request was dropped rather than accepted — the
    //! producer is busy, not started, or the request does not fall on its own
    //! duty cycle. A dropped request is normal operation, not an error: the
    //! cadence of absolute fixes belongs to the producer (it knows what it
    //! costs), never to the caller.
    //!
    //! true only means the request was TAKEN; a fix may still never be
    //! produced (no match), in which case nothing is delivered.
    virtual bool requestFix(const AnchorQuery& q) = 0;
};

} // namespace uavloc::anchor
