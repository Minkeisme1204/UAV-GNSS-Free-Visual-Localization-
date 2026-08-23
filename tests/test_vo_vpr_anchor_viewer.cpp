ff
    const dv::AnchorFixCounters    afc = viewer.anchorFixCounters();
    const fusion::FusionFixStats   fst = sys.fixStats();
    if (fake_anchor != nullptr) {
        const anchor::FakeAnchorStats ast = fake_anchor->stats();
        spdlog::warn("{}: the absolute fixes above were GENERATED FROM "
                     "GROUNDTRUTH — this run shows the back-end consuming "
                     "absolute positions, not a real system", DRIVER_NAME);
        spdlog::info("anchor: requests={} generated={} ({} outliers) emitted={} | "
                     "skipped: cadence={} no_telemetry={} no_groundtruth={} "
                     "no_origin={}",
                     ast.requested, ast.generated, ast.outliers, ast.emitted,
                     ast.skipped_cadence, ast.skipped_no_telemetry,
                     ast.skipped_no_groundtruth, ast.skipped_no_origin);
    }
#if UAVLOC_HAVE_VPR
    if (vpr_anchor != nullptr) {
        const anchor::VprAnchorStats vst = vpr_anchor->stats();
        spdlog::info("anchor[VPR]: yêu cầu={} nhận={} truy hồi được={} phát={} | "
                     "bỏ: nhịp={} thiếu telemetry={} chưa có gốc={} nghiêng={} "
                     "bận={} | không khớp={} conf thấp={} | p50={:.1f} ms",
                     vst.requested, vst.accepted, vst.retrieved, vst.emitted,
                     vst.skipped_cadence, vst.skipped_no_telemetry,
                     vst.skipped_no_origin, vst.skipped_tilt, vst.dropped_busy,
                     vst.unmatched, vst.low_confidence, vst.p50_query_ms);
    }
#endif
    spdlog::info("anchor markers: generated={} accepted={} rejected={} "
                 "pending={} (back-end: injected={} applied={} gated={} "
                 "low_confidence={} age_expired={} unmatched={} marginalized={} "
                 "no_graph={})",
                 afc.generated, afc.accepted, afc.rejected, afc.pending,
                 fst.injected, fst.applied, fst.gated, fst.low_confidence,
                 fst.age_expired, fst.unmatched, fst.marginalized, fst.no_graph);
    // Re-anchor breakdown on its own line (square markers in the viewer).
    // Producer-side counters differ per anchor kind; the viewer-side ones do not.
    const auto reinit_req = (fake_anchor != nullptr)
                                ? fake_anchor->stats().reinit_requested
                                : 0ULL;
    const auto reinit_thr = (fake_anchor != nullptr)
                                ? fake_anchor->stats().reinit_throttled
                                : 0ULL;
    spdlog::info("anchor re-anchor: requests={} generated={} accepted={} "
                 "throttled={} | cadence fixes={}",
                 reinit_req, afc.reinit_generated,
                 afc.reinit_accepted, reinit_thr,
                 afc.generated - afc.reinit_generated);
    // Not a failure — a FINDING that has to be visible in the log, because a run
    // that applied nothing measured nothing about the back-end consuming fixes.
    if (fst.applied == 0) {
        spdlog::warn("{}: NO absolute fix reached the graph in this run "
                     "(applied=0). The run is valid as a smoke test but says "
                     "NOTHING about the effect of absolute positioning — report "
                     "it as such", DRIVER_NAME);
    }

    spdlog::info("{}: frames_processed={} pose_successes={} final_state={}",
                 DRIVER_NAME, frames_processed.load(), pose_successes.load(),
                 final_state.load());
    return 0;
}
