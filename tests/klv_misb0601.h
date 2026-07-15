// klv_misb0601.h — shared MISB ST 0601 KLV parser for tests-only tools.
//
// Factored out of tests/test_ts_reader.cpp so that both the .ts smoke test and
// the ts_klv_to_csv converter share one parser. Tests-only helper: lives in
// tests/, is never installed, and is not part of the libuavloc public API.
//
// Structure:
//   * KlvRecord + describe()      — always available.
//   * libav demux + KLV decoding  — only under UAVLOC_WITH_LIBAV (the consumer
//     target must link PkgConfig::LIBAV and define UAVLOC_WITH_LIBAV).

#pragma once

#include <spdlog/spdlog.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#ifdef UAVLOC_WITH_LIBAV
extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
}
#endif

namespace uavloc::tests {

// One decoded MISB ST 0601 Local Set (only the tags these tools care about).
struct KlvRecord {
    uint64_t ts_usec = 0;   // tag 2  — Precision Time Stamp (µs since epoch)

    bool   has_heading = false, has_pitch = false, has_roll = false;
    double heading_deg = 0.0, pitch_deg = 0.0, roll_deg = 0.0;   // tags 5/6/7

    bool   has_lat = false, has_lon = false, has_alt = false;
    double lat_deg = 0.0, lon_deg = 0.0, alt_m = 0.0;            // tags 13/14/15

    bool   has_hfov = false, has_vfov = false;
    double hfov_deg = 0.0, vfov_deg = 0.0;                       // tags 16/17

    bool   has_rel_az = false, has_rel_el = false;
    double rel_az_deg = 0.0, rel_el_deg = 0.0;                   // tags 18/19

    bool   has_fc_lat = false, has_fc_lon = false;
    double fc_lat_deg = 0.0, fc_lon_deg = 0.0;                   // tags 23/24

    bool   has_fc_elev = false;
    double fc_elev_m = 0.0;                                      // tag 25
};

inline std::string describe(const KlvRecord& r) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "ts=%llu us lat=%.7f lon=%.7f alt=%.1f m heading=%.2f pitch=%.2f roll=%.2f",
                  static_cast<unsigned long long>(r.ts_usec),
                  r.lat_deg, r.lon_deg, r.alt_m,
                  r.heading_deg, r.pitch_deg, r.roll_deg);
    return buf;
}

#ifdef UAVLOC_WITH_LIBAV

// MISB ST 0601 UAS Datalink Local Set 16-byte Universal Label key.
constexpr uint8_t UAS_LDS_KEY[16] = {0x06, 0x0E, 0x2B, 0x34, 0x02, 0x0B, 0x01, 0x01,
                                     0x0E, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00};

// ── Big-endian fixed-width readers ────────────────────────────────────────────
inline uint64_t beUint(const uint8_t* p, size_t n) {
    uint64_t v = 0;
    for (size_t i = 0; i < n; ++i) v = (v << 8) | p[i];
    return v;
}
inline int64_t beInt(const uint8_t* p, size_t n) {
    uint64_t v = beUint(p, n);
    // Sign-extend from n*8 bits.
    if (n < 8 && (v & (1ULL << (n * 8 - 1)))) v |= ~((1ULL << (n * 8)) - 1);
    return static_cast<int64_t>(v);
}

// ── BER length: short form (<0x80) or long form (0x80|n, then n BE bytes) ────
inline bool readBerLength(const uint8_t* buf, size_t& pos, size_t end, size_t& len_out) {
    if (pos >= end) return false;
    const uint8_t first = buf[pos++];
    if (first < 0x80) { len_out = first; return true; }
    const size_t n = first & 0x7F;
    if (n == 0 || n > 8 || pos + n > end) return false;
    len_out = static_cast<size_t>(beUint(buf + pos, n));
    pos += n;
    return true;
}

// ── BER-OID tag: 7 bits per byte, MSB is the continuation flag ────────────────
inline bool readBerOidTag(const uint8_t* buf, size_t& pos, size_t end, uint32_t& tag_out) {
    tag_out = 0;
    for (int i = 0; i < 4; ++i) {   // tags in 0601 fit easily in 28 bits
        if (pos >= end) return false;
        const uint8_t b = buf[pos++];
        tag_out = (tag_out << 7) | (b & 0x7F);
        if (!(b & 0x80)) return true;
    }
    return false;
}

// Decode a single 0601 item value into the record. Unknown tags skip silently.
// All scaling constants are the standard MISB ST 0601 mappings (commented per tag).
inline void decodeItem(uint32_t tag, const uint8_t* v, size_t len, KlvRecord& rec) {
    switch (tag) {
        case 2:   // Precision Time Stamp: uint64 microseconds since epoch
            if (len == 8) rec.ts_usec = beUint(v, 8);
            break;
        case 5:   // Platform Heading Angle: uint16 * 360/65535 deg
            if (len == 2) { rec.heading_deg = beUint(v, 2) * (360.0 / 65535.0); rec.has_heading = true; }
            break;
        case 6:   // Platform Pitch Angle: int16 * 20/32767 deg
            if (len == 2) { rec.pitch_deg = beInt(v, 2) * (20.0 / 32767.0); rec.has_pitch = true; }
            break;
        case 7:   // Platform Roll Angle: int16 * 50/32767 deg
            if (len == 2) { rec.roll_deg = beInt(v, 2) * (50.0 / 32767.0); rec.has_roll = true; }
            break;
        case 13:  // Sensor Latitude: int32 * 90/2147483647 deg
            if (len == 4) { rec.lat_deg = beInt(v, 4) * (90.0 / 2147483647.0); rec.has_lat = true; }
            break;
        case 14:  // Sensor Longitude: int32 * 180/2147483647 deg
            if (len == 4) { rec.lon_deg = beInt(v, 4) * (180.0 / 2147483647.0); rec.has_lon = true; }
            break;
        case 15:  // Sensor True Altitude: uint16 * 19900/65535 - 900 m
            if (len == 2) { rec.alt_m = beUint(v, 2) * (19900.0 / 65535.0) - 900.0; rec.has_alt = true; }
            break;
        case 16:  // Sensor Horizontal FOV: uint16 * 180/65535 deg
            if (len == 2) { rec.hfov_deg = beUint(v, 2) * (180.0 / 65535.0); rec.has_hfov = true; }
            break;
        case 17:  // Sensor Vertical FOV: uint16 * 180/65535 deg
            if (len == 2) { rec.vfov_deg = beUint(v, 2) * (180.0 / 65535.0); rec.has_vfov = true; }
            break;
        case 18:  // Sensor Relative Azimuth: uint32 * 360/4294967295 deg
            if (len == 4) { rec.rel_az_deg = beUint(v, 4) * (360.0 / 4294967295.0); rec.has_rel_az = true; }
            break;
        case 19:  // Sensor Relative Elevation: int32 * 180/4294967294 deg
            if (len == 4) { rec.rel_el_deg = beInt(v, 4) * (180.0 / 4294967294.0); rec.has_rel_el = true; }
            break;
        case 23:  // Frame Center Latitude: int32 * 90/2147483647 deg
            if (len == 4) { rec.fc_lat_deg = beInt(v, 4) * (90.0 / 2147483647.0); rec.has_fc_lat = true; }
            break;
        case 24:  // Frame Center Longitude: int32 * 180/2147483647 deg
            if (len == 4) { rec.fc_lon_deg = beInt(v, 4) * (180.0 / 2147483647.0); rec.has_fc_lon = true; }
            break;
        case 25:  // Frame Center Elevation: uint16 * 19900/65535 - 900 m
            if (len == 2) { rec.fc_elev_m = beUint(v, 2) * (19900.0 / 65535.0) - 900.0; rec.has_fc_elev = true; }
            break;
        default:  // unknown/uninteresting tag — skip silently
            break;
    }
}

// Parse one packet's payload: scan for UL key(s) and decode each Local Set.
// A single AVPacket may (rarely) carry more than one Local Set.
inline void parseKlvLocalSet(const uint8_t* data, size_t size, std::vector<KlvRecord>& out) {
    size_t pos = 0;
    while (pos + sizeof(UAS_LDS_KEY) <= size) {
        // Scan for the 16-byte UL key (absorbs any leading AU/sync header).
        const uint8_t* hit = std::search(data + pos, data + size,
                                         UAS_LDS_KEY, UAS_LDS_KEY + sizeof(UAS_LDS_KEY));
        if (hit == data + size) return;
        pos = static_cast<size_t>(hit - data) + sizeof(UAS_LDS_KEY);

        size_t set_len = 0;
        if (!readBerLength(data, pos, size, set_len)) return;
        const size_t set_end = std::min(pos + set_len, size);

        KlvRecord rec;
        while (pos < set_end) {
            uint32_t tag = 0;
            size_t   len = 0;
            if (!readBerOidTag(data, pos, set_end, tag)) break;
            if (!readBerLength(data, pos, set_end, len)) break;
            if (pos + len > set_end) break;   // truncated item — bail on this set
            decodeItem(tag, data + pos, len, rec);
            pos += len;
        }
        if (rec.ts_usec != 0) out.push_back(rec);
        pos = set_end;
    }
}

// Demux the .ts and parse every KLV data packet. No video decode.
// Records are returned sorted by timestamp.
inline bool readKlvStream(const std::string& ts_path, std::vector<KlvRecord>& records) {
    AVFormatContext* fmt = nullptr;
    if (avformat_open_input(&fmt, ts_path.c_str(), nullptr, nullptr) < 0) {
        spdlog::error("[KLV] avformat_open_input failed for '{}'", ts_path);
        return false;
    }
    if (avformat_find_stream_info(fmt, nullptr) < 0) {
        spdlog::error("[KLV] avformat_find_stream_info failed");
        avformat_close_input(&fmt);
        return false;
    }

    int data_stream = -1;
    for (unsigned i = 0; i < fmt->nb_streams; ++i) {
        const AVCodecParameters* par = fmt->streams[i]->codecpar;
        if (par->codec_type == AVMEDIA_TYPE_DATA) {
            data_stream = static_cast<int>(i);
            if (par->codec_id == AV_CODEC_ID_SMPTE_KLV) break;   // prefer explicit KLV
        }
    }
    if (data_stream < 0) {
        spdlog::error("[KLV] no data stream found in '{}'", ts_path);
        avformat_close_input(&fmt);
        return false;
    }
    spdlog::info("[KLV] data stream index {} (codec_id={})", data_stream,
                 avcodec_get_name(fmt->streams[data_stream]->codecpar->codec_id));

    AVPacket* pkt = av_packet_alloc();
    while (av_read_frame(fmt, pkt) >= 0) {
        if (pkt->stream_index == data_stream && pkt->size > 0) {
            parseKlvLocalSet(pkt->data, static_cast<size_t>(pkt->size), records);
        }
        av_packet_unref(pkt);
    }
    av_packet_free(&pkt);
    avformat_close_input(&fmt);

    std::sort(records.begin(), records.end(),
              [](const KlvRecord& a, const KlvRecord& b) { return a.ts_usec < b.ts_usec; });
    return true;
}

#endif  // UAVLOC_WITH_LIBAV

}  // namespace uavloc::tests
