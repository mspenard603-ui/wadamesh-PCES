// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// PCES (Path Computation Element Service) client codec: Commsol's on-air path
// service. A Commsol station answers "what is the best path from A to B?" from the
// link-quality database it builds out of TRACE results. Spec of record: Commsol
// docs/pces.md (request type 0xC5, version 1).
//
// Pure functions, no Arduino or LVGL, so the host test (test/test_pces_proto.cpp)
// exercises exactly what the firmware sends and decodes. The touch UI's PCES screen
// is src/ui-touch/PcesApp.h.
//
// Wire format (all little-endian; SNR is int8 dB x4, -128 = none):
//   request   [0xC5][1][op] + op data
//     INFO  0x00  -
//     PATH  0x01  [width 1..4][lenA][A..][lenB][B..]   (len 0 = the answering station)
//     LINK  0x02  [lenA][A..][lenB][B..]
//   response  (after the 4-byte reflected tag) [1][op][status] + body when status 0
//     PATH  [metric u8][weakest SNR i8][flags u8][count u8][count x width]
//     LINK  [ab u8][ba u8][ab SNR i8][ba SNR i8][ab n u16][ba n u16][age min u16]
//     INFO  [window h u16][links u16][newest min u16][decode limit i8][SF u8]
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace PcesProto {

static constexpr uint8_t REQ_TYPE = 0xC5;
static constexpr uint8_t VERSION = 1;
enum Op : uint8_t { OP_INFO = 0x00, OP_PATH = 0x01, OP_LINK = 0x02 };
enum Status : uint8_t {
  ST_OK = 0, ST_NO_PATH = 1, ST_A_UNKNOWN = 2, ST_B_UNKNOWN = 3, ST_A_AMBIGUOUS = 4,
  ST_B_AMBIGUOUS = 5, ST_BAD_REQUEST = 6, ST_RATE_LIMITED = 7, ST_NO_DATA = 8, ST_UNSUPPORTED = 9,
  ST_MALFORMED = 0xFF   // not on the wire: the reply did not parse
};
static constexpr int8_t SNR_NONE = -128;
static constexpr uint8_t MAX_NODE = 8;     // a node is a 1..8-byte pubkey prefix
static constexpr uint8_t MAX_HOPS = 63;    // MeshCore path limit
static constexpr uint8_t MAX_WIDTH = 3;    // MeshCore path hashes are 1..3 bytes

// Advert feature-1 marker: (f & 0xFF00) == 0xC500 is a Commsol node; bit 1 answers PCES.
inline bool isCommsolNode(uint16_t feat1) { return (feat1 & 0xFF00) == 0xC500; }
inline bool answersPces(uint16_t feat1) { return isCommsolNode(feat1) && (feat1 & 0x0002); }

inline const char* statusText(uint8_t st) {
  switch (st) {
    case ST_OK: return "ok";
    case ST_NO_PATH: return "No path known";
    case ST_A_UNKNOWN: return "First hop unknown to PCES";
    case ST_B_UNKNOWN: return "Destination unknown to PCES";
    case ST_A_AMBIGUOUS: return "First hop ambiguous";
    case ST_B_AMBIGUOUS: return "Destination ambiguous";
    case ST_BAD_REQUEST: return "Bad request";
    case ST_RATE_LIMITED: return "Rate limited";
    case ST_NO_DATA: return "No link data yet";
    case ST_UNSUPPORTED: return "Unsupported version";
    default: return "Malformed reply";
  }
}

// ---- requests: each returns the length written (out must hold 3 + 2 + 2*8 + 1 = 22) ----
inline size_t putNode(uint8_t* out, const uint8_t* n, uint8_t len) {
  if (len > MAX_NODE) len = MAX_NODE;
  out[0] = len;
  if (len) memcpy(out + 1, n, len);
  return 1u + len;
}
inline size_t encodeInfo(uint8_t* out) {
  out[0] = REQ_TYPE; out[1] = VERSION; out[2] = OP_INFO;
  return 3;
}
inline size_t encodePath(uint8_t* out, const uint8_t* a, uint8_t alen, const uint8_t* b, uint8_t blen, uint8_t width) {
  size_t i = 0;
  out[i++] = REQ_TYPE; out[i++] = VERSION; out[i++] = OP_PATH; out[i++] = width;
  i += putNode(out + i, a, alen);
  i += putNode(out + i, b, blen);
  return i;
}
inline size_t encodeLink(uint8_t* out, const uint8_t* a, uint8_t alen, const uint8_t* b, uint8_t blen) {
  size_t i = 0;
  out[i++] = REQ_TYPE; out[i++] = VERSION; out[i++] = OP_LINK;
  i += putNode(out + i, a, alen);
  i += putNode(out + i, b, blen);
  return i;
}

// ---- responses ------------------------------------------------------------------------
struct PathAnswer {
  uint8_t status = ST_MALFORMED;
  uint8_t metric = 0;
  int8_t  weakest_snr4 = SNR_NONE;
  bool    starts_at_station = false, ends_at_station = false;
  uint8_t width = 0, hops = 0;
  uint8_t hop[MAX_HOPS * MAX_WIDTH] = {0};   // hops x width bytes, relays from A to B
};
struct LinkAnswer {
  uint8_t status = ST_MALFORMED;
  uint8_t ab_metric = 255, ba_metric = 255;   // 255 = that direction never measured
  int8_t  ab_snr4 = SNR_NONE, ba_snr4 = SNR_NONE;
  uint16_t ab_samples = 0, ba_samples = 0, age_min = 0;
};
struct InfoAnswer {
  uint8_t status = ST_MALFORMED;
  uint16_t window_h = 0, links = 0, newest_min = 0;
  int8_t  decode_limit4 = 0;
  uint8_t sf = 0;
};

inline uint16_t rd16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

// Shared header check: version and op must match; returns the status (ST_MALFORMED on a bad header).
inline uint8_t header(const uint8_t* d, size_t len, uint8_t op) {
  if (!d || len < 3 || d[0] != VERSION || d[1] != op) return ST_MALFORMED;
  return d[2];
}

// `width` is the one we asked for: the answer's hops are that many bytes each.
inline bool decodePath(const uint8_t* d, size_t len, uint8_t width, PathAnswer& out) {
  out = PathAnswer();
  out.status = header(d, len, OP_PATH);
  if (out.status != ST_OK) return false;
  if (len < 7 || width < 1 || width > MAX_WIDTH) { out.status = ST_MALFORMED; return false; }
  const uint8_t n = d[6];
  if (n > MAX_HOPS || len < 7u + (size_t)n * width) { out.status = ST_MALFORMED; return false; }
  out.metric = d[3];
  out.weakest_snr4 = (int8_t)d[4];
  out.starts_at_station = (d[5] & 1) != 0;
  out.ends_at_station = (d[5] & 2) != 0;
  out.width = width;
  out.hops = n;
  memcpy(out.hop, d + 7, (size_t)n * width);
  return true;
}
inline bool decodeLink(const uint8_t* d, size_t len, LinkAnswer& out) {
  out = LinkAnswer();
  out.status = header(d, len, OP_LINK);
  if (out.status != ST_OK) return false;
  if (len < 13) { out.status = ST_MALFORMED; return false; }
  out.ab_metric = d[3]; out.ba_metric = d[4];
  out.ab_snr4 = (int8_t)d[5]; out.ba_snr4 = (int8_t)d[6];
  out.ab_samples = rd16(d + 7); out.ba_samples = rd16(d + 9); out.age_min = rd16(d + 11);
  return true;
}
inline bool decodeInfo(const uint8_t* d, size_t len, InfoAnswer& out) {
  out = InfoAnswer();
  out.status = header(d, len, OP_INFO);
  if (out.status != ST_OK) return false;
  if (len < 11) { out.status = ST_MALFORMED; return false; }
  out.window_h = rd16(d + 3); out.links = rd16(d + 5); out.newest_min = rd16(d + 7);
  out.decode_limit4 = (int8_t)d[9]; out.sf = d[10];
  return true;
}

// ---- turning an answer into a contact route ------------------------------------------------
// Commsol spec, "Using the answer as a contact route": A is our first-hop repeater,
// B the contact's last relay, or the contact itself when it is a relay. The answer
// lists the relays from A to B, A and B included when they are relays. A route
// never contains its own destination, so when B is the contact it is dropped.
struct Plan {
  uint8_t width = 0;                         // bytes per hop, = the stored route's hash size
  uint8_t a[MAX_NODE] = {0}, alen = 0;
  uint8_t b[MAX_NODE] = {0}, blen = 0;
  bool    b_is_contact = false;
};
enum PlanErr : uint8_t { PLAN_OK = 0, PLAN_NO_ROUTE, PLAN_NO_LAST_RELAY, PLAN_BAD_WIDTH };

// route: the contact's stored out path (hops x width bytes, first hop first), hops 0 =
// direct / none. contact_pub: its public key (at least 4 bytes). contact_is_relay:
// repeater or room server.
inline PlanErr makePlan(const uint8_t* route, uint8_t hops, uint8_t width,
                        const uint8_t* contact_pub, bool contact_is_relay, Plan& p) {
  p = Plan();
  if (width < 1 || width > MAX_WIDTH) return PLAN_BAD_WIDTH;
  if (hops == 0) return PLAN_NO_ROUTE;   // no first hop to start from (or it is a neighbour already)
  p.width = width;
  memcpy(p.a, route, width); p.alen = width;
  if (contact_is_relay) {
    // Four bytes: well clear of the prefix collisions PCES answers "ambiguous" to.
    memcpy(p.b, contact_pub, 4); p.blen = 4; p.b_is_contact = true;
  } else {
    if (hops < 2) return PLAN_NO_LAST_RELAY;   // A would be B: nothing to compute
    memcpy(p.b, route + (size_t)(hops - 1) * width, width); p.blen = width;
  }
  return PLAN_OK;
}

// Fills `out` (hops x width) with the route to install; returns its hop count, or -1
// when the answer does not fit the plan (wrong width, or not starting at A).
inline int routeFromAnswer(const Plan& p, const PathAnswer& a, uint8_t* out) {
  if (a.status != ST_OK || a.width != p.width) return -1;
  if (a.hops == 0 || memcmp(a.hop, p.a, p.width) != 0) return -1;   // must begin at our first hop
  uint8_t n = a.hops;
  if (p.b_is_contact && memcmp(a.hop + (size_t)(n - 1) * p.width, p.b, p.width) == 0) --n;
  memcpy(out, a.hop, (size_t)n * p.width);
  return n;
}

}  // namespace PcesProto
