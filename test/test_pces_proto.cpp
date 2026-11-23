// SPDX-License-Identifier: GPL-3.0-or-later
//
// Host test for the PCES client codec (src/helpers/PcesProto.h), against the worked
// example and tables in Commsol's docs/pces.md. Run from the repo root:
//
//   c++ -std=c++17 -Wall -Wextra -I src test/test_pces_proto.cpp -o /tmp/test_pces_proto
//   /tmp/test_pces_proto
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <vector>

#include "helpers/PcesProto.h"

namespace P = PcesProto;

static std::vector<uint8_t> hex(const char* s) {
  std::vector<uint8_t> v;
  int hi = -1;
  for (; *s; ++s) {
    int n = (*s >= '0' && *s <= '9') ? *s - '0' : (*s >= 'A' && *s <= 'F') ? *s - 'A' + 10
          : (*s >= 'a' && *s <= 'f') ? *s - 'a' + 10 : -1;
    if (n < 0) continue;
    if (hi < 0) hi = n; else { v.push_back((uint8_t)(hi << 4 | n)); hi = -1; }
  }
  return v;
}
static bool same(const uint8_t* a, size_t n, const char* h) {
  std::vector<uint8_t> b = hex(h);
  return b.size() == n && memcmp(a, b.data(), n) == 0;
}

int main() {
  uint8_t buf[32];

  // The spec's worked example: best path from repeater 3A7D.. to the station, 2-byte hops.
  {
    const uint8_t a[] = { 0x3A, 0x7D };
    const size_t n = P::encodePath(buf, a, 2, nullptr, 0, 2);
    assert(same(buf, n, "C5 01 01 02 02 3A 7D 00"));
    std::vector<uint8_t> r = hex("01 01 00 50 1D 02 02 3A 7D B4 1E");   // after the 4-byte tag
    P::PathAnswer pa;
    assert(P::decodePath(r.data(), r.size(), 2, pa));
    assert(pa.status == P::ST_OK && pa.metric == 80 && pa.weakest_snr4 == 29);   // +7.25 dB
    assert(pa.ends_at_station && !pa.starts_at_station);
    assert(pa.hops == 2 && same(pa.hop, 4, "3A7D B41E"));
  }
  // Error statuses carry no body; a short or foreign reply is malformed, never read past.
  {
    P::PathAnswer pa;
    std::vector<uint8_t> r = hex("01 01 01");
    assert(!P::decodePath(r.data(), r.size(), 2, pa) && pa.status == P::ST_NO_PATH);
    r = hex("01 01 00 50 1D 00 05 3A 7D");   // claims 5 hops, carries 1
    assert(!P::decodePath(r.data(), r.size(), 2, pa) && pa.status == P::ST_MALFORMED);
    r = hex("02 01 00");                     // another version
    assert(!P::decodePath(r.data(), r.size(), 2, pa) && pa.status == P::ST_MALFORMED);
    r = hex("01 02 00 50");                  // a LINK answer is not a PATH one
    assert(!P::decodePath(r.data(), r.size(), 2, pa) && pa.status == P::ST_MALFORMED);
    assert(!P::decodePath(nullptr, 0, 2, pa));
    r = hex("01 01 00 50 80 00 00");
    assert(P::decodePath(r.data(), r.size(), 2, pa) && pa.weakest_snr4 == P::SNR_NONE && pa.hops == 0);
  }
  // LINK and INFO.
  {
    const uint8_t a[] = { 0x3A, 0x7D }, b[] = { 0xB4, 0x1E };
    assert(same(buf, P::encodeLink(buf, a, 2, b, 2), "C5 01 02 02 3A 7D 02 B4 1E"));
    assert(same(buf, P::encodeInfo(buf), "C5 01 00"));
    P::LinkAnswer la;
    std::vector<uint8_t> r = hex("01 02 00 50 FF 1D 80 0A 00 00 00 05 00");
    assert(P::decodeLink(r.data(), r.size(), la));
    assert(la.ab_metric == 80 && la.ba_metric == 255 && la.ab_snr4 == 29 && la.ba_snr4 == P::SNR_NONE);
    assert(la.ab_samples == 10 && la.ba_samples == 0 && la.age_min == 5);
    P::InfoAnswer ia;
    r = hex("01 00 00 A8 00 2A 00 03 00 E2 07");
    assert(P::decodeInfo(r.data(), r.size(), ia));
    assert(ia.window_h == 168 && ia.links == 42 && ia.newest_min == 3 && ia.decode_limit4 == -30 && ia.sf == 7);
  }
  // Node lengths are capped at 8 bytes; 0 means the station.
  {
    const uint8_t k[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
    assert(P::encodePath(buf, k, 12, nullptr, 0, 2) == 4 + 1 + 8 + 1);
  }
  // Feature-1 marker.
  assert(P::answersPces(0xC502) && P::answersPces(0xC503) && !P::answersPces(0xC501) && !P::answersPces(0x0002));

  // Planning and installing: chat contact behind relays 3A7D > 9999.
  {
    const uint8_t route[] = { 0x3A, 0x7D, 0x99, 0x99 };
    const uint8_t pub[] = { 0xA1, 0x1C, 0xE0, 0x00 };
    P::Plan p;
    assert(P::makePlan(route, 2, 2, pub, false, p) == P::PLAN_OK);
    assert(p.alen == 2 && same(p.a, 2, "3A7D") && p.blen == 2 && same(p.b, 2, "9999") && !p.b_is_contact);
    assert(same(buf, P::encodePath(buf, p.a, p.alen, p.b, p.blen, p.width), "C5 01 01 02 02 3A 7D 02 99 99"));
    std::vector<uint8_t> r = hex("01 01 00 50 1D 00 03 3A 7D B4 1E 99 99");
    P::PathAnswer pa;
    assert(P::decodePath(r.data(), r.size(), 2, pa));
    uint8_t out[P::MAX_HOPS * P::MAX_WIDTH];
    assert(P::routeFromAnswer(p, pa, out) == 3 && same(out, 6, "3A7D B41E 9999"));   // both ends kept
    // An answer that does not start at our first hop is refused rather than installed.
    r = hex("01 01 00 50 1D 00 02 B4 1E 99 99");
    assert(P::decodePath(r.data(), r.size(), 2, pa) && P::routeFromAnswer(p, pa, out) == -1);
    // A different width from the one asked for is refused too.
    pa.width = 1;
    assert(P::routeFromAnswer(p, pa, out) == -1);
  }
  // Repeater contact: B is the contact itself, which is not on its own route.
  {
    const uint8_t route[] = { 0x3A, 0x7D };
    const uint8_t pub[] = { 0xB4, 0x1E, 0x33, 0x44 };
    P::Plan p;
    assert(P::makePlan(route, 1, 2, pub, true, p) == P::PLAN_OK);
    assert(p.blen == 4 && same(p.b, 4, "B41E3344") && p.b_is_contact);
    std::vector<uint8_t> r = hex("01 01 00 3C 10 00 03 3A 7D 55 55 B4 1E");
    P::PathAnswer pa;
    assert(P::decodePath(r.data(), r.size(), 2, pa));
    uint8_t out[P::MAX_HOPS * P::MAX_WIDTH];
    assert(P::routeFromAnswer(p, pa, out) == 2 && same(out, 4, "3A7D 5555"));
  }
  // What cannot be planned.
  {
    const uint8_t one[] = { 0x3A, 0x7D }, pub[] = { 1, 2, 3, 4 };
    P::Plan p;
    assert(P::makePlan(one, 0, 2, pub, false, p) == P::PLAN_NO_ROUTE);
    assert(P::makePlan(one, 1, 2, pub, false, p) == P::PLAN_NO_LAST_RELAY);
    assert(P::makePlan(one, 1, 0, pub, true, p) == P::PLAN_BAD_WIDTH);
    assert(P::makePlan(one, 1, 4, pub, true, p) == P::PLAN_BAD_WIDTH);
  }
  // Every status has text.
  for (int s = 0; s <= 9; ++s) assert(P::statusText((uint8_t)s)[0]);

  printf("test_pces_proto: all passed\n");
  return 0;
}
