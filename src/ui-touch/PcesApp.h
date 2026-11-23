// SPDX-License-Identifier: GPL-3.0-or-later
// PCES: Commsol's Path Computation Element Service, the client side.
//
// One page, two ways in: the "PCES" tile in the app drawer (pick a target, then
// Start), and "PCES route" on a contact's action sheet (the same page, that contact
// already the target). A run is the four-step test the uConsole Console's PCES page
// does (meshcore-uconsole, meshcore/pces.py):
//
//   1. First hop R: the target's current first hop, then repeaters this radio hears
//      directly (Discover hits by SNR, then 0-hop repeater contacts). Up to 3, because
//      PCES must know R.
//   2. Server: a guest login to the PCES server (blank password = PCES-only guest at a
//      station) and INFO: is it there, how old is its data, what is its decode limit.
//   3. Route: PATH from R to the target -- a repeater / room: itself; a companion, which
//      never relays: its last relay. The next R is tried when PCES does not know one.
//   4. Accept: PCES's own data must back the route -- its weakest leg MARGIN_DB above the
//      decode limit INFO reported, and INFO's newest measurement at most MAX_DATA_AGE_MIN
//      old. Accepted: installed as the target's out path, the previous one kept for
//      Restore. Not accepted: shown, with "Use anyway".
//   (The uConsole also test-sends over the new route; here it stays "unverified" until
//   use confirms it -- MeshCore floods and relearns a route that stops delivering.)
//
// Automatic: when a PCES server's advert reaches us through relays, the client asks
// that server for the best path to itself, with the relay that handed us the advert as
// A (pcesAutoTick). Once per server per 6 hours, single attempt, only while nothing else
// holds the request slot. Installed when the server has no route yet; otherwise kept for
// the page.
//
// On-air etiquette, from the Commsol spec:
//   * nothing is sent except on Find Best Route / Server info, or a PCES server's advert -- never on
//     a timer and never in a loop over contacts;
//   * no request is retried; a run tries at most 3 first hops, each once;
//   * at most one PCES request every 12 s (the server answers 5 a minute per client).
//
// Native firmware on purpose: Lua apps cannot transmit requests to nodes. It reuses what
// the repeater admin app sends (uiSendAdminLogin, uiSendBinaryRequest); replies are routed
// here from UITask::onAdminLoginResult / onBinaryReply. Codec and planning:
// src/helpers/PcesProto.h (host test test/test_pces_proto.cpp). Notes: PCES.md.
//
// NOT a normal header: UITask.cpp includes it once, right after RepeaterAdminApp.h, and
// it shares that translation unit's static helpers (settings kit, app page, popups).
#pragma once
#include <stdarg.h>
#include "helpers/PcesProto.h"
#include "helpers/PcesStats.h"

namespace {
namespace PP = PcesProto;

static constexpr uint32_t kPcesReqGapMs     = 12000;   // 5 a minute, the server's limit
static constexpr uint32_t kPcesLoginWaitMs  = 20000;
static constexpr uint32_t kPcesReplyWaitMs  = 40000;   // under the UI slot's 45 s expiry
static constexpr uint32_t kPcesAfterLoginMs = 2500;    // let the re-flooded login reply pass (#683)
static constexpr uint32_t kPcesSessionMs    = 15u * 60u * 1000u;
static constexpr uint32_t kPcesAutoEveryMs  = 6u * 60u * 60u * 1000u;   // re-ask a server after 6 h
static constexpr uint32_t kPcesAutoHoldMs   = 60000;   // a queued advert job waits this long
static constexpr int      kPcesMarginQ4     = 12;      // MARGIN_DB 3.0, in SNR x4 units
static constexpr uint16_t kPcesMaxDataAgeMin = 60;     // MAX_DATA_AGE_MIN
static constexpr int      kPcesMaxFirstHops = 3;       // MAX_FIRST_HOPS
static constexpr int      kPcesLogMax       = 10;
static constexpr int      kPcesAcceptedMax  = 8;
static constexpr uint32_t kPcesTraceWaitMs  = 20000;   // out and back along the route
static constexpr uint32_t kPcesTraceGapMs   = 15000;   // every hop answers: same floor as a Lua ping
static constexpr int      kPcesBenchMax     = 100;     // samples per route, at most
static const uint8_t  kPcesBenchRounds[]  = { 10, 25, 50, 100 };   // per route
static const uint8_t  kPcesBenchGapS[]    = { 15, 30, 60 };        // between traces
static constexpr uint32_t kPcesDiscoverWaitMs = 30000;   // a flooded discovery's reply

enum PcesRun : uint8_t { PRUN_NONE = 0, PRUN_TEST, PRUN_INFO, PRUN_AUTO };
// What the run waits for / does next.
enum PcesStep : uint8_t { PST_IDLE = 0, PST_LOGIN_WAIT, PST_INFO_SEND, PST_INFO_WAIT, PST_PATH_SEND, PST_PATH_WAIT };
enum PcesView : uint8_t { PV_MAIN = 0, PV_PICK };

struct PcesFirstHop { uint8_t hop[PP::MAX_WIDTH]; char name[24]; };
struct PcesAccepted {
  bool     used;
  uint8_t  pub[PUB_KEY_SIZE];
  uint8_t  old_len;                      // raw out_path_len before (OUT_PATH_UNKNOWN = none)
  uint8_t  old_path[MAX_PATH_SIZE];
  char     route_txt[48];
  uint8_t  metric;
  bool     backed;                       // passed step 4 (else "Use anyway")
  uint8_t  new_len;                      // the route PCES installed (raw out_path_len)
  uint8_t  new_path[MAX_PATH_SIZE];
};
// "Compare routes": the default route vs the PCES route, by traces along each.
struct PcesSample { uint16_t rtt_ms; int8_t weakest4; uint8_t ok; };
struct PcesCmpSide { uint8_t len; uint8_t path[MAX_PATH_SIZE]; uint8_t sent, back; PcesSample s[kPcesBenchMax]; };

struct PcesState {
  bool     open;
  uint8_t  view;
  // Server.
  uint8_t  server[PUB_KEY_SIZE];
  bool     have_server, server_marked, server_saved, prefs_loaded;
  uint8_t  session_server[PUB_KEY_SIZE];
  uint32_t session_ms;
  bool     session_valid;
  // Target.
  uint8_t  target[PUB_KEY_SIZE];
  bool     have_target;
  // The run in progress.
  uint8_t  run, step, next_step;          // next_step: what follows a login
  uint8_t  req_server[PUB_KEY_SIZE];     // who the login / request in flight went to
  uint32_t deadline, send_at, last_req_ms;
  bool     have_last_req;
  uint8_t  req[24], req_len;
  PcesFirstHop hops[kPcesMaxFirstHops];
  uint8_t  n_hops, hop_i, width;
  bool     b_is_contact;
  uint8_t  b[PP::MAX_NODE], blen;
  PP::Plan plan;
  bool     have_info;
  PP::InfoAnswer info;
  bool     have_answer;
  PP::PathAnswer ans;
  uint8_t  answer_for[PUB_KEY_SIZE];
  uint8_t  route[PP::MAX_HOPS * PP::MAX_WIDTH];
  int      route_hops;                   // -1: the answer does not make a route
  bool     can_use_anyway;
  // Log of the last run, shown on the page.
  struct { char text[96]; uint8_t tone; } log[kPcesLogMax];
  uint8_t  n_log;
  // Routes PCES installed, for Restore (RAM: gone after a reboot).
  PcesAccepted accepted[kPcesAcceptedMax];
  // Advert-triggered asks.
  bool     auto_pending;
  uint8_t  auto_pub[PUB_KEY_SIZE], auto_hop[3], auto_width;
  uint32_t auto_seen_ms;
  struct { uint8_t pub4[4]; uint32_t ms; bool used; } asked[8];
  uint8_t  asked_next;
  bool     dirty;
  // "Trace route": out along the target's route and back, one SNR per hop.
  uint32_t trace_tag, trace_ms, trace_last_ms;
  // The page is rebuilt on every update: keep where the user had scrolled to.
  lv_coord_t keep_scroll;
  uint8_t  rendered_view;
  // Compare routes.
  bool     cmp_active, cmp_done, cmp_waiting;
  uint16_t cmp_i;                        // turns taken so far (even: default, odd: PCES)
  uint32_t cmp_next_at, cmp_t0;
  uint8_t  bench_rounds_i, bench_gap_i;  // index into kPcesBenchRounds / kPcesBenchGapS
  char     bench_csv[64];                // the CSV this run appends to ("" = not saved)
  bool     bench_discovering;            // finding the default path first (flooded discovery)
  uint32_t bench_disc_deadline;
  char     bench_msg[120];               // why a benchmark did not start, or what it is doing
  uint8_t  cmp_for[PUB_KEY_SIZE];
  PcesCmpSide cmp[2];                    // 0 = default, 1 = PCES
};
PcesState* s_pces = nullptr;   // psAlloc: zero-initialised
lv_obj_t*  s_pces_root = nullptr;
lv_obj_t*  s_pces_page = nullptr;
lv_timer_t* s_pces_timer = nullptr;   // the service tick: runs whether or not the page is open
char       s_pces_title[48] = "";

void pcesRender();   // fwd
void pcesBack();     // fwd
void pcesInstall(bool backed);   // fwd
void pcesCompareTick(uint32_t now);   // fwd
void pcesBenchMissed();              // fwd
}  // namespace
// Defined in UITask.cpp beside the UI data filesystem: appends a line to `path` (made
// relative to the UI data root, directories created); `fresh` truncates first.
static bool pcesCsvWrite(const char* path, const char* line, bool fresh);
static void pcesCsvFullPath(const char* rel, char* out, size_t cap);
namespace {

// ---- small helpers -------------------------------------------------------------------------
void pcesLog(uint8_t tone, const char* fmt, ...) {
  if (s_pces->n_log >= kPcesLogMax) {   // keep the newest
    memmove(&s_pces->log[0], &s_pces->log[1], sizeof(s_pces->log[0]) * (kPcesLogMax - 1));
    s_pces->n_log = kPcesLogMax - 1;
  }
  auto& l = s_pces->log[s_pces->n_log++];
  va_list ap; va_start(ap, fmt);
  vsnprintf(l.text, sizeof l.text, fmt, ap);
  va_end(ap);
  l.tone = tone;
  s_pces->dirty = true;
}
void pcesLogClear() { s_pces->n_log = 0; }

ContactInfo* pcesTarget() {
  return s_pces->have_target ? the_mesh.lookupContactByPubKey(s_pces->target, PUB_KEY_SIZE) : nullptr;
}
ContactInfo* pcesServer() {
  return s_pces->have_server ? the_mesh.lookupContactByPubKey(s_pces->server, PUB_KEY_SIZE) : nullptr;
}
bool pcesIsRelay(const ContactInfo& c) { return c.type == ADV_TYPE_REPEATER || c.type == ADV_TYPE_ROOM; }
bool pcesHasPath(const ContactInfo& c) { return c.out_path_len != OUT_PATH_UNKNOWN && mesh::Packet::isValidPathLen(c.out_path_len); }
const char* pcesName(const ContactInfo& c) { return contactShownName(c.id.pub_key, c.name); }

void pcesHex(const uint8_t* b, int n, char* out, size_t cap) {
  size_t o = 0; out[0] = 0;
  for (int i = 0; i < n && o + 3 <= cap; ++i) o += snprintf(out + o, cap - o, "%02X", b[i]);
}
// "3A7D > B41E", or the route's state when it has no relays.
void pcesFmtRoute(const uint8_t* path, int hops, int width, char* out, size_t cap) {
  if (hops < 0) { snprintf(out, cap, "%s", TR("None (floods)")); return; }
  if (hops == 0) { snprintf(out, cap, "%s", TR("Direct")); return; }
  size_t o = 0; out[0] = 0;
  for (int i = 0; i < hops && o + 12 < cap; ++i) {
    if (i) o += snprintf(out + o, cap - o, " > ");
    for (int k = 0; k < width; ++k) o += snprintf(out + o, cap - o, "%02X", path[i * width + k]);
  }
}
void pcesFmtContactRoute(const ContactInfo& c, char* out, size_t cap) {
  if (!pcesHasPath(c)) { pcesFmtRoute(nullptr, -1, 1, out, cap); return; }
  pcesFmtRoute(c.out_path, c.out_path_len & 63, (c.out_path_len >> 6) + 1, out, cap);
}
void pcesFmtSnr(int8_t snr4, char* out, size_t cap) {
  if (snr4 == PP::SNR_NONE) snprintf(out, cap, "-");
  else snprintf(out, cap, "%+.2f dB", snr4 / 4.0);
}

// ---- server --------------------------------------------------------------------------------
void pcesLoadSaved() {
  if (s_pces->prefs_loaded) return;
  s_pces->prefs_loaded = true;
#if defined(ESP32)
  uint8_t k[PUB_KEY_SIZE];
  if (touchPrefsGetPcesServer(k)) {
    memcpy(s_pces->server, k, PUB_KEY_SIZE);
    s_pces->have_server = s_pces->server_saved = true;
  }
#endif
}
void pcesSaveServer(const uint8_t* pub) {   // null = forget
#if defined(ESP32)
  touchPrefsSetPcesServer(pub);
#else
  (void)pub;
#endif
  s_pces->server_saved = pub != nullptr;
}
// The saved server if there is one (even while it is not a contact: the user chose
// it), else the first contact advertising "answers PCES".
void pcesFindServer() {
  pcesLoadSaved();
  if (s_pces->server_saved) {
    s_pces->server_marked = PP::answersPces(the_mesh.advertFeat1(s_pces->server));
    return;
  }
  if (s_pces->have_server && pcesServer()) {
    s_pces->server_marked = PP::answersPces(the_mesh.advertFeat1(s_pces->server));
    return;
  }
  s_pces->have_server = false;
  const uint32_t n = the_mesh.getNumContacts();
  for (uint32_t i = 0; i < n; ++i) {
    ContactInfo c;
    if (!the_mesh.getContactByIdx(i, c) || !pcesIsRelay(c)) continue;
    if (PP::answersPces(the_mesh.advertFeat1(c.id.pub_key))) {
      memcpy(s_pces->server, c.id.pub_key, PUB_KEY_SIZE);
      s_pces->have_server = s_pces->server_marked = true;
      return;
    }
  }
}

// ---- the request engine --------------------------------------------------------------------
bool pcesSessionFresh(const uint8_t* srv) {
  return s_pces->session_valid && memcmp(s_pces->session_server, srv, PUB_KEY_SIZE) == 0 &&
         (uint32_t)(millis() - s_pces->session_ms) < kPcesSessionMs;
}
// The earliest moment the next PCES request may go: 12 s after the last one.
uint32_t pcesFloorAt() {
  return s_pces->have_last_req ? s_pces->last_req_ms + kPcesReqGapMs : millis();
}
void pcesEndRun() {
  if (s_pces->step == PST_INFO_WAIT || s_pces->step == PST_PATH_WAIT) the_mesh.cancelUIPingPending();
  s_pces->run = PRUN_NONE;
  s_pces->step = PST_IDLE;
  s_pces->dirty = true;
}
// Log in (unless a session is fresh), then do `next` at the next allowed moment.
void pcesBegin(uint8_t run, ContactInfo* srv, uint8_t next) {
  s_pces->run = run;
  memcpy(s_pces->req_server, srv->id.pub_key, PUB_KEY_SIZE);
  if (pcesSessionFresh(srv->id.pub_key)) {
    s_pces->step = next;
    s_pces->send_at = pcesFloorAt();
    return;
  }
  // Blank = PCES-only guest at a station. Keeps the stored route: the admin app's login
  // wipes it first, which would throw away a route PCES itself installed to the server.
  const int r = the_mesh.uiSendGuestLoginKeepPath(*srv);
  if (r != MSG_SEND_SENT_FLOOD && r != MSG_SEND_SENT_DIRECT) {
    if (run != PRUN_AUTO) pcesLog(SET_TONE_DANGER, "%s", TR("Login not sent: radio busy."));
    pcesEndRun();
    return;
  }
  s_pces->step = PST_LOGIN_WAIT;
  s_pces->deadline = millis() + kPcesLoginWaitMs;
  s_pces->next_step = next;
  if (run != PRUN_AUTO) pcesLog(SET_TONE_INFO, TR("Logging in to %s as a guest..."), pcesName(*srv));
}
bool pcesSend(uint8_t wait_step) {
  ContactInfo* srv = the_mesh.lookupContactByPubKey(s_pces->req_server, PUB_KEY_SIZE);
  if (!srv || the_mesh.hasUIRequestPending()) return false;
  const int r = the_mesh.uiSendBinaryRequest(*srv, s_pces->req, s_pces->req_len);
  if (r != MSG_SEND_SENT_FLOOD && r != MSG_SEND_SENT_DIRECT) return false;
  s_pces->step = wait_step;
  s_pces->deadline = millis() + kPcesReplyWaitMs;
  s_pces->last_req_ms = millis(); s_pces->have_last_req = true;
  return true;
}

// ---- step 1: first-hop candidates ----------------------------------------------------------
void pcesAddHop(const uint8_t* hop, const char* name) {
  if (s_pces->n_hops >= kPcesMaxFirstHops) return;
  for (int i = 0; i < s_pces->n_hops; ++i)
    if (memcmp(s_pces->hops[i].hop, hop, s_pces->width) == 0) return;
  // The target itself is never its own first hop.
  if (s_pces->b_is_contact && memcmp(hop, s_pces->target, s_pces->width) == 0) return;
  auto& h = s_pces->hops[s_pces->n_hops++];
  memcpy(h.hop, hop, s_pces->width);
  snprintf(h.name, sizeof h.name, "%s", name ? name : "");
}
const char* pcesNameForHop(const uint8_t* hop, uint8_t width) {
  const uint32_t n = the_mesh.getNumContacts();
  for (uint32_t i = 0; i < n; ++i) {
    ContactInfo c;
    if (the_mesh.getContactByIdx(i, c) && c.type == ADV_TYPE_REPEATER && memcmp(c.id.pub_key, hop, width) == 0)
      return contactShownName(c.id.pub_key, c.name);
  }
  return nullptr;
}
void pcesFindFirstHops(const ContactInfo& t) {
  s_pces->n_hops = 0;
  // The target's current first hop, if it has a route through relays.
  if (pcesHasPath(t) && (t.out_path_len & 63) > 0) pcesAddHop(t.out_path, pcesNameForHop(t.out_path, s_pces->width));
  // Repeaters this radio hears directly: Discover hits (heard direct), best SNR first.
  {
    MyMesh::DiscoverHit best[8]; int nb = 0;
    for (uint8_t i = 0; i < the_mesh.discoverCount() && nb < 8; ++i) {
      MyMesh::DiscoverHit h;
      if (the_mesh.discoverGet(i, h) && h.node_type == ADV_TYPE_REPEATER && h.path_len == 0) best[nb++] = h;
    }
    for (int i = 0; i < nb; ++i)
      for (int j = i + 1; j < nb; ++j)
        if (best[j].our_snr_q4 > best[i].our_snr_q4) { MyMesh::DiscoverHit x = best[i]; best[i] = best[j]; best[j] = x; }
    for (int i = 0; i < nb; ++i) {
      ContactInfo* c = the_mesh.lookupContactByPubKey(best[i].pubkey, PUB_KEY_SIZE);
      pcesAddHop(best[i].pubkey, c ? pcesName(*c) : nullptr);
    }
  }
  // Repeater contacts we reach directly (0-hop route), most recently heard first.
  {
    const uint32_t n = the_mesh.getNumContacts();
    for (int pass = 0; pass < kPcesMaxFirstHops && s_pces->n_hops < kPcesMaxFirstHops; ++pass) {
      int best = -1; uint32_t best_ts = 0; ContactInfo bc;
      for (uint32_t i = 0; i < n; ++i) {
        ContactInfo c;
        if (!the_mesh.getContactByIdx(i, c) || c.type != ADV_TYPE_REPEATER || c.out_path_len != 0) continue;
        bool taken = false;
        for (int k = 0; k < s_pces->n_hops; ++k) if (memcmp(s_pces->hops[k].hop, c.id.pub_key, s_pces->width) == 0) taken = true;
        if (taken) continue;
        if (best < 0 || c.last_advert_timestamp > best_ts) { best = (int)i; best_ts = c.last_advert_timestamp; bc = c; }
      }
      if (best < 0) break;
      pcesAddHop(bc.id.pub_key, pcesName(bc));
    }
  }
}

// ---- a test run -----------------------------------------------------------------------------
void pcesStartTest() {
  ContactInfo* t = pcesTarget();
  ContactInfo* srv = pcesServer();
  pcesLogClear();
  s_pces->have_answer = false; s_pces->can_use_anyway = false; s_pces->have_info = false;
  if (!t) { pcesLog(SET_TONE_WARN, "%s", TR("Choose a target first.")); return; }
  if (!srv) { pcesLog(SET_TONE_WARN, "%s", TR("No PCES server: pick one under PCES server.")); return; }
  if (memcmp(t->id.pub_key, srv->id.pub_key, PUB_KEY_SIZE) == 0) {
    pcesLog(SET_TONE_WARN, "%s", TR("The target is the PCES server itself: its route is asked automatically when its advert arrives."));
    return;
  }
  // Width: the target's stored hash size, else the one this node floods with.
  s_pces->width = pcesHasPath(*t) ? (uint8_t)((t->out_path_len >> 6) + 1)
                                  : (uint8_t)(the_mesh.getNodePrefs()->path_hash_mode + 1);
  if (s_pces->width > PP::MAX_WIDTH) s_pces->width = PP::MAX_WIDTH;
  // B: a relay is its own end; a companion's is its last relay.
  s_pces->b_is_contact = pcesIsRelay(*t);
  if (s_pces->b_is_contact) {
    memcpy(s_pces->b, t->id.pub_key, 4); s_pces->blen = 4;
  } else {
    const uint8_t th = pcesHasPath(*t) ? (uint8_t)(t->out_path_len & 63) : 0;
    if (th == 0) {
      pcesLog(SET_TONE_WARN, "%s", TR("A companion's route ends at its last relay, and none is known yet. Message it once so the mesh learns one, then tap Find Best Route."));
      return;
    }
    memcpy(s_pces->b, t->out_path + (size_t)(th - 1) * s_pces->width, s_pces->width);
    s_pces->blen = s_pces->width;
  }
  pcesFindFirstHops(*t);
  char hx[12], line[96];
  if (s_pces->n_hops == 0) {
    pcesLog(SET_TONE_WARN, "%s", TR("1. First hop: no repeater is heard directly. Run Discover (or wait for adverts), then tap Find Best Route."));
    return;
  }
  size_t o = snprintf(line, sizeof line, "%s", TR("1. First hop:"));
  for (int i = 0; i < s_pces->n_hops; ++i) {
    pcesHex(s_pces->hops[i].hop, s_pces->width, hx, sizeof hx);
    o += snprintf(line + o, sizeof line - o, "%s %s%s%s%s", i ? "," : "", hx,
                  s_pces->hops[i].name[0] ? " (" : "", s_pces->hops[i].name, s_pces->hops[i].name[0] ? ")" : "");
    if (o >= sizeof line) break;
  }
  pcesLog(SET_TONE_HELP, "%s", line);
  s_pces->hop_i = 0;
  memcpy(s_pces->answer_for, t->id.pub_key, PUB_KEY_SIZE);
  pcesBegin(PRUN_TEST, srv, PST_INFO_SEND);
}

void pcesBuildPath(const uint8_t* a, uint8_t width, const uint8_t* b, uint8_t blen, bool b_is_contact) {
  PP::Plan& p = s_pces->plan;
  p = PP::Plan();
  p.width = width;
  memcpy(p.a, a, width); p.alen = width;
  if (blen) memcpy(p.b, b, blen);
  p.blen = blen;
  p.b_is_contact = b_is_contact;
  s_pces->req_len = (uint8_t)PP::encodePath(s_pces->req, p.a, p.alen, p.b, p.blen, p.width);
}

// Step 4 for a test run: does PCES's own data back the route?
void pcesAccept() {
  ContactInfo* t = the_mesh.lookupContactByPubKey(s_pces->answer_for, PUB_KEY_SIZE);
  if (!t) { pcesLog(SET_TONE_DANGER, "%s", TR("4. Target is no longer a contact.")); return; }
  const PP::PathAnswer& a = s_pces->ans;
  char snr[16], lim[16], rt[64];
  pcesFmtSnr(a.weakest_snr4, snr, sizeof snr);
  pcesFmtRoute(s_pces->route, s_pces->route_hops, s_pces->plan.width, rt, sizeof rt);
  pcesLog(SET_TONE_HELP, TR("3. Route %s, metric %u, weakest leg %s"), rt, (unsigned)a.metric, snr);
  const char* why = nullptr;
  char whyb[96];
  if (!s_pces->have_info) why = TR("no INFO from the server to judge it by");
  else if (a.weakest_snr4 == PP::SNR_NONE) why = TR("the weakest leg has no SNR");
  else if ((int)a.weakest_snr4 < (int)s_pces->info.decode_limit4 + kPcesMarginQ4) {
    pcesFmtSnr(s_pces->info.decode_limit4, lim, sizeof lim);
    snprintf(whyb, sizeof whyb, TR("weakest leg %s is less than 3 dB above the decode limit %s"), snr, lim);
    why = whyb;
  } else if (s_pces->info.newest_min > kPcesMaxDataAgeMin) {
    snprintf(whyb, sizeof whyb, TR("PCES's newest measurement is %u min old (max %u)"),
             (unsigned)s_pces->info.newest_min, (unsigned)kPcesMaxDataAgeMin);
    why = whyb;
  }
  if (why) {
    pcesLog(SET_TONE_WARN, TR("4. Not accepted: %s."), why);
    s_pces->can_use_anyway = true;
    return;
  }
  pcesInstall(true);
}

// Install the answer as the target's route, keeping the old one for Restore.
void pcesInstall(bool backed) {
  ContactInfo* t = the_mesh.lookupContactByPubKey(s_pces->answer_for, PUB_KEY_SIZE);
  if (!t || s_pces->route_hops < 0) return;
  PcesAccepted* slot = nullptr;
  for (auto& e : s_pces->accepted) if (e.used && memcmp(e.pub, t->id.pub_key, PUB_KEY_SIZE) == 0) slot = &e;   // keep the ORIGINAL
  if (!slot) {
    for (auto& e : s_pces->accepted) if (!e.used) { slot = &e; break; }
    if (!slot) {   // full: drop the oldest
      memmove(&s_pces->accepted[0], &s_pces->accepted[1], sizeof(PcesAccepted) * (kPcesAcceptedMax - 1));
      slot = &s_pces->accepted[kPcesAcceptedMax - 1];
    }
    memset(slot, 0, sizeof *slot);
    slot->used = true;
    memcpy(slot->pub, t->id.pub_key, PUB_KEY_SIZE);
    slot->old_len = t->out_path_len;
    memcpy(slot->old_path, t->out_path, sizeof slot->old_path);
  }
  pcesFmtRoute(s_pces->route, s_pces->route_hops, s_pces->plan.width, slot->route_txt, sizeof slot->route_txt);
  slot->metric = s_pces->ans.metric;
  slot->backed = backed;
  if (the_mesh.uiSetContactPath(t->id.pub_key, s_pces->route, (uint8_t)s_pces->route_hops, s_pces->plan.width)) {
    slot->new_len = t->out_path_len;
    memcpy(slot->new_path, t->out_path, sizeof slot->new_path);
    the_mesh.uiPinRoute(t->id.pub_key, true, s_pces->ans.metric);   // a failed Ping or a login must not wipe it
    contactsListMarkStale();                    // the "PCES <score>" badge
    pcesLog(SET_TONE_OK, "%s", backed ? TR("4. Accepted and installed (unverified until used). Restore puts the old route back.")
                                      : TR("Installed anyway (unverified). Restore puts the old route back."));
  } else {
    pcesLog(SET_TONE_DANGER, "%s", TR("Route not installed: it does not fit."));
  }
  s_pces->can_use_anyway = false;
  s_pces->dirty = true;
}

// ---- automatic: best path to a PCES server whose advert we heard ---------------------------
bool pcesAskedRecently(const uint8_t* pub) {
  for (auto& a : s_pces->asked)
    if (a.used && memcmp(a.pub4, pub, 4) == 0) return (uint32_t)(millis() - a.ms) < kPcesAutoEveryMs;
  return false;
}
void pcesMarkAsked(const uint8_t* pub) {
  for (auto& a : s_pces->asked)
    if (a.used && memcmp(a.pub4, pub, 4) == 0) { a.ms = millis(); return; }
  auto& a = s_pces->asked[s_pces->asked_next];
  s_pces->asked_next = (uint8_t)((s_pces->asked_next + 1) % 8);
  memcpy(a.pub4, pub, 4); a.ms = millis(); a.used = true;
}
void pcesAutoAnswer() {
  ContactInfo* srv = the_mesh.lookupContactByPubKey(s_pces->req_server, PUB_KEY_SIZE);
  if (!srv || !g_lv.task || s_pces->route_hops < 0) return;
  memcpy(s_pces->answer_for, srv->id.pub_key, PUB_KEY_SIZE);
  s_pces->have_answer = true;
  char msg[96];
  if (srv->out_path_len == OUT_PATH_UNKNOWN) {
    pcesInstall(true);
    snprintf(msg, sizeof msg, TR("PCES: route to %s installed (metric %u)"), pcesName(*srv), (unsigned)s_pces->ans.metric);
  } else {
    snprintf(msg, sizeof msg, TR("PCES: best path to %s, metric %u. See PCES."), pcesName(*srv), (unsigned)s_pces->ans.metric);
  }
  g_lv.task->showAlert(msg, 2600);
}
void pcesAutoTick() {
  uint8_t pub[PUB_KEY_SIZE], hop[3], width = 0, hops = 0;
  if (the_mesh.takePcesAdvert(pub, hop, &width, &hops)) {
    if (!pcesAskedRecently(pub) && the_mesh.lookupContactByPubKey(pub, PUB_KEY_SIZE)) {
      memcpy(s_pces->auto_pub, pub, PUB_KEY_SIZE);
      memcpy(s_pces->auto_hop, hop, 3);
      s_pces->auto_width = width;
      s_pces->auto_seen_ms = millis();
      s_pces->auto_pending = true;
    }
  }
  if (!s_pces->auto_pending) return;
  if ((uint32_t)(millis() - s_pces->auto_seen_ms) > kPcesAutoHoldMs) { s_pces->auto_pending = false; return; }
  // Never under the user: not while the page is open or anything is in flight.
  if (s_pces->open || s_pces->run != PRUN_NONE || the_mesh.hasUIRequestPending()) return;
  if ((int32_t)(millis() - pcesFloorAt()) < 0) return;
  ContactInfo* srv = the_mesh.lookupContactByPubKey(s_pces->auto_pub, PUB_KEY_SIZE);
  s_pces->auto_pending = false;
  if (!srv) return;
  pcesMarkAsked(s_pces->auto_pub);   // asked, answered or not: once per server per 6 h
  s_pces->width = s_pces->auto_width;
  s_pces->have_answer = false;
  pcesBuildPath(s_pces->auto_hop, s_pces->auto_width, nullptr, 0, false);   // B = the station
  pcesBegin(PRUN_AUTO, srv, PST_PATH_SEND);
}

// ---- replies (from UITask's mesh callbacks) ---------------------------------------------
bool pcesOnLoginResult(const ContactInfo& contact, bool success, uint8_t) {
  if (!s_pces || s_pces->step != PST_LOGIN_WAIT ||
      memcmp(contact.id.pub_key, s_pces->req_server, PUB_KEY_SIZE) != 0) return false;
  if (!success) {
    if (s_pces->run != PRUN_AUTO) pcesLog(SET_TONE_DANGER, "%s", TR("Login refused by the PCES server."));
    pcesEndRun();
    return true;
  }
  memcpy(s_pces->session_server, s_pces->req_server, PUB_KEY_SIZE);
  s_pces->session_ms = millis(); s_pces->session_valid = true;
  s_pces->step = s_pces->next_step;   // what pcesBegin was asked to do next
  const uint32_t after = millis() + kPcesAfterLoginMs, floor_at = pcesFloorAt();
  s_pces->send_at = (int32_t)(after - floor_at) > 0 ? after : floor_at;
  if (s_pces->run != PRUN_AUTO) pcesLog(SET_TONE_HELP, "%s", TR("Logged in."));
  s_pces->dirty = true;
  return true;
}

bool pcesOnBinaryReply(const ContactInfo& contact, const uint8_t* data, size_t len) {
  if (!s_pces || (s_pces->step != PST_INFO_WAIT && s_pces->step != PST_PATH_WAIT) ||
      memcmp(contact.id.pub_key, s_pces->req_server, PUB_KEY_SIZE) != 0) return false;
  s_pces->dirty = true;
  if (s_pces->step == PST_INFO_WAIT) {
    if (PP::decodeInfo(data, len, s_pces->info)) {
      s_pces->have_info = true;
      const PP::InfoAnswer& i = s_pces->info;
      char lim[16]; pcesFmtSnr(i.decode_limit4, lim, sizeof lim);
      pcesLog(SET_TONE_HELP, TR("2. Server: %u links over %u h, newest %u min old, SF%u, limit %s"),
              (unsigned)i.links, (unsigned)i.window_h, (unsigned)i.newest_min, (unsigned)i.sf, lim);
    } else {
      pcesLog(SET_TONE_WARN, TR("2. INFO: %s"), TR(PP::statusText(s_pces->info.status)));
    }
    if (s_pces->run == PRUN_INFO) { pcesEndRun(); return true; }
    s_pces->step = PST_PATH_SEND;
    s_pces->send_at = pcesFloorAt();
    return true;
  }
  // PATH
  const bool ok = PP::decodePath(data, len, s_pces->plan.width, s_pces->ans);
  if (ok) s_pces->route_hops = PP::routeFromAnswer(s_pces->plan, s_pces->ans, s_pces->route);
  if (s_pces->run == PRUN_AUTO) {
    if (ok) pcesAutoAnswer();
    pcesEndRun();
    return true;
  }
  if (ok && s_pces->route_hops >= 0) {
    s_pces->have_answer = true;
    pcesAccept();
    pcesEndRun();
    return true;
  }
  char hx[12]; pcesHex(s_pces->hops[s_pces->hop_i].hop, s_pces->width, hx, sizeof hx);
  pcesLog(SET_TONE_WARN, TR("3. From %s: %s"), hx,
          ok ? TR("answer does not start at that hop") : TR(PP::statusText(s_pces->ans.status)));
  // PCES does not know this first hop (or has no path from it): try the next one.
  if (++s_pces->hop_i < s_pces->n_hops) {
    s_pces->step = PST_PATH_SEND;
    s_pces->send_at = pcesFloorAt();
  } else {
    pcesLog(SET_TONE_WARN, "%s", TR("No route found from any first hop."));
    pcesEndRun();
  }
  return true;
}

// ---- the service tick ---------------------------------------------------------------------
void pcesTimerCb(lv_timer_t*) {
  if (!s_pces) return;
  const uint32_t now = millis();
  switch (s_pces->step) {
    case PST_LOGIN_WAIT:
    case PST_INFO_WAIT:
    case PST_PATH_WAIT:
      if ((int32_t)(now - s_pces->deadline) >= 0) {
        s_pces->session_valid = false;   // a station restart, or a full client list, drops guests
        if (s_pces->run != PRUN_AUTO)
          pcesLog(SET_TONE_WARN, "%s", TR("No answer from PCES. Not retried: check the station is in range and running."));
        pcesEndRun();
      }
      break;
    case PST_INFO_SEND:
      if ((int32_t)(now - s_pces->send_at) >= 0) {
        s_pces->req_len = (uint8_t)PP::encodeInfo(s_pces->req);
        if (pcesSend(PST_INFO_WAIT)) { if (s_pces->run != PRUN_AUTO) pcesLog(SET_TONE_INFO, "%s", TR("2. Asking the server for INFO...")); }
        else s_pces->send_at = now + 1000;   // the UI slot is busy: try again shortly
        s_pces->dirty = true;
      }
      break;
    case PST_PATH_SEND:
      if ((int32_t)(now - s_pces->send_at) >= 0) {
        if (s_pces->run == PRUN_TEST)
          pcesBuildPath(s_pces->hops[s_pces->hop_i].hop, s_pces->width, s_pces->b, s_pces->blen, s_pces->b_is_contact);
        if (pcesSend(PST_PATH_WAIT)) {
          if (s_pces->run == PRUN_TEST) {
            char a[12], b[20];
            pcesHex(s_pces->plan.a, s_pces->plan.alen, a, sizeof a);
            pcesHex(s_pces->plan.b, s_pces->plan.blen, b, sizeof b);
            pcesLog(SET_TONE_INFO, TR("3. Asking PATH %s > %s..."), a, b);
          }
        } else {
          s_pces->send_at = now + 1000;
        }
        s_pces->dirty = true;
      }
      break;
    default:
      break;
  }
  if (s_pces->trace_tag && (uint32_t)(now - s_pces->trace_ms) >= kPcesTraceWaitMs) {
    s_pces->trace_tag = 0;
    if (s_pces->cmp_waiting) pcesBenchMissed();   // not back
    else pcesLog(SET_TONE_WARN, "%s", TR("Trace: no reply in 20 s. A hop on this route may not be reachable."));
  }
  pcesCompareTick(now);
  if (s_pces->run == PRUN_NONE) pcesAutoTick();
  if (s_pces->dirty) { s_pces->dirty = false; if (s_pces->open) pcesRender(); }
}

// ---- trace route -----------------------------------------------------------------------------
// A TRACE out along `path` (raw out_path_len `len`) to contact `t` and back. A repeater /
// room turns the trace itself; a companion never relays a trace, so its trace turns at the
// route's last relay. Returns the tag (0 = not sent); `why` says why not.
uint32_t pcesTraceAlong(const ContactInfo& t, uint8_t len, const uint8_t* path, const char** why) {
  ContactInfo turn = t;
  turn.out_path_len = len;
  memcpy(turn.out_path, path, sizeof turn.out_path);
  const bool has = len != OUT_PATH_UNKNOWN && mesh::Packet::isValidPathLen(len);
  const uint8_t hops = has ? (uint8_t)(len & 63) : 0;
  if (!pcesIsRelay(t)) {
    if (hops == 0) { *why = TR("no route through relays to trace"); return 0; }
    const uint8_t w = (uint8_t)((len >> 6) + 1);
    memset(turn.id.pub_key, 0, PUB_KEY_SIZE);
    memcpy(turn.id.pub_key, path + (size_t)(hops - 1) * w, w);
    turn.out_path_len = (uint8_t)(((w - 1) << 6) | (hops - 1));
  } else if (!has) {
    *why = TR("it floods: no fixed route to trace");
    return 0;
  }
  const uint32_t tag = the_mesh.uiSendTraceRoute(turn);
  if (!tag) *why = TR("radio busy or route too long");
  return tag;
}
bool pcesTraceBusy() { return s_pces->trace_tag || s_ui_ping_deadline_ms != 0; }

void pcesTraceCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces) return;
  ContactInfo* t = pcesTarget();
  if (!t) return;
  const uint32_t now = millis();
  if (pcesTraceBusy() || s_pces->cmp_active) { pcesLog(SET_TONE_WARN, "%s", TR("A trace or ping is already waiting.")); pcesRender(); return; }
  if (s_pces->trace_last_ms && (uint32_t)(now - s_pces->trace_last_ms) < kPcesTraceGapMs) {
    pcesLog(SET_TONE_WARN, TR("Wait %u s: every hop answers a trace."), (unsigned)((kPcesTraceGapMs - (now - s_pces->trace_last_ms) + 999) / 1000));
    pcesRender(); return;
  }
  const char* why = nullptr;
  const uint32_t tag = pcesTraceAlong(*t, t->out_path_len, t->out_path, &why);
  if (!tag) { pcesLog(SET_TONE_WARN, TR("Trace not sent: %s."), why); pcesRender(); return; }
  char rt[64]; pcesFmtContactRoute(*t, rt, sizeof rt);
  s_pces->trace_tag = tag; s_pces->trace_ms = now; s_pces->trace_last_ms = now;
  pcesLog(SET_TONE_INFO, TR("Trace: out along %s and back..."), rt);
  pcesRender();
}

// ---- benchmark: the default route vs the PCES route (a dev tool) -------------------------
// Traces along each, alternating (default first), N per route, a fixed gap apart. Same test
// both ways: delivered or not, round trip, weakest hop SNR (every hop's reading, out and
// back). Statistics as samples come in (PcesStats.h), every sample appended to a CSV on the
// UI data filesystem for offline analysis. Nothing is installed or changed. Runs with the
// page closed; one trace at a time, shared with "Trace route".
const PcesAccepted* pcesAcceptedFor(const uint8_t* pub) {
  for (const auto& a : s_pces->accepted) if (a.used && memcmp(a.pub, pub, PUB_KEY_SIZE) == 0) return &a;
  return nullptr;
}
// The two routes for a target: (default, PCES). From the accepted list when PCES
// installed a route; else the stored route vs the last answer for this target.
bool pcesCmpRoutes(const ContactInfo& t, uint8_t* dlen, uint8_t* dpath, uint8_t* plen, uint8_t* ppath) {
  if (const PcesAccepted* a = pcesAcceptedFor(t.id.pub_key)) {
    *dlen = a->old_len; memcpy(dpath, a->old_path, MAX_PATH_SIZE);
    *plen = a->new_len; memcpy(ppath, a->new_path, MAX_PATH_SIZE);
    return true;
  }
  if (s_pces->have_answer && s_pces->route_hops >= 0 && memcmp(s_pces->answer_for, t.id.pub_key, PUB_KEY_SIZE) == 0) {
    *dlen = t.out_path_len; memcpy(dpath, t.out_path, MAX_PATH_SIZE);
    *plen = (uint8_t)(((s_pces->plan.width - 1) << 6) | s_pces->route_hops);
    memset(ppath, 0, MAX_PATH_SIZE);
    memcpy(ppath, s_pces->route, (size_t)s_pces->route_hops * s_pces->plan.width);
    return true;
  }
  return false;
}
bool pcesFloods(uint8_t len) { return len == OUT_PATH_UNKNOWN || !mesh::Packet::isValidPathLen(len); }
uint8_t pcesBenchRounds() { return kPcesBenchRounds[s_pces->bench_rounds_i % (sizeof kPcesBenchRounds)]; }
uint32_t pcesBenchGapMs() { return 1000u * kPcesBenchGapS[s_pces->bench_gap_i % (sizeof kPcesBenchGapS)]; }

void pcesBenchCsvSample(int side, const PcesSample& sm, const char* hops_txt) {
  if (!s_pces->bench_csv[0]) return;
  char line[160];
  if (sm.ok)
    snprintf(line, sizeof line, "%u,%lu,%s,1,%u,%.2f,%s\n", (unsigned)s_pces->cmp_i, (unsigned long)(millis() - s_pces->cmp_t0),
             side ? "pces" : "default", (unsigned)sm.rtt_ms, sm.weakest4 / 4.0, hops_txt ? hops_txt : "");
  else
    snprintf(line, sizeof line, "%u,%lu,%s,0,,,\n", (unsigned)s_pces->cmp_i, (unsigned long)(millis() - s_pces->cmp_t0),
             side ? "pces" : "default");
  if (!pcesCsvWrite(s_pces->bench_csv, line, false)) s_pces->bench_csv[0] = 0;   // stop trying
}

bool pcesSameRoute(const PcesCmpSide& a, const PcesCmpSide& b) {
  if (a.len != b.len) return false;
  const int n = (a.len & 63) * ((a.len >> 6) + 1);
  return memcmp(a.path, b.path, n) == 0;
}
// With both routes fixed: start sampling, or say why not.
void pcesBenchBegin() {
  ContactInfo* t = the_mesh.lookupContactByPubKey(s_pces->cmp_for, PUB_KEY_SIZE);
  auto& d = s_pces->cmp[0]; auto& p = s_pces->cmp[1];
  s_pces->bench_discovering = false;
  if (!t) { snprintf(s_pces->bench_msg, sizeof s_pces->bench_msg, "%s", TR("Target is no longer a contact.")); s_pces->dirty = true; return; }
  if (pcesSameRoute(d, p)) {
    snprintf(s_pces->bench_msg, sizeof s_pces->bench_msg, "%s", TR("The default path is the PCES route: nothing to compare."));
    s_pces->dirty = true;
    return;
  }
  if (!pcesIsRelay(*t) && (d.len & 63) == 0) {
    snprintf(s_pces->bench_msg, sizeof s_pces->bench_msg, "%s", TR("The default path is direct (no relays): a companion does not relay a trace, so there is nothing to sample."));
    s_pces->dirty = true;
    return;
  }
  s_pces->bench_msg[0] = 0;
  s_pces->cmp_active = true; s_pces->cmp_done = false; s_pces->cmp_waiting = false;
  s_pces->cmp_i = 0;
  s_pces->cmp_t0 = millis();
  const uint32_t floor_at = s_pces->trace_last_ms ? s_pces->trace_last_ms + kPcesTraceGapMs : millis();
  s_pces->cmp_next_at = (int32_t)(floor_at - millis()) > 0 ? floor_at : millis();
  // CSV: /pces/bench-<clock>.csv, a commented header, then one row per sample.
  const uint32_t now_s = the_mesh.getRTCClock() ? the_mesh.getRTCClock()->getCurrentTime() : 0;
  snprintf(s_pces->bench_csv, sizeof s_pces->bench_csv, "/pces/bench-%lu.csv", (unsigned long)(now_s ? now_s : millis() / 1000));
  char dr[64], pr[64], head[300];
  pcesFmtRoute(d.path, d.len & 63, (d.len >> 6) + 1, dr, sizeof dr);
  pcesFmtRoute(p.path, p.len & 63, (p.len >> 6) + 1, pr, sizeof pr);
  snprintf(head, sizeof head,
           "# PCES route benchmark: target=%s, default=%s, pces=%s, rounds=%u per route, gap=%lu s, epoch=%lu\n"
           "seq,ms,route,delivered,rtt_ms,weakest_snr_db,hop_snrs_db\n",
           pcesName(*t), dr, pr, (unsigned)pcesBenchRounds(), (unsigned long)(pcesBenchGapMs() / 1000), (unsigned long)now_s);
  if (!pcesCsvWrite(s_pces->bench_csv, head, true)) s_pces->bench_csv[0] = 0;
  s_pces->dirty = true;
}
// The default side is MeshCore's own fixed path to the target, never a flood. When none
// is known, find it first with a flooded discovery (MyMesh::uiDiscoverPath): the reply
// carries the path back, and the target's stored route is left as it was.
void pcesCompareCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces) return;
  ContactInfo* t = pcesTarget();
  if (!t || s_pces->cmp_active || s_pces->bench_discovering || s_pces->run != PRUN_NONE) return;
  auto& d = s_pces->cmp[0]; auto& p = s_pces->cmp[1];
  memset(s_pces->cmp, 0, sizeof s_pces->cmp);
  s_pces->bench_csv[0] = 0;
  s_pces->cmp_done = false;
  if (!pcesCmpRoutes(*t, &d.len, d.path, &p.len, p.path)) return;
  memcpy(s_pces->cmp_for, t->id.pub_key, PUB_KEY_SIZE);
  if (!pcesFloods(d.len)) { pcesBenchBegin(); pcesRender(); return; }
  if (the_mesh.hasUIRequestPending()) {
    snprintf(s_pces->bench_msg, sizeof s_pces->bench_msg, "%s", TR("Another request is in flight. Try again shortly."));
    pcesRender(); return;
  }
  const int r = the_mesh.uiDiscoverPath(*t);
  if (r != MSG_SEND_SENT_FLOOD && r != MSG_SEND_SENT_DIRECT) {
    snprintf(s_pces->bench_msg, sizeof s_pces->bench_msg, "%s", TR("Path discovery not sent: radio busy."));
    pcesRender(); return;
  }
  s_pces->bench_discovering = true;
  s_pces->bench_disc_deadline = millis() + kPcesDiscoverWaitMs;
  snprintf(s_pces->bench_msg, sizeof s_pces->bench_msg, "%s",
           TR("No fixed default path: finding the one MeshCore would learn (one flooded discovery)..."));
  pcesRender();
}
void pcesCompareStopCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces) return;
  s_pces->cmp_active = false; s_pces->cmp_waiting = false; s_pces->trace_tag = 0;
  if (s_pces->bench_discovering) { s_pces->bench_discovering = false; the_mesh.uiCancelPathDiscovery(); s_pces->bench_msg[0] = 0; }
  else s_pces->cmp_done = true;
  pcesRender();
}
void pcesBenchRoundsCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces || s_pces->cmp_active) return;
  s_pces->bench_rounds_i = (uint8_t)((s_pces->bench_rounds_i + 1) % sizeof kPcesBenchRounds);
  pcesRender();
}
void pcesBenchGapCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces || s_pces->cmp_active) return;
  s_pces->bench_gap_i = (uint8_t)((s_pces->bench_gap_i + 1) % sizeof kPcesBenchGapS);
  pcesRender();
}
// A trace the benchmark sent got no answer in time.
void pcesBenchMissed() {
  PcesCmpSide& side = s_pces->cmp[(s_pces->cmp_i - 1) & 1];
  PcesSample sm{0, 0, 0};
  if (side.sent > 0 && side.sent <= kPcesBenchMax) side.s[side.sent - 1] = sm;
  pcesBenchCsvSample((s_pces->cmp_i - 1) & 1, sm, nullptr);
  s_pces->cmp_waiting = false;
  s_pces->dirty = true;
}
// One step of a benchmark, from the service tick.
void pcesCompareTick(uint32_t now) {
  if (s_pces->bench_discovering) {
    auto& d = s_pces->cmp[0];
    if (the_mesh.uiTakeDiscoveredPath(&d.len, d.path)) {
      pcesBenchBegin();
    } else if ((int32_t)(now - s_pces->bench_disc_deadline) >= 0) {
      the_mesh.uiCancelPathDiscovery();
      s_pces->bench_discovering = false;
      snprintf(s_pces->bench_msg, sizeof s_pces->bench_msg, "%s",
               TR("Path discovery got no reply in 30 s: no default path to compare with. Not retried."));
      s_pces->dirty = true;
    }
    return;
  }
  if (!s_pces->cmp_active || s_pces->cmp_waiting) return;
  if (s_pces->cmp_i >= 2u * pcesBenchRounds()) { s_pces->cmp_active = false; s_pces->cmp_done = true; s_pces->dirty = true; return; }
  if ((int32_t)(now - s_pces->cmp_next_at) < 0 || pcesTraceBusy()) return;
  ContactInfo* t = the_mesh.lookupContactByPubKey(s_pces->cmp_for, PUB_KEY_SIZE);
  if (!t) { s_pces->cmp_active = false; s_pces->cmp_done = true; s_pces->dirty = true; return; }
  const int which = s_pces->cmp_i & 1;
  PcesCmpSide& side = s_pces->cmp[which];
  const char* why = nullptr;
  const uint32_t tag = pcesTraceAlong(*t, side.len, side.path, &why);
  ++s_pces->cmp_i;
  s_pces->cmp_next_at = now + pcesBenchGapMs();
  if (side.sent < kPcesBenchMax) side.sent++;
  if (!tag) { pcesBenchMissed(); s_pces->cmp_waiting = false; return; }   // not sent counts as not back
  s_pces->trace_tag = tag; s_pces->trace_ms = now; s_pces->trace_last_ms = now;
  s_pces->cmp_waiting = true;
  s_pces->dirty = true;
}

bool pcesOnTraceResult(uint32_t tag, int8_t their_snr, int8_t our_snr, uint8_t extra_hops, const int8_t* extra) {
  if (!s_pces || !tag || tag != s_pces->trace_tag) return false;
  s_pces->trace_tag = 0;
  const uint32_t rtt = millis() - s_pces->trace_ms;
  if (s_pces->cmp_waiting) {   // a benchmark trace: record the sample
    const int which = (s_pces->cmp_i - 1) & 1;
    PcesCmpSide& side = s_pces->cmp[which];
    int8_t w = their_snr < our_snr ? their_snr : our_snr;
    char hops[96]; size_t o = snprintf(hops, sizeof hops, "%.2f", their_snr / 4.0);
    for (int i = 0; i < extra_hops && extra; ++i) {
      if (extra[i] < w) w = extra[i];
      if (o < sizeof hops) o += snprintf(hops + o, sizeof hops - o, " %.2f", extra[i] / 4.0);
    }
    if (o < sizeof hops) snprintf(hops + o, sizeof hops - o, " %.2f", our_snr / 4.0);
    PcesSample sm{ (uint16_t)(rtt > 65535 ? 65535 : rtt), w, 1 };
    if (side.sent > 0 && side.sent <= kPcesBenchMax) side.s[side.sent - 1] = sm;
    side.back++;
    pcesBenchCsvSample(which, sm, hops);
    s_pces->cmp_waiting = false;
    s_pces->dirty = true;
    return true;
  }
  char line[96], v[12];
  size_t o = snprintf(line, sizeof line, TR("Trace back in %u ms. SNR per hop:"), (unsigned)rtt);
  pcesFmtSnr(their_snr, v, sizeof v);
  o += snprintf(line + o, sizeof line - o, " %s", v);
  for (int i = 0; i < extra_hops && extra && o < sizeof line; ++i) {
    pcesFmtSnr(extra[i], v, sizeof v);
    o += snprintf(line + o, sizeof line - o, ", %s", v);
  }
  if (o < sizeof line) { pcesFmtSnr(our_snr, v, sizeof v); snprintf(line + o, sizeof line - o, TR(" (us: %s)"), v); }
  pcesLog(SET_TONE_OK, "%s", line);
  return true;
}

// ---- page actions ----------------------------------------------------------------------------
void pcesStartCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces) return;
  if (s_pces->run != PRUN_NONE) return;
  pcesStartTest();
  pcesRender();
}
void pcesStopCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces) return;
  if (s_pces->run == PRUN_NONE) return;
  pcesLog(SET_TONE_HELP, "%s", TR("Stopped. An answer still on its way is ignored."));
  pcesEndRun();
  pcesRender();
}
void pcesUseAnywayCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces || !s_pces->can_use_anyway) return;
  pcesInstall(false);
  pcesRender();
}
void pcesInfoCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces || s_pces->run != PRUN_NONE) return;
  ContactInfo* srv = pcesServer();
  if (!srv) return;
  pcesLogClear();
  s_pces->have_info = false;
  pcesBegin(PRUN_INFO, srv, PST_INFO_SEND);
  pcesRender();
}
void pcesRestoreCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces) return;
  const int i = (int)(intptr_t)lv_event_get_user_data(e);
  if (i < 0 || i >= kPcesAcceptedMax || !s_pces->accepted[i].used) return;
  PcesAccepted& a = s_pces->accepted[i];
  bool ok;
  if (a.old_len == OUT_PATH_UNKNOWN || !mesh::Packet::isValidPathLen(a.old_len))
    ok = the_mesh.uiResetContactPath(a.pub);
  else
    ok = the_mesh.uiSetContactPath(a.pub, a.old_path, (uint8_t)(a.old_len & 63), (uint8_t)((a.old_len >> 6) + 1));
  the_mesh.uiPinRoute(a.pub, false);
  contactsListMarkStale();
  a.used = false;
  pcesLogClear();
  pcesLog(ok ? SET_TONE_OK : SET_TONE_DANGER, "%s", ok ? TR("Previous route restored.") : TR("Could not restore: contact gone."));
  pcesRender();
}
void pcesPickServerCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces) return;
  ContactInfo c;
  if (!the_mesh.getContactByIdx((uint32_t)(uintptr_t)lv_event_get_user_data(e), c)) return;
  memcpy(s_pces->server, c.id.pub_key, PUB_KEY_SIZE);
  s_pces->have_server = true;
  s_pces->server_marked = PP::answersPces(the_mesh.advertFeat1(c.id.pub_key));
  pcesSaveServer(c.id.pub_key);
  pcesLogClear();
  pcesLog(SET_TONE_OK, "%s", TR("PCES server saved."));
  pcesRender();
}
void pcesForgetServerCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces) return;
  pcesSaveServer(nullptr);
  s_pces->have_server = false;
  pcesFindServer();
  pcesLogClear();
  pcesLog(SET_TONE_INFO, "%s", s_pces->have_server ? TR("Saved server cleared. Using the one that advertises PCES.")
                                                   : TR("Saved server cleared."));
  pcesRender();
}
void pcesPickTargetCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces) return;
  s_pces->view = PV_PICK;
  pcesRender();
}
void pcesTargetChosenCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces) return;
  ContactInfo c;
  if (the_mesh.getContactByIdx((uint32_t)(uintptr_t)lv_event_get_user_data(e), c)) {
    memcpy(s_pces->target, c.id.pub_key, PUB_KEY_SIZE);
    s_pces->have_target = true;
    pcesLogClear();
  }
  s_pces->view = PV_MAIN;
  pcesRender();
}
void pcesPickCancelCb(lv_event_t* e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED || !s_pces) return;
  s_pces->view = PV_MAIN;
  pcesRender();
}

// ---- page ----------------------------------------------------------------------------------
lv_obj_t* pcesText(lv_obj_t* parent, const char* text, const lv_font_t* f, uint32_t color) {
  lv_obj_t* l = lv_label_create(parent);
  lv_obj_set_width(l, s_settings_content_w);
  lv_label_set_long_mode(l, LV_LABEL_LONG_WRAP);
  lv_label_set_text(l, text ? text : "");
  lv_obj_set_style_text_font(l, f, LV_PART_MAIN);
  lv_obj_set_style_text_color(l, lv_color_hex(color), LV_PART_MAIN);
  return l;
}
uint32_t pcesToneColor(uint8_t tone) {
  switch (tone) {
    case SET_TONE_OK: return COLOR_STATUS_OK;
    case SET_TONE_WARN: return COLOR_STATUS_WARN;
    case SET_TONE_DANGER: return COLOR_STATUS_DANGER;
    case SET_TONE_INFO: return COLOR_STATUS_INFO;
    default: return COLOR_SUB;
  }
}

void pcesNewPage() {
  // Same view: keep the scroll position across the rebuild; a different view starts at the top.
  if (s_pces_page && s_pces->rendered_view == s_pces->view) s_pces->keep_scroll = lv_obj_get_scroll_y(s_pces_page);
  else s_pces->keep_scroll = 0;
  s_pces->rendered_view = s_pces->view;
  if (s_pces_page) {
    lv_indev_t* a = lv_indev_get_act();
    if (a) lv_indev_wait_release(a);
    lv_obj_del_async(s_pces_page);   // async: this may run from a button on the old page
    s_pces_page = nullptr;
  }
  s_pces_page = lv_obj_create(s_pces_root);
  lv_obj_remove_style_all(s_pces_page);
  lv_obj_set_size(s_pces_page, lv_disp_get_hor_res(nullptr), appPageContentH());
  const lv_coord_t bar_cover = (lv_coord_t)(STATUSBAR_H * 2) - appPageContentTop();
  lv_obj_set_style_pad_top(s_pces_page, LV_MAX(0, bar_cover) + 8, LV_PART_MAIN);
  lv_obj_set_style_pad_bottom(s_pces_page, 12, LV_PART_MAIN);
  lv_obj_set_style_pad_left(s_pces_page, 8, LV_PART_MAIN);
  lv_obj_set_flex_flow(s_pces_page, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_scroll_dir(s_pces_page, LV_DIR_VER);
}

void pcesRenderPicker() {
  lv_obj_t* card = setCardIn(s_pces_page, TR("Choose a target"));
  setButton(card, UI_ICON_X, "Cancel", pcesPickCancelCb);
  // Most recently heard first; servers are not targets (their route is asked on their advert).
  const uint32_t n = the_mesh.getNumContacts();
  static uint16_t order[64];
  int m = 0;
  for (uint32_t i = 0; i < n && m < 64; ++i) {
    ContactInfo c;
    if (!the_mesh.getContactByIdx(i, c) || c.type == ADV_TYPE_NONE) continue;
    order[m++] = (uint16_t)i;
  }
  for (int i = 0; i < m; ++i)
    for (int j = i + 1; j < m; ++j) {
      ContactInfo a, b;
      the_mesh.getContactByIdx(order[i], a); the_mesh.getContactByIdx(order[j], b);
      if (b.last_advert_timestamp > a.last_advert_timestamp) { uint16_t x = order[i]; order[i] = order[j]; order[j] = x; }
    }
  char buf[80], rt[48];
  for (int i = 0; i < m; ++i) {
    ContactInfo c;
    if (!the_mesh.getContactByIdx(order[i], c)) continue;
    pcesFmtContactRoute(c, rt, sizeof rt);
    snprintf(buf, sizeof buf, "%s  (%s)", pcesName(c), rt);
    setButton(card, c.type == ADV_TYPE_REPEATER ? UI_ICON_RADIO_TOWER : c.type == ADV_TYPE_ROOM ? UI_ICON_USERS : UI_ICON_USER,
              buf, pcesTargetChosenCb, (void*)(uintptr_t)order[i]);
  }
  if (n > 64) setNote(card, "Showing the 64 first contacts. Use PCES route on a contact for any other.");
}

void pcesRenderMain();
void pcesRender() {
  if (!s_pces || !s_pces_root) return;
  pcesNewPage();
  if (s_pces->view == PV_PICK) pcesRenderPicker();
  else pcesRenderMain();
  if (g_statusbar.root) lv_obj_move_foreground(g_statusbar.root);
  if (s_pces->keep_scroll > 0) {
    lv_obj_update_layout(s_pces_page);
    lv_obj_scroll_to_y(s_pces_page, s_pces->keep_scroll, LV_ANIM_OFF);
  }
}
void pcesRenderMain() {
  char buf[200];
  const bool running = s_pces->run != PRUN_NONE;
  ContactInfo* t = pcesTarget();

  // Target and the run.
  lv_obj_t* card = setCardIn(s_pces_page, TR("Best path test"));
  if (t) {
    setInfoRow(card, TR("Target"), pcesName(*t));
    pcesFmtContactRoute(*t, buf, sizeof buf);
    setInfoRow(card, TR("Route now"), buf);
  } else {
    setInfoRow(card, TR("Target"), TR("None"));
  }
  setButton(card, UI_ICON_USER, t ? "Change target" : "Choose target", pcesPickTargetCb);
  if (t && (pcesIsRelay(*t) || (pcesHasPath(*t) && (t->out_path_len & 63) > 0)))
    setButton(card, UI_ICON_ACTIVITY, s_pces->trace_tag ? "Tracing..." : "Trace route", pcesTraceCb);
  if (running && s_pces->run != PRUN_AUTO) setButton(card, UI_ICON_X, "Stop", pcesStopCb, nullptr, SET_DANGER);
  else setButton(card, UI_ICON_ROUTE, "Find Best Route", pcesStartCb, nullptr, (t && !running) ? SET_PRIMARY : SET_NORMAL);
  if (running && s_pces->run == PRUN_AUTO)
    pcesText(card, TR("Asking a PCES server for the path to it (its advert just arrived)..."), &g_font_12, COLOR_SUB);
  for (int i = 0; i < s_pces->n_log; ++i)
    pcesText(card, s_pces->log[i].text, &g_font_12, pcesToneColor(s_pces->log[i].tone));
  if (s_pces->can_use_anyway && !running)
    setButton(card, UI_ICON_CHECK, "Use anyway", pcesUseAnywayCb);
  if (!running && s_pces->n_log == 0)
    setNote(card, "Nothing is sent before Find Best Route. A run logs in to the server, asks INFO, then PATH from up to 3 "
                  "first hops (one request every 12 s), and installs the route only if PCES's own data backs it.");

  // The last answer, when it is for the target shown.
  if (s_pces->have_answer && t && memcmp(s_pces->answer_for, t->id.pub_key, PUB_KEY_SIZE) == 0) {
    lv_obj_t* ac = setCardIn(s_pces_page, TR("PCES answer"));
    const PP::PathAnswer& a = s_pces->ans;
    snprintf(buf, sizeof buf, "%u / 100", (unsigned)a.metric);
    pcesText(ac, buf, &g_font_semi_16, a.metric >= 50 ? COLOR_STATUS_OK : COLOR_STATUS_WARN);
    pcesFmtSnr(a.weakest_snr4, buf, sizeof buf);
    setInfoRow(ac, TR("Weakest leg"), buf);
    pcesFmtRoute(a.hop, a.hops, a.width, buf, sizeof buf);
    setInfoRow(ac, TR("PCES path"), buf);
  }

  // Benchmark: the default route vs the PCES route.
  if (t) {
    uint8_t dl, dp[MAX_PATH_SIZE], pl, pp[MAX_PATH_SIZE];
    const bool can = pcesCmpRoutes(*t, &dl, dp, &pl, pp);
    const bool mine = memcmp(s_pces->cmp_for, t->id.pub_key, PUB_KEY_SIZE) == 0 && (s_pces->cmp_active || s_pces->cmp_done);
    if (can || mine) {
      lv_obj_t* cc = setCardIn(s_pces_page, TR("Route benchmark (dev)"));
      const char* names[2] = { TR("Default"), TR("PCES") };
      for (int k = 0; k < 2; ++k) {
        char rt[64];
        const uint8_t len = mine ? s_pces->cmp[k].len : (k ? pl : dl);
        const uint8_t* path = mine ? s_pces->cmp[k].path : (k ? pp : dp);
        if (pcesFloods(len)) snprintf(rt, sizeof rt, "%s", TR("none fixed: found on Run"));
        else pcesFmtRoute(path, len & 63, (len >> 6) + 1, rt, sizeof rt);
        setInfoRow(cc, names[k], rt);
      }
      if (s_pces->bench_msg[0] && memcmp(s_pces->cmp_for, t->id.pub_key, PUB_KEY_SIZE) == 0)
        pcesText(cc, s_pces->bench_msg, &g_font_12, s_pces->bench_discovering ? COLOR_STATUS_INFO : COLOR_STATUS_WARN);
      if (s_pces->bench_discovering) setButton(cc, UI_ICON_X, "Stop", pcesCompareStopCb, nullptr, SET_DANGER);
      if (mine) {
        namespace PST = PcesStats;   // not "PS": a xtensa register macro
        static double xs[kPcesBenchMax], scratch[kPcesBenchMax];
        PST::Summary rtt[2], snr[2];
        for (int k = 0; k < 2; ++k) {
          const PcesCmpSide& c = s_pces->cmp[k];
          int n = 0;
          for (int i = 0; i < c.sent && i < kPcesBenchMax; ++i) if (c.s[i].ok) xs[n++] = c.s[i].rtt_ms;
          rtt[k] = PST::summarize(xs, n, scratch);
          n = 0;
          for (int i = 0; i < c.sent && i < kPcesBenchMax; ++i) if (c.s[i].ok) xs[n++] = c.s[i].weakest4 / 4.0;
          snr[k] = PST::summarize(xs, n, scratch);
          // A side still waiting on its last trace counts that one as pending, not lost.
          const int done = c.sent - ((s_pces->cmp_waiting && ((s_pces->cmp_i - 1) & 1) == k) ? 1 : 0);
          double lo, hi; PST::wilson(c.back, done, &lo, &hi);
          if (done <= 0) { snprintf(buf, sizeof buf, "%s: %s", names[k], TR("no samples yet")); }
          else snprintf(buf, sizeof buf, TR("%s: delivered %u/%u = %.0f%% (95%% CI %.0f-%.0f%%)"), names[k],
                        (unsigned)c.back, (unsigned)done, 100.0 * c.back / done, 100 * lo, 100 * hi);
          pcesText(cc, buf, &g_font_semi_14, COLOR_TEXT);
          if (rtt[k].n > 0) {
            snprintf(buf, sizeof buf, TR("  RTT ms: mean %.0f, sd %.0f, median %.0f, p90 %.0f"),
                     rtt[k].mean, rtt[k].sd, rtt[k].median, rtt[k].p90);
            pcesText(cc, buf, &g_font_12, COLOR_SUB);
            snprintf(buf, sizeof buf, TR("  Weakest hop dB: mean %+.2f, sd %.2f, worst %+.2f"),
                     snr[k].mean, snr[k].sd, snr[k].min);
            pcesText(cc, buf, &g_font_12, COLOR_SUB);
          }
        }
        // Differences, PCES minus default, with two-sided p.
        const PcesCmpSide& d = s_pces->cmp[0]; const PcesCmpSide& q = s_pces->cmp[1];
        const int dn = d.sent - ((s_pces->cmp_waiting && ((s_pces->cmp_i - 1) & 1) == 0) ? 1 : 0);
        const int qn = q.sent - ((s_pces->cmp_waiting && ((s_pces->cmp_i - 1) & 1) == 1) ? 1 : 0);
        if (dn > 0 && qn > 0) {
          double z, t1, t2, df1, df2;
          const double pd = PST::twoPropP(q.back, qn, d.back, dn, &z);
          const double ps = PST::welchP(snr[1], snr[0], &t1, &df1);
          const double pr = PST::welchP(rtt[1], rtt[0], &t2, &df2);
          auto pfmt = [](double p, char* o, size_t c) { if (isnan(p)) snprintf(o, c, "n/a"); else if (p < 0.001) snprintf(o, c, "<0.001"); else snprintf(o, c, "%.3f", p); };
          char a[12], b2[12], c2[12];
          pfmt(pd, a, sizeof a); pfmt(ps, b2, sizeof b2); pfmt(pr, c2, sizeof c2);
          snprintf(buf, sizeof buf, TR("PCES - default: delivery %+.0f pts (p %s), weakest hop %+.2f dB (p %s), RTT %+.0f ms (p %s)"),
                   100.0 * ((double)q.back / qn - (double)d.back / dn), a,
                   (snr[1].n && snr[0].n) ? snr[1].mean - snr[0].mean : 0.0, b2,
                   (rtt[1].n && rtt[0].n) ? rtt[1].mean - rtt[0].mean : 0.0, c2);
          pcesText(cc, buf, &g_font_12, COLOR_TEXT);
          // Plain words, only for what is significant at 5%.
          char sum[160]; size_t o = 0; sum[0] = 0;
          if (!isnan(pd) && pd < 0.05) o += snprintf(sum + o, sizeof sum - o, "%s", q.back * dn > d.back * qn ? TR("PCES delivers more. ") : TR("Default delivers more. "));
          if (!isnan(ps) && ps < 0.05 && o < sizeof sum) o += snprintf(sum + o, sizeof sum - o, "%s", snr[1].mean > snr[0].mean ? TR("PCES has the stronger weakest hop. ") : TR("Default has the stronger weakest hop. "));
          if (!isnan(pr) && pr < 0.05 && o < sizeof sum) o += snprintf(sum + o, sizeof sum - o, "%s", rtt[1].mean < rtt[0].mean ? TR("PCES is faster.") : TR("Default is faster."));
          if (!sum[0]) snprintf(sum, sizeof sum, "%s", (qn < 10 || dn < 10) ? TR("Too few samples to tell yet.") : TR("No significant difference (p < 0.05) so far."));
          pcesText(cc, sum, &g_font_semi_14, COLOR_STATUS_OK);
        }
        if (s_pces->cmp_active) {
          snprintf(buf, sizeof buf, TR("Sample %u of %u, one every %lu s..."), (unsigned)s_pces->cmp_i,
                   (unsigned)(2 * pcesBenchRounds()), (unsigned long)(pcesBenchGapMs() / 1000));
          pcesText(cc, buf, &g_font_12, COLOR_STATUS_INFO);
          setButton(cc, UI_ICON_X, "Stop benchmark", pcesCompareStopCb, nullptr, SET_DANGER);
        }
        if (s_pces->bench_csv[0]) {
          char root_path[96]; pcesCsvFullPath(s_pces->bench_csv, root_path, sizeof root_path);
          snprintf(buf, sizeof buf, TR("Samples saved to %s"), root_path);
          pcesText(cc, buf, &g_font_12, COLOR_SUB);
        }
      }
      if (!s_pces->cmp_active && !s_pces->bench_discovering) {
        snprintf(buf, sizeof buf, TR("Samples per route: %u"), (unsigned)pcesBenchRounds());
        setButton(cc, UI_ICON_SLIDERS_HORIZONTAL, buf, pcesBenchRoundsCb);
        snprintf(buf, sizeof buf, TR("Gap between traces: %u s"), (unsigned)(pcesBenchGapMs() / 1000));
        setButton(cc, UI_ICON_TIMER, buf, pcesBenchGapCb);
        snprintf(buf, sizeof buf, TR("Run benchmark (about %u min)"),
                 (unsigned)((2u * pcesBenchRounds() * pcesBenchGapMs() / 1000 + 59) / 60));
        setButton(cc, UI_ICON_ACTIVITY, buf, pcesCompareCb, nullptr, SET_PRIMARY);
        if (!mine)
          setNote(cc, "Compares the PCES route with MeshCore's own fixed path (never a flood); with none known, "
                      "Run first finds it with one flooded discovery. Then traces along each, alternating, and "
                      "statistics: delivery rate with a 95% interval, RTT and weakest-hop SNR, p-values for the "
                      "differences. Every sample goes to a CSV. Runs with the page closed. Nothing is changed.");
      }
    }
  }

  // Server.
  lv_obj_t* sc = setCardIn(s_pces_page, TR("PCES server"));
  ContactInfo* srv = pcesServer();
  if (!srv && s_pces->server_saved) {
    setNote(sc, "The saved PCES server is not in your contacts. It is used again once its advert is heard, or clear it.", SET_TONE_WARN);
    setButton(sc, UI_ICON_TRASH_2, "Clear saved server", pcesForgetServerCb, nullptr, SET_DANGER);
  }
  if (srv) {
    snprintf(buf, sizeof buf, "%s%s", pcesName(*srv), s_pces->server_marked ? "" : TR(" (no PCES marker)"));
    setInfoRow(sc, TR("Server"), buf);
    setInfoRow(sc, TR("Chosen"), s_pces->server_saved ? TR("Saved by you") : TR("Advertises PCES"));
    setButton(sc, UI_ICON_INFO, "Server info", pcesInfoCb);
    if (s_pces->server_saved) setButton(sc, UI_ICON_TRASH_2, "Clear saved server", pcesForgetServerCb, nullptr, SET_DANGER);
  } else if (!s_pces->server_saved) {
    setNote(sc, "No contact advertises PCES. Pick the Commsol station (or a room it hosts):");
  }
  // Other candidates: every repeater and room when none is set (the marker is optional),
  // only advertised servers once one is.
  int shown = 0;
  const uint32_t n = the_mesh.getNumContacts();
  for (uint32_t i = 0; i < n && shown < 12; ++i) {
    ContactInfo k;
    if (!the_mesh.getContactByIdx(i, k) || !pcesIsRelay(k)) continue;
    if (s_pces->have_server && memcmp(k.id.pub_key, s_pces->server, PUB_KEY_SIZE) == 0) continue;
    const bool marked = PP::answersPces(the_mesh.advertFeat1(k.id.pub_key));
    if (srv && !marked) continue;
    snprintf(buf, sizeof buf, "%s%s", pcesName(k), marked ? "  [PCES]" : "");
    setButton(sc, UI_ICON_ANTENNA, buf, pcesPickServerCb, (void*)(uintptr_t)i);
    ++shown;
  }

  // Routes PCES installed.
  bool any = false;
  for (const auto& a : s_pces->accepted) any |= a.used;
  if (any) {
    lv_obj_t* rc = setCardIn(s_pces_page, TR("Accepted PCES routes"));
    for (int i = 0; i < kPcesAcceptedMax; ++i) {
      const PcesAccepted& a = s_pces->accepted[i];
      if (!a.used) continue;
      ContactInfo* c = the_mesh.lookupContactByPubKey(a.pub, PUB_KEY_SIZE);
      snprintf(buf, sizeof buf, "%s: %s (%u%s)", c ? pcesName(*c) : "?", a.route_txt, (unsigned)a.metric,
               a.backed ? "" : TR(", used anyway"));
      pcesText(rc, buf, &g_font_12, COLOR_TEXT);
      setButton(rc, UI_ICON_HISTORY, "Restore", pcesRestoreCb, (void*)(intptr_t)i);
    }
    setNote(rc, "Kept until reboot.");
  }

  if (g_statusbar.root) lv_obj_move_foreground(g_statusbar.root);
}

// Closing only takes the page down: a run finishes in the background and the service
// tick keeps running.
void pcesClose() {
  if (!s_pces_root) return;
  if (s_pces) { s_pces->open = false; s_pces->view = PV_MAIN; }
  s_pces_page = nullptr;
  popupClose(&s_pces_root);
  appPageEnd(pcesBack);
}
void pcesBack() {
  if (s_pces && s_pces->view == PV_PICK) { s_pces->view = PV_MAIN; pcesRender(); return; }
  pcesClose();
}

}  // namespace

// From UITask::loop: start the service tick once the UI is up (it is what notices PCES
// server adverts). Cheap after the first call.
static void pcesServiceEnsure() {
  if (s_pces_timer || !g_lv.task) return;
  if (!s_pces) s_pces = (PcesState*)psAlloc(sizeof(PcesState));
  if (!s_pces) return;
  pcesLoadSaved();
  s_pces_timer = lv_timer_create(pcesTimerCb, 250, nullptr);
}

// Open the PCES page; with a contact, that contact is the target.
static void openPcesFor(const ContactInfo* c) {
  pcesServiceEnsure();
  if (!s_pces) { if (g_lv.task) g_lv.task->showAlert(TR("Out of memory"), 1400); return; }
  pcesClose();
  if (c && !(s_pces->have_target && memcmp(s_pces->target, c->id.pub_key, PUB_KEY_SIZE) == 0)) {
    memcpy(s_pces->target, c->id.pub_key, PUB_KEY_SIZE);
    s_pces->have_target = true;
    if (s_pces->run == PRUN_NONE) pcesLogClear();
  }
  s_pces->open = true;
  s_pces->view = PV_MAIN;
  s_pces->dirty = false;
  pcesFindServer();
  snprintf(s_pces_title, sizeof s_pces_title, "%s", "PCES");
  s_pces_root = appPageCreateRoot(COLOR_BG);
  appPageBegin(s_pces_title, pcesBack);
  pcesRender();
}
static void openPces(const ContactInfo& c) { openPcesFor(&c); }
static void openPcesTool() { openPcesFor(nullptr); }
