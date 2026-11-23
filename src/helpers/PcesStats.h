// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// Small statistics kit for the PCES route benchmark (src/ui-touch/PcesApp.h): the
// default route vs the PCES route over many traces. Pure functions, no Arduino, so
// the host test (test/test_pces_stats.cpp) checks them against known values.
//
//   * delivery rate: Wilson 95% interval; two routes compared with a two-proportion
//     z-test (pooled), two-sided p;
//   * RTT and weakest-hop SNR: mean / SD / median / p90; two routes compared with
//     Welch's t-test, two-sided p from the Student t distribution (regularised
//     incomplete beta), Welch-Satterthwaite degrees of freedom.
#include <math.h>
#include <stddef.h>

namespace PcesStats {

struct Summary { int n; double mean, sd, min, max, median, p90; };

inline void sortAsc(double* x, int n) {   // n <= a few hundred: insertion sort
  for (int i = 1; i < n; ++i) {
    const double v = x[i];
    int j = i - 1;
    while (j >= 0 && x[j] > v) { x[j + 1] = x[j]; --j; }
    x[j + 1] = v;
  }
}
// Linear interpolation between closest ranks (type 7, as R and numpy default).
inline double quantileSorted(const double* x, int n, double q) {
  if (n <= 0) return NAN;
  if (n == 1) return x[0];
  const double h = (n - 1) * q;
  const int lo = (int)floor(h);
  const int hi = lo + 1 < n ? lo + 1 : n - 1;
  return x[lo] + (h - lo) * (x[hi] - x[lo]);
}
// `scratch` holds n doubles; x is left unchanged.
inline Summary summarize(const double* x, int n, double* scratch) {
  Summary s{n, NAN, NAN, NAN, NAN, NAN, NAN};
  if (n <= 0) return s;
  double sum = 0;
  s.min = s.max = x[0];
  for (int i = 0; i < n; ++i) {
    sum += x[i];
    if (x[i] < s.min) s.min = x[i];
    if (x[i] > s.max) s.max = x[i];
    scratch[i] = x[i];
  }
  s.mean = sum / n;
  if (n > 1) {
    double ss = 0;
    for (int i = 0; i < n; ++i) ss += (x[i] - s.mean) * (x[i] - s.mean);
    s.sd = sqrt(ss / (n - 1));
  } else {
    s.sd = 0;
  }
  sortAsc(scratch, n);
  s.median = quantileSorted(scratch, n, 0.5);
  s.p90 = quantileSorted(scratch, n, 0.9);
  return s;
}

// Wilson score interval for k successes out of n, 95%.
inline void wilson(int k, int n, double* lo, double* hi) {
  if (n <= 0) { *lo = 0; *hi = 1; return; }
  const double z = 1.959963984540054, p = (double)k / n, z2 = z * z;
  const double den = 1 + z2 / n;
  const double ctr = (p + z2 / (2.0 * n)) / den;
  const double half = z * sqrt(p * (1 - p) / n + z2 / (4.0 * n * n)) / den;
  *lo = ctr - half < 0 ? 0 : ctr - half;
  *hi = ctr + half > 1 ? 1 : ctr + half;
}

// Two-sided p for a standard normal z.
inline double normalP2(double z) { return erfc(fabs(z) / sqrt(2.0)); }

// Two-proportion z-test (pooled). Returns the two-sided p; NAN when undefined
// (an empty group, or both groups all-or-nothing the same way).
inline double twoPropP(int k1, int n1, int k2, int n2, double* z_out) {
  if (z_out) *z_out = NAN;
  if (n1 <= 0 || n2 <= 0) return NAN;
  const double p1 = (double)k1 / n1, p2 = (double)k2 / n2, pp = (double)(k1 + k2) / (n1 + n2);
  const double se = sqrt(pp * (1 - pp) * (1.0 / n1 + 1.0 / n2));
  if (se <= 0) return NAN;
  const double z = (p1 - p2) / se;
  if (z_out) *z_out = z;
  return normalP2(z);
}

// Continued fraction for the incomplete beta (Numerical Recipes betacf).
inline double betacf(double a, double b, double x) {
  // Names chosen not to clash with toolchain macros (xtensa specreg.h defines EPS_*).
  const int kMaxIt = 200; const double kEpsilon = 3e-14, kTiny = 1e-300;
  double qab = a + b, qap = a + 1, qam = a - 1, c = 1, d = 1 - qab * x / qap;
  if (fabs(d) < kTiny) d = kTiny;
  d = 1 / d;
  double h = d;
  for (int m = 1; m <= kMaxIt; ++m) {
    const int m2 = 2 * m;
    double aa = m * (b - m) * x / ((qam + m2) * (a + m2));
    d = 1 + aa * d; if (fabs(d) < kTiny) d = kTiny;
    c = 1 + aa / c; if (fabs(c) < kTiny) c = kTiny;
    d = 1 / d; h *= d * c;
    aa = -(a + m) * (qab + m) * x / ((a + m2) * (qap + m2));
    d = 1 + aa * d; if (fabs(d) < kTiny) d = kTiny;
    c = 1 + aa / c; if (fabs(c) < kTiny) c = kTiny;
    d = 1 / d;
    const double del = d * c;
    h *= del;
    if (fabs(del - 1) < kEpsilon) break;
  }
  return h;
}
// Regularised incomplete beta I_x(a, b).
inline double ibeta(double a, double b, double x) {
  if (x <= 0) return 0;
  if (x >= 1) return 1;
  const double bt = exp(lgamma(a + b) - lgamma(a) - lgamma(b) + a * log(x) + b * log(1 - x));
  if (x < (a + 1) / (a + b + 2)) return bt * betacf(a, b, x) / a;
  return 1 - bt * betacf(b, a, 1 - x) / b;
}
// Two-sided p for Student's t with df degrees of freedom.
inline double studentP2(double t, double df) {
  if (!(df > 0) || isnan(t)) return NAN;
  return ibeta(df / 2, 0.5, df / (df + t * t));
}

// Welch's t-test from two summaries. Returns the two-sided p (NAN when undefined).
inline double welchP(const Summary& a, const Summary& b, double* t_out, double* df_out) {
  if (t_out) *t_out = NAN;
  if (df_out) *df_out = NAN;
  if (a.n < 2 || b.n < 2) return NAN;
  const double va = a.sd * a.sd / a.n, vb = b.sd * b.sd / b.n;
  const double se = sqrt(va + vb);
  if (se <= 0) return NAN;
  const double t = (a.mean - b.mean) / se;
  const double df = (va + vb) * (va + vb) / (va * va / (a.n - 1) + vb * vb / (b.n - 1));
  if (t_out) *t_out = t;
  if (df_out) *df_out = df;
  return studentP2(t, df);
}

}  // namespace PcesStats
