// SPDX-License-Identifier: GPL-3.0-or-later
//
// Host test for the PCES benchmark statistics (src/helpers/PcesStats.h). Expected
// values are from R (t.test, prop.test without continuity correction, quantile type 7)
// and standard tables. Run from the repo root:
//
//   c++ -std=c++17 -Wall -Wextra -I src test/test_pces_stats.cpp -o /tmp/test_pces_stats
//   /tmp/test_pces_stats
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "helpers/PcesStats.h"

namespace S = PcesStats;
static bool near(double a, double b, double tol) { return fabs(a - b) <= tol; }

int main() {
  double scratch[16];
  // Summary: mean, SD (n-1), median, p90 (type 7).
  {
    const double x[] = { 2, 4, 4, 4, 5, 5, 7, 9 };
    S::Summary s = S::summarize(x, 8, scratch);
    assert(s.n == 8 && near(s.mean, 5.0, 1e-12));
    assert(near(s.sd, 2.138089935, 1e-8));          // sd(c(2,4,4,4,5,5,7,9))
    assert(near(s.median, 4.5, 1e-12));
    assert(near(s.p90, 7.6, 1e-12));                // quantile(x, .9)
    assert(s.min == 2 && s.max == 9);
    assert(x[0] == 2 && x[7] == 9);                 // input untouched
  }
  // Wilson 95%: 8 of 10 -> [0.4902, 0.9433].
  {
    double lo, hi;
    S::wilson(8, 10, &lo, &hi);
    assert(near(lo, 0.4902, 1e-3) && near(hi, 0.9433, 1e-3));
    S::wilson(0, 10, &lo, &hi);
    assert(lo == 0 && near(hi, 0.2775, 1e-3));
  }
  // Two-proportion z: 45/50 vs 35/50 -> z = 2.5, p = 0.01242.
  {
    double z;
    const double p = S::twoPropP(45, 50, 35, 50, &z);
    assert(near(z, 2.5, 1e-9) && near(p, 0.0124193, 1e-6));
    assert(isnan(S::twoPropP(10, 10, 10, 10, nullptr)));   // both all-delivered: undefined
  }
  // Student t, two-sided: t = 2.228, df = 10 -> p ~ 0.05; t = 2, df = 10 -> 0.07339.
  assert(near(S::studentP2(2.228138852, 10), 0.05, 1e-6));
  assert(near(S::studentP2(2.0, 10), 0.0733880, 1e-6));
  assert(near(S::studentP2(0.0, 5), 1.0, 1e-12));
  // Welch: t.test(c(1,2,3,4,5), c(2,4,6,8,10)) -> t = -1.8974, df = 5.882, p = 0.1075.
  {
    const double a[] = { 1, 2, 3, 4, 5 }, b[] = { 2, 4, 6, 8, 10 };
    S::Summary sa = S::summarize(a, 5, scratch), sb = S::summarize(b, 5, scratch);
    double t, df;
    const double p = S::welchP(sa, sb, &t, &df);
    assert(near(t, -1.897367, 1e-5) && near(df, 5.882353, 1e-5) && near(p, 0.1075, 5e-4));
  }
  // Too few samples: undefined, not a crash.
  {
    const double a[] = { 1 };
    S::Summary s1 = S::summarize(a, 1, scratch);
    assert(isnan(S::welchP(s1, s1, nullptr, nullptr)));
  }
  printf("test_pces_stats: all passed\n");
  return 0;
}
