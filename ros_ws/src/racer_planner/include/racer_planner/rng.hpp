// Deterministic fast Gaussian generator for the solver: xoshiro256++ for uniform bits and the
// Marsaglia and Tsang ziggurat (128 strips) for normals. Seeded, so a solve is reproducible.
#ifndef RACER_PLANNER_RNG_HPP_
#define RACER_PLANNER_RNG_HPP_

#include <cmath>
#include <cstdint>

namespace racer_planner {

class GaussianRng {
 public:
  explicit GaussianRng(std::uint64_t seed = 1) { reseed(seed); }

  void reseed(std::uint64_t seed) {
    for (auto& w : s_) {
      seed += 0x9E3779B97F4A7C15ULL;  // splitmix64
      std::uint64_t z = seed;
      z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
      z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
      w = z ^ (z >> 31);
    }
    init_tables();
  }

  std::uint64_t next_u64() {
    const std::uint64_t result = rotl(s_[0] + s_[3], 23) + s_[0];
    const std::uint64_t t = s_[1] << 17;
    s_[2] ^= s_[0];
    s_[3] ^= s_[1];
    s_[1] ^= s_[2];
    s_[0] ^= s_[3];
    s_[2] ^= t;
    s_[3] = rotl(s_[3], 45);
    return result;
  }

  // Uniform in (0, 1).
  double uniform() {
    return (static_cast<double>(next_u64() >> 11) + 0.5) * (1.0 / 9007199254740992.0);
  }

  // Standard normal.
  double normal() {
    const std::int32_t hz = static_cast<std::int32_t>(next_u64() >> 32);
    const unsigned iz = static_cast<unsigned>(hz) & 127u;
    const std::int64_t a = hz < 0 ? -static_cast<std::int64_t>(hz) : hz;
    if (static_cast<std::uint64_t>(a) < kn_[iz]) return hz * wn_[iz];
    return slow_path(hz, iz);
  }

 private:
  static std::uint64_t rotl(std::uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }

  double slow_path(std::int32_t hz, unsigned iz) {
    constexpr double r = 3.442619855899;
    for (;;) {
      double x = hz * wn_[iz];
      if (iz == 0) {
        double y;
        do {
          x = -std::log(uniform()) * 0.2904764;
          y = -std::log(uniform());
        } while (y + y < x * x);
        return hz > 0 ? r + x : -r - x;
      }
      if (fn_[iz] + uniform() * (fn_[iz - 1] - fn_[iz]) < std::exp(-0.5 * x * x)) return x;
      hz = static_cast<std::int32_t>(next_u64() >> 32);
      iz = static_cast<unsigned>(hz) & 127u;
      const std::int64_t a = hz < 0 ? -static_cast<std::int64_t>(hz) : hz;
      if (static_cast<std::uint64_t>(a) < kn_[iz]) return hz * wn_[iz];
    }
  }

  void init_tables() {
    const double m1 = 2147483648.0;
    double dn = 3.442619855899, tn = dn, vn = 9.91256303526217e-3;
    const double q = vn / std::exp(-0.5 * dn * dn);
    kn_[0] = static_cast<std::uint32_t>((dn / q) * m1);
    kn_[1] = 0;
    wn_[0] = q / m1;
    wn_[127] = dn / m1;
    fn_[0] = 1.0;
    fn_[127] = std::exp(-0.5 * dn * dn);
    for (int i = 126; i >= 1; --i) {
      dn = std::sqrt(-2.0 * std::log(vn / dn + std::exp(-0.5 * dn * dn)));
      kn_[i + 1] = static_cast<std::uint32_t>((dn / tn) * m1);
      tn = dn;
      fn_[i] = std::exp(-0.5 * dn * dn);
      wn_[i] = dn / m1;
    }
  }

  std::uint64_t s_[4];
  std::uint32_t kn_[128];
  double wn_[128];
  double fn_[128];
};

}  // namespace racer_planner

#endif  // RACER_PLANNER_RNG_HPP_
