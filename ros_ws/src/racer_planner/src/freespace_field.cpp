#include "racer_planner/freespace_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace racer_planner {

namespace {
constexpr float kInf = std::numeric_limits<float>::infinity();
constexpr double kTwoPi = 6.283185307179586;
constexpr double kPi = 3.141592653589793;

int bearing_bin(double dx, double dy) {
  int b = static_cast<int>(std::floor((std::atan2(dy, dx) + kPi) / kTwoPi * FreespaceField::kBins));
  b %= FreespaceField::kBins;
  if (b < 0) b += FreespaceField::kBins;
  return b;
}

struct Step {
  int di, dj;
  float len;
};
constexpr float kDiag = 1.41421356f;
constexpr Step kSteps[8] = {{1, 0, 1.0f},  {-1, 0, 1.0f},  {0, 1, 1.0f},   {0, -1, 1.0f},
                            {1, 1, kDiag}, {1, -1, kDiag}, {-1, 1, kDiag}, {-1, -1, kDiag}};
}  // namespace

FreespaceField::FreespaceField(const VehicleGeometry& g, const FreespaceParams& params)
    : p_(params),
      nx_(static_cast<int>(std::lround((DistanceField::kXMaxM - DistanceField::kXMinM) /
                                       DistanceField::kResolutionM)) +
          1),
      ny_(static_cast<int>(std::lround((DistanceField::kYMaxM - DistanceField::kYMinM) /
                                       DistanceField::kResolutionM)) +
          1),
      cells_(static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_)),
      head_x_(g.lidar_mount_x_m),
      head_y_(g.lidar_mount_y_m),
      rear_x_(g.body_min_x_m()),
      front_axle_x_(g.wheelbase_m) {
  cell_bin_.assign(cells_, 0);
  cell_range_.assign(cells_, 0.0f);
  for (int j = 0; j < ny_; ++j) {
    for (int i = 0; i < nx_; ++i) {
      const double x = DistanceField::kXMinM + i * DistanceField::kResolutionM;
      const double y = DistanceField::kYMinM + j * DistanceField::kResolutionM;
      const double dx = x - head_x_, dy = y - head_y_;
      cell_bin_[idx(i, j)] = static_cast<std::uint16_t>(bearing_bin(dx, dy));
      cell_range_[idx(i, j)] = static_cast<float>(std::hypot(dx, dy));
    }
  }
  vis_.assign(cells_, 0);
  trav_.assign(cells_, 0);
  reached_.assign(cells_, 0);
  cost_.assign(cells_, 1.0f);
  g_.assign(cells_, kInf);
  phi_.assign(cells_, 0.0f);
  key_.assign(cells_, kInf);
  carry_.assign(cells_, 0.0f);
  heap_.assign(cells_, 0);
  pos_.assign(cells_, -1);
  std::fill(vis_range_, vis_range_ + kBins, 0.0f);
}

void FreespaceField::build_visibility(const PointXY* points, std::size_t count) {
  float img[kBins];
  std::fill(img, img + kBins, kInf);
  for (std::size_t n = 0; n < count; ++n) {
    if (!std::isfinite(points[n].x) || !std::isfinite(points[n].y)) continue;
    const double dx = points[n].x - head_x_, dy = points[n].y - head_y_;
    const int b = bearing_bin(dx, dy);
    const float r = static_cast<float>(std::hypot(dx, dy));
    if (r < img[b]) img[b] = r;
  }
  // Empty runs: up to max_gap_bins takes the smaller neighbouring range, longer is unseen (0).
  int start = -1;
  for (int b = 0; b < kBins; ++b) {
    if (std::isfinite(img[b])) {
      start = b;
      break;
    }
  }
  if (start < 0) {
    std::fill(vis_range_, vis_range_ + kBins, 0.0f);
    return;
  }
  for (int b = 0; b < kBins; ++b) vis_range_[b] = std::isfinite(img[b]) ? img[b] : 0.0f;
  int k = 0;
  while (k < kBins) {
    const int b = (start + k) % kBins;
    if (std::isfinite(img[b])) {
      ++k;
      continue;
    }
    int len = 0;
    while (k + len < kBins && !std::isfinite(img[(start + k + len) % kBins])) ++len;
    const float left = img[(start + k - 1 + kBins) % kBins];
    const float right = img[(start + k + len) % kBins];
    const float fill = len <= p_.max_gap_bins ? std::min(left, right) : 0.0f;
    for (int m = 0; m < len; ++m) vis_range_[(start + k + m) % kBins] = fill;
    k += len;
  }
}

// -- indexed binary heap -------------------------------------------------------------------------

void FreespaceField::heap_clear() {
  for (int i = 0; i < heap_size_; ++i) pos_[static_cast<std::size_t>(heap_[i])] = -1;
  heap_size_ = 0;
}

void FreespaceField::sift_up(int i, const float* key) {
  const int cell = heap_[i];
  const float k = key[cell];
  while (i > 0) {
    const int parent = (i - 1) / 2;
    if (key[heap_[parent]] <= k) break;
    heap_[i] = heap_[parent];
    pos_[static_cast<std::size_t>(heap_[i])] = i;
    i = parent;
  }
  heap_[i] = cell;
  pos_[static_cast<std::size_t>(cell)] = i;
}

void FreespaceField::sift_down(int i, const float* key) {
  const int cell = heap_[i];
  const float k = key[cell];
  for (;;) {
    int child = 2 * i + 1;
    if (child >= heap_size_) break;
    if (child + 1 < heap_size_ && key[heap_[child + 1]] < key[heap_[child]]) ++child;
    if (key[heap_[child]] >= k) break;
    heap_[i] = heap_[child];
    pos_[static_cast<std::size_t>(heap_[i])] = i;
    i = child;
  }
  heap_[i] = cell;
  pos_[static_cast<std::size_t>(cell)] = i;
}

void FreespaceField::heap_push(std::size_t cell, const float* key) {
  int& p = pos_[cell];
  if (p < 0) {
    heap_[static_cast<std::size_t>(heap_size_)] = static_cast<int>(cell);
    p = heap_size_;
    ++heap_size_;
    sift_up(p, key);
  } else {
    sift_up(p, key);  // the key only decreases
  }
}

std::size_t FreespaceField::heap_pop(const float* key) {
  const int top = heap_[0];
  pos_[static_cast<std::size_t>(top)] = -1;
  --heap_size_;
  if (heap_size_ > 0) {
    heap_[0] = heap_[heap_size_];
    pos_[static_cast<std::size_t>(heap_[0])] = 0;
    sift_down(0, key);
  }
  return static_cast<std::size_t>(top);
}

// -- searches ------------------------------------------------------------------------------------

void FreespaceField::dijkstra_from(std::size_t seed, float* out) {
  std::fill(out, out + cells_, kInf);
  heap_clear();
  out[seed] = 0.0f;
  heap_push(seed, out);
  const float res = static_cast<float>(DistanceField::kResolutionM);
  while (!heap_empty()) {
    const std::size_t c = heap_pop(out);
    const int ci = static_cast<int>(c % static_cast<std::size_t>(nx_));
    const int cj = static_cast<int>(c / static_cast<std::size_t>(nx_));
    const float d = out[c];
    for (const Step& s : kSteps) {
      const int ni = ci + s.di, nj = cj + s.dj;
      if (ni < 0 || nj < 0 || ni >= nx_ || nj >= ny_) continue;
      const std::size_t n = idx(ni, nj);
      if (!trav_[n]) continue;
      const float nd = d + s.len * res * 0.5f * (cost_[c] + cost_[n]);
      if (nd < out[n]) {
        out[n] = nd;
        heap_push(n, out);
      }
    }
  }
}

void FreespaceField::fill_outside() {
  std::fill(key_.begin(), key_.end(), kInf);
  heap_clear();
  for (std::size_t c = 0; c < cells_; ++c) {
    if (reached_[c]) {
      key_[c] = 0.0f;
      carry_[c] = phi_[c];
      heap_push(c, key_.data());
    }
  }
  const float res = static_cast<float>(DistanceField::kResolutionM);
  while (!heap_empty()) {
    const std::size_t c = heap_pop(key_.data());
    const int ci = static_cast<int>(c % static_cast<std::size_t>(nx_));
    const int cj = static_cast<int>(c / static_cast<std::size_t>(nx_));
    for (const Step& s : kSteps) {
      const int ni = ci + s.di, nj = cj + s.dj;
      if (ni < 0 || nj < 0 || ni >= nx_ || nj >= ny_) continue;
      const std::size_t n = idx(ni, nj);
      if (reached_[n] || !vis_[n]) continue;
      const float nk = key_[c] + s.len * res;
      if (nk < key_[n]) {
        key_[n] = nk;
        carry_[n] = carry_[c];
        heap_push(n, key_.data());
      }
    }
  }
  const float pen = static_cast<float>(p_.outside_penalty_per_m);
  float max_phi = 0.0f;
  for (std::size_t c = 0; c < cells_; ++c) {
    if (!reached_[c]) {
      if (!std::isfinite(key_[c])) continue;
      phi_[c] = carry_[c] + pen * key_[c];
    }
    max_phi = std::max(max_phi, phi_[c]);
  }
  // Every cell still without a value (not visible, or beyond the grid via the lookup) is above
  // every value that is finite.
  far_ = static_cast<double>(max_phi) + 1.0;
  const float far = static_cast<float>(far_);
  for (std::size_t c = 0; c < cells_; ++c) {
    if (!reached_[c] && !std::isfinite(key_[c])) phi_[c] = far;
  }
}

bool FreespaceField::build(const DistanceField& field, const PointXY* points, std::size_t count,
                           const PointXY* prev_goal) {
  valid_ = false;
  build_visibility(points, count);
  const double res = DistanceField::kResolutionM;
  for (int j = 0; j < ny_; ++j) {
    for (int i = 0; i < nx_; ++i) {
      const std::size_t c = idx(i, j);
      const bool vis = cell_range_[c] < vis_range_[cell_bin_[c]];
      vis_[c] = vis ? 1 : 0;
      const double d = field.node(i, j);
      const double x = DistanceField::kXMinM + i * res;
      const bool trav = vis && d >= p_.nav_radius_m && x >= rear_x_;
      trav_[c] = trav ? 1 : 0;
      if (trav) {
        const double e = std::min(std::max((p_.nav_pref_m - d) / p_.nav_pref_m, 0.0), 1.0);
        cost_[c] = static_cast<float>(1.0 + p_.nav_centre_weight * e * e);
      }
    }
  }
  // Seed: the traversable cell nearest the front axle point.
  std::size_t seed = cells_;
  double best_d2 = p_.seed_max_m * p_.seed_max_m;
  for (int j = 0; j < ny_; ++j) {
    for (int i = 0; i < nx_; ++i) {
      if (!trav_[idx(i, j)]) continue;
      const double dx = DistanceField::kXMinM + i * res - front_axle_x_;
      const double dy = DistanceField::kYMinM + j * res;
      const double d2 = dx * dx + dy * dy;
      if (d2 <= best_d2) {
        best_d2 = d2;
        seed = idx(i, j);
      }
    }
  }
  if (seed == cells_) return false;
  dijkstra_from(seed, g_.data());
  std::size_t best = cells_;
  float best_g = -1.0f;
  for (std::size_t c = 0; c < cells_; ++c) {
    const bool r = std::isfinite(g_[c]);
    reached_[c] = r ? 1 : 0;
    if (r && g_[c] > best_g) {
      best_g = g_[c];
      best = c;
    }
  }
  if (best == cells_) return false;
  std::size_t goal = best;
  if (prev_goal != nullptr && std::isfinite(prev_goal->x) && std::isfinite(prev_goal->y)) {
    std::size_t cand = cells_;
    float cand_g = -1.0f;
    const double r2 = p_.hysteresis_radius_m * p_.hysteresis_radius_m;
    for (int j = 0; j < ny_; ++j) {
      for (int i = 0; i < nx_; ++i) {
        const std::size_t c = idx(i, j);
        if (!reached_[c]) continue;
        const double dx = DistanceField::kXMinM + i * res - prev_goal->x;
        const double dy = DistanceField::kYMinM + j * res - prev_goal->y;
        if (dx * dx + dy * dy <= r2 && g_[c] > cand_g) {
          cand_g = g_[c];
          cand = c;
        }
      }
    }
    if (cand != cells_ && cand_g >= p_.hysteresis_keep * best_g) goal = cand;
  }
  if (g_[goal] < p_.min_goal_cost_m) return false;
  reach_m_ = g_[goal];
  goal_ = PointXY{
      DistanceField::kXMinM + static_cast<double>(goal % static_cast<std::size_t>(nx_)) * res,
      DistanceField::kYMinM + static_cast<double>(goal / static_cast<std::size_t>(nx_)) * res};
  dijkstra_from(goal, phi_.data());
  fill_outside();
  valid_ = true;
  return true;
}

double FreespaceField::phi_lookup(double x, double y) const {
  const double inv = 1.0 / DistanceField::kResolutionM;
  const double fx = (x - DistanceField::kXMinM) * inv;
  const double fy = (y - DistanceField::kYMinM) * inv;
  if (!(fx >= 0.0 && fy >= 0.0 && fx <= static_cast<double>(nx_ - 1) &&
        fy <= static_cast<double>(ny_ - 1))) {
    return far_;
  }
  const int i = std::min(static_cast<int>(fx), nx_ - 2);
  const int j = std::min(static_cast<int>(fy), ny_ - 2);
  const double ax = fx - i, ay = fy - j;
  const float* r0 = &phi_[idx(i, j)];
  const float* r1 = r0 + nx_;
  const double d0 = r0[0] + ax * (r0[1] - r0[0]);
  const double d1 = r1[0] + ax * (r1[1] - r1[0]);
  return d0 + ay * (d1 - d0);
}

}  // namespace racer_planner
