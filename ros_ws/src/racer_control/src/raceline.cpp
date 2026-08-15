#include "racer_control/raceline.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace racer_control {

namespace {

std::vector<std::string> split_csv_line(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;
  while (std::getline(ss, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

double parse_double(const std::string& field, const std::string& path, int line_number) {
  try {
    std::size_t consumed = 0;
    double value = std::stod(field, &consumed);
    if (consumed != field.size()) {
      throw std::invalid_argument("trailing characters");
    }
    return value;
  } catch (const std::exception&) {
    std::ostringstream msg;
    msg << path << ":" << line_number << ": could not parse '" << field << "' as a number";
    throw RacelineLoadError(msg.str());
  }
}

}  // namespace

Raceline::Raceline(std::vector<RacelinePoint> points) : points_(std::move(points)) {
  if (points_.empty()) {
    throw RacelineLoadError("Raceline: refusing to construct from zero points");
  }
}

Raceline Raceline::load_from_csv(const std::string& path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw RacelineLoadError("Raceline::load_from_csv: could not open '" + path + "'");
  }

  static const std::vector<std::string> kExpectedHeader = {
      "s_m", "x_m", "y_m", "heading_rad", "curvature_1pm", "target_speed_mps"};

  std::string line;
  int line_number = 0;
  bool found_header = false;
  std::vector<RacelinePoint> points;

  while (std::getline(file, line)) {
    ++line_number;
    // std::getline splits on '\n' only; a CRLF-terminated file (or any file that picks up
    // "\r\n" line endings from a Windows editor/tool) would otherwise leave a trailing '\r'
    // stuck to the last field of every line, corrupting the header match and every row's
    // last (target_speed_mps) column. Defense in depth: tools/raceline/io.py writes plain
    // "\n" line endings, but this reader should not silently misparse a file that doesn't.
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.empty()) {
      continue;
    }
    if (line.front() == '#') {
      continue;  // provenance header line, see tools/raceline/io.py
    }
    std::vector<std::string> fields = split_csv_line(line);
    if (!found_header) {
      if (fields != kExpectedHeader) {
        throw RacelineLoadError(path +
                                ": expected CSV header 's_m,x_m,y_m,heading_rad,"
                                "curvature_1pm,target_speed_mps', got '" +
                                line + "'");
      }
      found_header = true;
      continue;
    }
    if (fields.size() != 6) {
      std::ostringstream msg;
      msg << path << ":" << line_number << ": expected 6 columns, got " << fields.size();
      throw RacelineLoadError(msg.str());
    }
    RacelinePoint point;
    point.s_m = parse_double(fields[0], path, line_number);
    point.x_m = parse_double(fields[1], path, line_number);
    point.y_m = parse_double(fields[2], path, line_number);
    point.heading_rad = parse_double(fields[3], path, line_number);
    point.curvature_1pm = parse_double(fields[4], path, line_number);
    point.target_speed_mps = parse_double(fields[5], path, line_number);
    points.push_back(point);
  }

  if (!found_header) {
    throw RacelineLoadError(path + ": no CSV header row found");
  }
  if (points.empty()) {
    throw RacelineLoadError(path + ": no raceline data rows found");
  }

  return Raceline(std::move(points));
}

std::size_t Raceline::nearest_index(double x, double y) const {
  std::size_t best_index = 0;
  double best_dist2 = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < points_.size(); ++i) {
    const double dx = points_[i].x_m - x;
    const double dy = points_[i].y_m - y;
    const double dist2 = dx * dx + dy * dy;
    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      best_index = i;
    }
  }
  return best_index;
}

std::size_t Raceline::advance_to_lookahead(std::size_t from_index, double x, double y,
                                           double lookahead_m) const {
  const std::size_t n = points_.size();
  std::size_t idx = from_index % n;
  std::size_t best_index = idx;
  double best_dist = -1.0;
  for (std::size_t steps = 0; steps < n; ++steps) {
    const double dx = points_[idx].x_m - x;
    const double dy = points_[idx].y_m - y;
    const double dist = std::hypot(dx, dy);
    if (dist > best_dist) {
      best_dist = dist;
      best_index = idx;
    }
    if (dist >= lookahead_m) {
      return idx;
    }
    idx = (idx + 1) % n;
  }
  // Walked the whole closed loop without reaching lookahead_m (a very short raceline, or a
  // lookahead_m larger than the track): return the farthest point found, rather than an
  // arbitrary one.
  return best_index;
}

}  // namespace racer_control
