// SHA-256 with no external library. Used to check bundle payloads against the manifest.
#ifndef RACER_PLANNER_SHA256_HPP_
#define RACER_PLANNER_SHA256_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace racer_planner {

class Sha256 {
 public:
  Sha256();
  void update(const void* data, std::size_t len);
  // Finishes the hash and returns the lower case hex digest. The object is spent afterwards.
  std::string hex_digest();

 private:
  void process_block(const std::uint8_t* block);
  std::array<std::uint32_t, 8> h_;
  std::array<std::uint8_t, 64> buf_;
  std::size_t buf_len_;
  std::uint64_t total_len_;
};

std::string sha256_hex(const void* data, std::size_t len);
// false when the file cannot be read.
bool sha256_file_hex(const std::string& path, std::string* out_hex);

}  // namespace racer_planner

#endif  // RACER_PLANNER_SHA256_HPP_
