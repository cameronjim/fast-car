#include "racer_planner/sha256.hpp"

#include <algorithm>
#include <cstring>
#include <fstream>

namespace racer_planner {

namespace {

const std::uint32_t kK[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

inline std::uint32_t rotr(std::uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

}  // namespace

Sha256::Sha256()
    : h_{{0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab,
          0x5be0cd19}},
      buf_{},
      buf_len_(0),
      total_len_(0) {}

void Sha256::process_block(const std::uint8_t* block) {
  std::uint32_t w[64];
  for (int i = 0; i < 16; ++i) {
    w[i] = (std::uint32_t(block[4 * i]) << 24) | (std::uint32_t(block[4 * i + 1]) << 16) |
           (std::uint32_t(block[4 * i + 2]) << 8) | std::uint32_t(block[4 * i + 3]);
  }
  for (int i = 16; i < 64; ++i) {
    const std::uint32_t s0 = rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
    const std::uint32_t s1 = rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }
  std::uint32_t a = h_[0], b = h_[1], c = h_[2], d = h_[3], e = h_[4], f = h_[5], g = h_[6],
                h = h_[7];
  for (int i = 0; i < 64; ++i) {
    const std::uint32_t S1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
    const std::uint32_t ch = (e & f) ^ (~e & g);
    const std::uint32_t t1 = h + S1 + ch + kK[i] + w[i];
    const std::uint32_t S0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
    const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
    const std::uint32_t t2 = S0 + maj;
    h = g;
    g = f;
    f = e;
    e = d + t1;
    d = c;
    c = b;
    b = a;
    a = t1 + t2;
  }
  h_[0] += a;
  h_[1] += b;
  h_[2] += c;
  h_[3] += d;
  h_[4] += e;
  h_[5] += f;
  h_[6] += g;
  h_[7] += h;
}

void Sha256::update(const void* data, std::size_t len) {
  const std::uint8_t* p = static_cast<const std::uint8_t*>(data);
  total_len_ += len;
  while (len > 0) {
    const std::size_t take = std::min<std::size_t>(len, 64 - buf_len_);
    std::memcpy(buf_.data() + buf_len_, p, take);
    buf_len_ += take;
    p += take;
    len -= take;
    if (buf_len_ == 64) {
      process_block(buf_.data());
      buf_len_ = 0;
    }
  }
}

std::string Sha256::hex_digest() {
  const std::uint64_t bit_len = total_len_ * 8;
  const std::uint8_t pad_one = 0x80;
  update(&pad_one, 1);
  const std::uint8_t zero = 0;
  while (buf_len_ != 56) update(&zero, 1);
  std::uint8_t len_bytes[8];
  for (int i = 0; i < 8; ++i) len_bytes[i] = static_cast<std::uint8_t>(bit_len >> (56 - 8 * i));
  update(len_bytes, 8);
  static const char* kHex = "0123456789abcdef";
  std::string out;
  out.reserve(64);
  for (std::uint32_t word : h_) {
    for (int i = 7; i >= 0; --i) out.push_back(kHex[(word >> (4 * i)) & 0xF]);
  }
  return out;
}

std::string sha256_hex(const void* data, std::size_t len) {
  Sha256 h;
  h.update(data, len);
  return h.hex_digest();
}

bool sha256_file_hex(const std::string& path, std::string* out_hex) {
  std::ifstream in(path, std::ios::binary);
  if (!in) return false;
  Sha256 h;
  char buf[8192];
  while (in) {
    in.read(buf, sizeof(buf));
    const std::streamsize got = in.gcount();
    if (got > 0) h.update(buf, static_cast<std::size_t>(got));
  }
  if (in.bad()) return false;
  *out_hex = h.hex_digest();
  return true;
}

}  // namespace racer_planner
