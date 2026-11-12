#include <gtest/gtest.h>

#include <string>

#include "racer_planner/sha256.hpp"
#include "test_support.hpp"

using namespace racer_planner;

TEST(Sha256, EmptyString) {
  EXPECT_EQ(sha256_hex("", 0), "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
}

TEST(Sha256, Abc) {
  EXPECT_EQ(sha256_hex("abc", 3),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST(Sha256, TwoBlockMessage) {
  const std::string m = "abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq";
  EXPECT_EQ(sha256_hex(m.data(), m.size()),
            "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
}

TEST(Sha256, OneMillionAs) {
  const std::string m(1000000, 'a');
  EXPECT_EQ(sha256_hex(m.data(), m.size()),
            "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
}

TEST(Sha256, OneMegabytePatternInOddChunksMatchesOneShot) {
  std::string m(1 << 20, '\0');
  for (std::size_t i = 0; i < m.size(); ++i) m[i] = static_cast<char>((i * 31 + 7) & 0xFF);
  Sha256 h;
  std::size_t off = 0, chunk = 1;
  while (off < m.size()) {
    const std::size_t take = std::min(chunk, m.size() - off);
    h.update(m.data() + off, take);
    off += take;
    chunk = chunk * 3 + 1;
  }
  EXPECT_EQ(h.hex_digest(), sha256_hex(m.data(), m.size()));
}

TEST(Sha256, FileHashMatchesAndMissingFileFails) {
  const std::string path = test::fs::temp_directory_path().string() + "/racer_planner_sha_test.bin";
  test::write_text(path, "abc");
  std::string hex;
  ASSERT_TRUE(sha256_file_hex(path, &hex));
  EXPECT_EQ(hex, "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  test::fs::remove(path);
  EXPECT_FALSE(sha256_file_hex(path, &hex));
}
