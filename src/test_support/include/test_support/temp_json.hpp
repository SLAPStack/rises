// RAII temp file for JSON content used in geofence map-loader tests. Creates a
// uniquely-named file under TMPDIR (or /tmp) on construction, writes the
// given content, and unlinks on destruction. Race-free creation via mkstemps;
// no path is reused, so parallel tests do not collide.

#pragma once

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>

#include <fcntl.h>
#include <unistd.h>

namespace test_support {

class TempJsonFile {
public:
  explicit TempJsonFile(const std::string &content) {
    const char *tmpdir = std::getenv("TMPDIR");
    std::string templ;
    templ.reserve(64);
    templ.append(tmpdir != nullptr ? tmpdir : "/tmp");
    templ.append("/aris_test_XXXXXX.json");

    // mkstemps creates the file with mode 0600 and an O_EXCL race-free open.
    // The ".json" suffix length (5) is passed so the random portion lands in
    // the X-marker region.
    std::string mutable_templ = templ;
    constexpr int kSuffixLen = 5;
    const int fd = ::mkstemps(mutable_templ.data(), kSuffixLen);
    if (fd < 0) {
      const int err = errno;
      throw std::system_error(err, std::generic_category(),
                              "TempJsonFile: mkstemps failed");
    }
    ::close(fd);
    path_ = std::move(mutable_templ);

    std::ofstream out(path_, std::ios::binary | std::ios::trunc);
    if (!out) {
      ::unlink(path_.c_str());
      throw std::runtime_error("TempJsonFile: open for write failed: " + path_);
    }
    out << content;
    if (!out) {
      ::unlink(path_.c_str());
      throw std::runtime_error("TempJsonFile: write failed: " + path_);
    }
  }

  ~TempJsonFile() noexcept {
    if (!path_.empty()) {
      ::unlink(path_.c_str());
    }
  }

  TempJsonFile(const TempJsonFile &) = delete;
  TempJsonFile &operator=(const TempJsonFile &) = delete;
  TempJsonFile(TempJsonFile &&) = delete;
  TempJsonFile &operator=(TempJsonFile &&) = delete;

  const std::string &path() const noexcept { return path_; }

private:
  std::string path_;
};

} // namespace test_support
