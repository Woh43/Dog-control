#include "custom_dog_control/nmpc/ModelCache.hpp"

#include <boost/uuid/detail/sha1.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>

namespace custom_dog_control {
namespace {

// Bump this schema when the taped-function implementation or upstream AD ABI
// changes. The current upstream algorithm sources are independently pinned and
// checked by the repository's provenance test.
constexpr char kCacheSchema[] = "custom-dog-cppad-cache-v1";

std::string NativeCodeTag() {
  std::ostringstream tag;
#if defined(__aarch64__)
  tag << "aarch64";
#elif defined(__x86_64__)
  tag << "x86_64";
#elif defined(__arm__)
  tag << "arm";
#else
  tag << "other-architecture";
#endif
  tag << ":pointer-bits=" << sizeof(void*) * 8;
  tag << ":cxx=" << __cplusplus;
#if defined(__clang__)
  tag << ":clang=" << __clang_version__;
#elif defined(__GNUC__)
  tag << ":gcc=" << __VERSION__;
#endif
#if defined(_GLIBCXX_USE_CXX11_ABI)
  tag << ":glibcxx-abi=" << _GLIBCXX_USE_CXX11_ABI;
#endif
  return tag.str();
}

void HashPart(boost::uuids::detail::sha1& hash, const std::string& role,
              const std::string& contents) {
  // Role and byte-count framing prevents different file boundaries from
  // producing the same preimage (for example task="ab", reference="c").
  const std::string prefix = role + '\0' + std::to_string(contents.size()) + '\0';
  hash.process_bytes(prefix.data(), prefix.size());
  hash.process_bytes(contents.data(), contents.size());
}

std::string ReadInput(const std::string& path, const std::string& role) {
  std::error_code error;
  if (!std::filesystem::is_regular_file(path, error) || error) {
    throw std::runtime_error("CppAD cache: missing or non-regular " + role +
                             " file: " + path);
  }
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    throw std::runtime_error("CppAD cache: cannot read " + role + " file: " + path);
  }
  const std::string contents{std::istreambuf_iterator<char>(input),
                             std::istreambuf_iterator<char>()};
  if (input.bad()) {
    throw std::runtime_error("CppAD cache: read failed for " + role + " file: " + path);
  }
  return contents;
}

}  // namespace

std::string ModelCacheDirectory(const std::string& cache_base,
                                const std::string& urdf_file,
                                const std::string& task_file,
                                const std::string& reference_file) {
  boost::uuids::detail::sha1 hash;
  HashPart(hash, "schema", kCacheSchema);
  HashPart(hash, "native-code", NativeCodeTag());
  HashPart(hash, "urdf", ReadInput(urdf_file, "URDF"));
  HashPart(hash, "task", ReadInput(task_file, "task"));
  HashPart(hash, "reference", ReadInput(reference_file, "reference"));
  boost::uuids::detail::sha1::digest_type digest;
  hash.get_digest(digest);
  std::ostringstream hex;
  hex << std::hex << std::setfill('0');
  for (const auto word : digest) hex << std::setw(8) << word;

  const auto base = cache_base.empty() ? std::filesystem::path(".")
                                      : std::filesystem::path(cache_base);
  return (base / (std::string(kCacheSchema) + "-" + hex.str())).string();
}

}  // namespace custom_dog_control
