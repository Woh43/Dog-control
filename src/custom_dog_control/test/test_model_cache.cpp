#include "custom_dog_control/nmpc/ModelCache.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <vector>

namespace custom_dog_control {
namespace {

class ModelCacheTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const auto pattern =
        (std::filesystem::temp_directory_path() / "qr-model-cache-XXXXXX").string();
    std::vector<char> buffer(pattern.begin(), pattern.end());
    buffer.push_back('\0');
    const char* directory = ::mkdtemp(buffer.data());
    ASSERT_NE(directory, nullptr);
    root_ = directory;
    cache_ = root_ / "cache";
    urdf_ = root_ / "dog.urdf";
    task_ = root_ / "task.info";
    reference_ = root_ / "reference.info";
    Write(urdf_, "<robot><mass value='13.84916'/><inertia ixx='0.03'/></robot>");
    Write(task_, "model_settings { recompileLibrariesCppAd false }");
    Write(reference_, "nominal_height 0.29");
  }

  void TearDown() override {
    if (!root_.empty()) {
      std::error_code ignored;
      std::filesystem::remove_all(root_, ignored);
    }
  }

  static void Write(const std::filesystem::path& path, const std::string& text) {
    std::ofstream output(path, std::ios::binary);
    if (!(output << text)) throw std::runtime_error("could not write test input");
  }

  std::string Directory() const {
    return ModelCacheDirectory(cache_.string(), urdf_.string(), task_.string(),
                                reference_.string());
  }

  std::filesystem::path root_, cache_, urdf_, task_, reference_;
};

TEST_F(ModelCacheTest, IdenticalContentsReuseCacheAfterInputRelocation) {
  const auto first = Directory();
  EXPECT_EQ(Directory(), first);
  EXPECT_EQ(std::filesystem::path(first).parent_path(), cache_);
  EXPECT_NE(first, cache_.string());
  EXPECT_FALSE(std::filesystem::exists(first));

  const auto relocated = root_ / "relocated.urdf";
  std::filesystem::copy_file(urdf_, relocated);
  EXPECT_EQ(ModelCacheDirectory(cache_.string(), relocated.string(), task_.string(),
                                 reference_.string()), first);
}

TEST_F(ModelCacheTest, BatteryMassAndInertiaCannotLoadOldGeneratedLibrary) {
  const auto old_namespace = std::filesystem::path(Directory());
  const auto old_library = old_namespace / "dynamics" / "cppad_generated" /
                           "dynamics_systemFlowMap_lib.so";
  std::filesystem::create_directories(old_library.parent_path());
  Write(old_library, "old generated model");

  Write(urdf_, "<robot><mass value='15.15'/><inertia ixx='0.03'/></robot>");
  const auto mass_namespace = Directory();
  EXPECT_NE(mass_namespace, old_namespace.string());
  EXPECT_FALSE(std::filesystem::exists(mass_namespace));
  EXPECT_TRUE(std::filesystem::exists(old_library));

  Write(urdf_, "<robot><mass value='15.15'/><inertia ixx='0.031'/></robot>");
  EXPECT_NE(Directory(), mass_namespace);
  EXPECT_NE(Directory(), old_namespace.string());
}

TEST_F(ModelCacheTest, TaskAndReferenceChangesIsolateGeneratedLibraries) {
  const auto original = Directory();
  Write(task_, "model_settings { recompileLibrariesCppAd true }");
  const auto changed_task = Directory();
  EXPECT_NE(changed_task, original);
  Write(reference_, "nominal_height 0.30");
  EXPECT_NE(Directory(), original);
  EXPECT_NE(Directory(), changed_task);
}

TEST_F(ModelCacheTest, FileBoundariesArePartOfTheFingerprint) {
  Write(task_, "ab");
  Write(reference_, "c");
  const auto first = Directory();
  Write(task_, "a");
  Write(reference_, "bc");
  EXPECT_NE(Directory(), first);
}

TEST_F(ModelCacheTest, MissingOrNonRegularInputsFailClearly) {
  std::filesystem::remove(urdf_);
  EXPECT_THROW(Directory(), std::runtime_error);
  std::filesystem::create_directory(urdf_);
  EXPECT_THROW(Directory(), std::runtime_error);
  std::filesystem::remove(urdf_);
  Write(urdf_, "<robot/>");
  std::filesystem::remove(task_);
  EXPECT_THROW(Directory(), std::runtime_error);
  Write(task_, "settings");
  std::filesystem::remove(reference_);
  EXPECT_THROW(Directory(), std::runtime_error);
}

}  // namespace
}  // namespace custom_dog_control
