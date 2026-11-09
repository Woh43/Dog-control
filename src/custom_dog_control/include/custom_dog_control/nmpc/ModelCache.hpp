#pragma once

#include <string>

namespace custom_dog_control {

// Namespace generated AD libraries by the complete model/config contents and
// the native code-generation ABI. Does not create or modify files/directories.
// Paths themselves are not hashed, so relocating identical inputs can reuse
// the same cache base. Missing/non-regular/unreadable inputs throw.
std::string ModelCacheDirectory(const std::string& cache_base,
                                const std::string& urdf_file,
                                const std::string& task_file,
                                const std::string& reference_file);

}  // namespace custom_dog_control
