#ifndef slic3r_filesystem_hpp_
#define slic3r_filesystem_hpp_

#include <string>

#if __cplusplus >= 201703L && __has_include(<filesystem>)
    // Visual Studio 2019, new GCC & clang
    #include <filesystem>
#else // __cpp_lib_filesystem
    // Old Visual Studio, GCC & clang.
    #include <experimental/filesystem>
#endif // __cpp_lib_filesystem

namespace Slic3r {

#if __cplusplus >= 201703L && __has_include(<filesystem>)
    // Visual Studio 2019, new GCC & clang
    namespace filesystem = filesystem;
#else // __cpp_lib_filesystem
    // Old Visual Studio, GCC & clang.
    namespace filesystem = std::experimental::filesystem;
#endif // __cpp_lib_filesystem

filesystem::path gen_temp_file_path(const std::string &prefix, const std::string &suffix = std::string());

} // namespace Slic3r

#endif // slic3r_filesystem_hpp_
