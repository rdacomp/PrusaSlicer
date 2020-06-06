#ifndef slic3r_filesystem_hpp_
#define slic3r_filesystem_hpp_

#include <string>

#ifdef CXX_FILESYSTEM_IS_EXPERIMENTAL
    // Old Visual Studio, GCC & clang.
    #include <experimental/filesystem>
#else // __cpp_lib_filesystem
    // Visual Studio 2019, new GCC & clang
    #include <filesystem>
#endif // __cpp_lib_filesystem

namespace Slic3r {

#ifdef CXX_FILESYSTEM_IS_EXPERIMENTAL
    // Old Visual Studio, GCC & clang.
    namespace filesystem = std::experimental::filesystem;
#else // __cpp_lib_filesystem    
    // Visual Studio 2019, new GCC & clang
    namespace filesystem = std::filesystem;
#endif // __cpp_lib_filesystem

filesystem::path gen_temp_file_path(const std::string &prefix, const std::string &suffix = std::string());

} // namespace Slic3r

#endif // slic3r_filesystem_hpp_
