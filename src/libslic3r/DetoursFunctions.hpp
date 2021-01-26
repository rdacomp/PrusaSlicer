#ifndef slic3r_DetoursFunctions_hpp_
#define slic3r_DetoursFunctions_hpp_

#ifdef  WIN32
#include <windows.h>
#include <vector>
#include <string>
#include "../detours/detours.h"
#endif //WIN32

namespace Slic3r {

#ifdef  WIN32
// Detour functions
HMODULE WINAPI FakeLoadLibraryA(LPCSTR  lpLibFileName);
HMODULE WINAPI FakeLoadLibraryW(LPCWSTR lpLibFileName);
HMODULE WINAPI FakeLoadLibraryExA(LPCSTR  lpLibFileName, HANDLE hFile, DWORD dwFlags);
HMODULE WINAPI FakeLoadLibraryExW(LPCWSTR lpLibFileName, HANDLE hFile, DWORD dwFlags);

class DetourLoadLibrary
{
public:
    static bool detour_load_library();
    // New names of hooked functions - To be called instead of original funtions
    static inline HMODULE(WINAPI* TrueLoadLibraryA)  (LPCSTR  lpLibFileName) = LoadLibraryA;
    static inline HMODULE(WINAPI* TrueLoadLibraryW)  (LPCWSTR lpLibFileName) = LoadLibraryW;
    static inline HMODULE(WINAPI* TrueLoadLibraryExA)(LPCSTR  lpLibFileName, HANDLE hFile, DWORD dwFlags) = LoadLibraryExA;
    static inline HMODULE(WINAPI* TrueLoadLibraryExW)(LPCWSTR lpLibFileName, HANDLE hFile, DWORD dwFlags) = LoadLibraryExW;  
};

#endif //WIN32

} // namespace Slic3r 

#endif //slic3r_DetoursFunctions_hpp_