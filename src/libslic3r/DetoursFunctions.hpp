#ifndef slic3r_DetoursFunctions_hpp_
#define slic3r_DetoursFunctions_hpp_

#ifdef  WIN32
#include <windows.h>
#include <vector>
#include <string>
#include "../detours/detours.h"

// Detours LoadLibrary functions stated bellow


//#ifdef __cplusplus
//extern "C" {
//#endif
/*
bool detourLoadLibrary();

// New names of hooked functions - To be called instead of original funtions
extern HMODULE(WINAPI* TrueLoadLibraryA)(LPCSTR lpLibFileName);// = LoadLibraryA;
extern HMODULE(WINAPI* TrueLoadLibraryW)(LPCWSTR lpLibFileName);// = LoadLibraryW;
extern HMODULE(WINAPI* TrueLoadLibraryExA)(LPCSTR  lpLibFileName, HANDLE hFile, DWORD dwFlags);// = LoadLibraryExA;
extern HMODULE(WINAPI* TrueLoadLibraryExW)(LPCWSTR lpLibFileName, HANDLE hFile, DWORD dwFlags);// = LoadLibraryExW;
*/
//#ifdef __cplusplus
//}
//#endif
// Detour functions
HMODULE WINAPI FakeLoadLibraryA(LPCSTR  lpLibFileName);
HMODULE WINAPI FakeLoadLibraryW(LPCWSTR lpLibFileName);
HMODULE WINAPI FakeLoadLibraryExA(LPCSTR  lpLibFileName, HANDLE hFile, DWORD dwFlags);
HMODULE WINAPI FakeLoadLibraryExW(LPCWSTR lpLibFileName, HANDLE hFile, DWORD dwFlags);



class DetourLoadLibrary
{
public:
    static bool detourLoadLibrary();
    // New names of hooked functions - To be called instead of original funtions
    static inline HMODULE(WINAPI* TrueLoadLibraryA)  (LPCSTR  lpLibFileName) = LoadLibraryA;
    static inline HMODULE(WINAPI* TrueLoadLibraryW)  (LPCWSTR lpLibFileName) = LoadLibraryW;
    static inline HMODULE(WINAPI* TrueLoadLibraryExA)(LPCSTR  lpLibFileName, HANDLE hFile, DWORD dwFlags) = LoadLibraryExA;
    static inline HMODULE(WINAPI* TrueLoadLibraryExW)(LPCWSTR lpLibFileName, HANDLE hFile, DWORD dwFlags) = LoadLibraryExW;
    
    static bool isBlacklisted(std::string  dllpath);
    static bool isBlacklisted(std::wstring dllpath);
private:
    static const std::vector<std::wstring> blacklistDLL;
};

#endif //WIN32

#endif //slic3r_DetoursFunctions_hpp_