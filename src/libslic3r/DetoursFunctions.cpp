#include "DetoursFunctions.hpp"
#include "LibraryCheck.hpp"
#include <cstdio>


namespace Slic3r {
#ifdef  WIN32

bool DetourLoadLibrary::detour_load_library()
{
    DetourRestoreAfterWith();

    DetourTransactionBegin();
    DetourUpdateThread(GetCurrentThread());
    DetourAttach(&(PVOID&)DetourLoadLibrary::TrueLoadLibraryA, FakeLoadLibraryA);
    DetourTransactionCommit();

    DetourTransactionBegin();
    DetourUpdateThread(GetCurrentThread());
    DetourAttach(&(PVOID&)DetourLoadLibrary::TrueLoadLibraryW, FakeLoadLibraryW);
    DetourTransactionCommit();

    DetourTransactionBegin();
    DetourUpdateThread(GetCurrentThread());
    DetourAttach(&(PVOID&)DetourLoadLibrary::TrueLoadLibraryExA, FakeLoadLibraryExA);
    DetourTransactionCommit();

    DetourTransactionBegin();
    DetourUpdateThread(GetCurrentThread());
    DetourAttach(&(PVOID&)DetourLoadLibrary::TrueLoadLibraryExW, FakeLoadLibraryExW);
    DetourTransactionCommit();
    return true;
}

HMODULE WINAPI FakeLoadLibraryA(LPCSTR  lpLibFileName)
{
    //std::printf("LoadLibraryA: %s\n", lpLibFileName);
    if (LibraryCheck::is_blacklisted(std::string(lpLibFileName)))
        return NULL;
    return DetourLoadLibrary::TrueLoadLibraryA(lpLibFileName);
}
HMODULE WINAPI FakeLoadLibraryW(LPCWSTR lpLibFileName)
{
    //std::wprintf(L"LoadLibraryW: %s\n", lpLibFileName);
    if (LibraryCheck::is_blacklisted(std::wstring(lpLibFileName)))
        return NULL;
    return DetourLoadLibrary::TrueLoadLibraryW(lpLibFileName);
}
HMODULE WINAPI FakeLoadLibraryExA(LPCSTR  lpLibFileName, HANDLE hFile, DWORD dwFlags)
{
    //std::printf("LoadLibraryExA: %s\n", lpLibFileName);
    if (LibraryCheck::is_blacklisted(std::string(lpLibFileName)))
        return NULL;
    return DetourLoadLibrary::TrueLoadLibraryExA(lpLibFileName, hFile, dwFlags);
}
HMODULE WINAPI FakeLoadLibraryExW(LPCWSTR lpLibFileName, HANDLE hFile, DWORD dwFlags)
{
    //std::wprintf(L"LoadLibraryExW: %s\n", lpLibFileName);
    if (LibraryCheck::is_blacklisted(std::wstring(lpLibFileName)))
        return NULL;
    return DetourLoadLibrary::TrueLoadLibraryExW(lpLibFileName, hFile, dwFlags);
}
#endif //WIN32

} // namespace Slic3r 
