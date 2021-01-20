#include "DetoursFunctions.hpp"

#include <cstdio>
#include <boost/nowide/convert.hpp>

#ifdef  WIN32
/*
HMODULE(WINAPI* DetourLoadLibrary::TrueLoadLibraryA)(LPCSTR lpLibFileName) = LoadLibraryA;
HMODULE(WINAPI* DetourLoadLibrary::TrueLoadLibraryW)(LPCWSTR lpLibFileName) = LoadLibraryW;
HMODULE(WINAPI* DetourLoadLibrary::TrueLoadLibraryExA)(LPCSTR  lpLibFileName, HANDLE hFile, DWORD dwFlags) = LoadLibraryExA;
HMODULE(WINAPI* DetourLoadLibrary::TrueLoadLibraryExW)(LPCWSTR lpLibFileName, HANDLE hFile, DWORD dwFlags) = LoadLibraryExW;
*/

//only dll name with .dll suffix
const std::vector<std::wstring> DetourLoadLibrary::blacklistDLL ({ L"ASProxy64.dll", L"NahimicOSD.dll" });

bool DetourLoadLibrary::detourLoadLibrary()
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

//returns
bool DetourLoadLibrary::isBlacklisted(std::wstring dllpath)
{
    std::wstring dllname = boost::filesystem::path(dllpath).filename().wstring();
    //std::transform(dllname.begin(), dllname.end(), dllname.begin(), std::tolower);
    if (std::find(DetourLoadLibrary::blacklistDLL.begin(), DetourLoadLibrary::blacklistDLL.end(), dllname) != DetourLoadLibrary::blacklistDLL.end()) {
        std::wprintf(L"%s is blacklisted\n", dllname.c_str());
        return true;
    }
    //std::wprintf(L"%s is NOT blacklisted\n", dllname.c_str());
    return false;
}
bool DetourLoadLibrary::isBlacklisted(std::string dllpath)
{
    return DetourLoadLibrary::isBlacklisted(boost::nowide::widen(dllpath));
}

HMODULE WINAPI FakeLoadLibraryA(LPCSTR  lpLibFileName)
{
    std::printf("LoadLibraryA: %s\n", lpLibFileName);
    if (DetourLoadLibrary::isBlacklisted(std::string(lpLibFileName)))
        return NULL;
    return DetourLoadLibrary::TrueLoadLibraryA(lpLibFileName);
    //return NULL;
}
HMODULE WINAPI FakeLoadLibraryW(LPCWSTR lpLibFileName)
{
    std::wprintf(L"LoadLibraryW: %s\n", lpLibFileName);
    if (DetourLoadLibrary::isBlacklisted(std::wstring(lpLibFileName)))
        return NULL;
    return DetourLoadLibrary::TrueLoadLibraryW(lpLibFileName);
    //return NULL;
}
HMODULE WINAPI FakeLoadLibraryExA(LPCSTR  lpLibFileName, HANDLE hFile, DWORD dwFlags)
{
    std::printf("LoadLibraryExA: %s\n", lpLibFileName);
    if (DetourLoadLibrary::isBlacklisted(std::string(lpLibFileName)))
        return NULL;
    return DetourLoadLibrary::TrueLoadLibraryExA(lpLibFileName, hFile, dwFlags);
    //return NULL;
}
HMODULE WINAPI FakeLoadLibraryExW(LPCWSTR lpLibFileName, HANDLE hFile, DWORD dwFlags)
{
    std::wprintf(L"LoadLibraryExW: %s\n", lpLibFileName);
    if (DetourLoadLibrary::isBlacklisted(std::wstring(lpLibFileName)))
        return NULL;
    return DetourLoadLibrary::TrueLoadLibraryExW(lpLibFileName, hFile, dwFlags);
    //return NULL;
}
#endif //WIN32