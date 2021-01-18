#include "DetoursFunctions.hpp"

#ifdef  WIN32

HMODULE(WINAPI* TrueLoadLibraryA)(LPCSTR lpLibFileName) = LoadLibraryA;
HMODULE(WINAPI* TrueLoadLibraryW)(LPCWSTR lpLibFileName) = LoadLibraryW;
HMODULE(WINAPI* TrueLoadLibraryExA)(LPCSTR  lpLibFileName, HANDLE hFile, DWORD dwFlags) = LoadLibraryExA;
HMODULE(WINAPI* TrueLoadLibraryExW)(LPCWSTR lpLibFileName, HANDLE hFile, DWORD dwFlags) = LoadLibraryExW;

BOOL detourLoadLibrary()
{
    /*
    TrueLoadLibraryA = LoadLibraryA;
    TrueLoadLibraryW = LoadLibraryW;
    TrueLoadLibraryExA = LoadLibraryExA;
    TrueLoadLibraryExW = LoadLibraryExW;
    */
    //if (dwReason == DLL_PROCESS_ATTACH) {
    DetourRestoreAfterWith();

    DetourTransactionBegin();
    DetourUpdateThread(GetCurrentThread());
    DetourAttach(&(PVOID&)TrueLoadLibraryA, FakeLoadLibraryA);
    DetourAttach(&(PVOID&)TrueLoadLibraryW, FakeLoadLibraryW);
    DetourAttach(&(PVOID&)TrueLoadLibraryExA, FakeLoadLibraryExA);
    DetourAttach(&(PVOID&)TrueLoadLibraryExW, FakeLoadLibraryExW);
    DetourTransactionCommit();
    //}
    /*else if (dwReason == DLL_PROCESS_DETACH) {
        DetourTransactionBegin();
        DetourUpdateThread(GetCurrentThread());
        DetourDetach(&(PVOID&)TrueSleep, TimedSleep);
        DetourTransactionCommit();
    }
    return TRUE;
    */
    return TRUE;
}

HMODULE WINAPI FakeLoadLibraryA(LPCSTR  lpLibFileName) { return NULL; }
HMODULE WINAPI FakeLoadLibraryW(LPCWSTR lpLibFileName) { return NULL; }
HMODULE WINAPI FakeLoadLibraryExA(LPCSTR  lpLibFileName, HANDLE hFile, DWORD dwFlags) { return NULL; }
HMODULE WINAPI FakeLoadLibraryExW(LPCWSTR lpLibFileName, HANDLE hFile, DWORD dwFlags) { return NULL; }
#endif //WIN32