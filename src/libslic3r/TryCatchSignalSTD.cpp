#include "TryCatchSignalSTD.hpp"

std::sig_atomic_t Slic3r::detail::TryCatchSignal::m_flag = false;
std::jmp_buf Slic3r::detail::TryCatchSignal::m_jbuf;

[[noreturn]] void Slic3r::detail::TryCatchSignal::sig_catcher(int)
{
    m_flag = true;
    std::longjmp(m_jbuf, 0);
}
