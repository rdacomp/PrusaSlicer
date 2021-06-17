#ifndef TRY_CATCH_SIGNAL_STD
#define TRY_CATCH_SIGNAL_STD

#include <csignal>
#include <csetjmp>
#include <utility>
#include <array>
#include <functional>

#include <exception>

namespace Slic3r {

using SignalT = decltype (SIGSEGV);

namespace detail {

// Experimental signal catching and recovering feature. Used facilites:
// std::signal https://en.cppreference.com/w/cpp/utility/program/signal
// std::longjmp https://en.cppreference.com/w/cpp/utility/program/longjmp
//
// GCC comments: https://www.gnu.org/software/libc/manual/html_node/Longjmp-in-Handler.html
//
class TryCatchSignal {
    static std::sig_atomic_t m_flag;
    static std::jmp_buf m_jbuf;
    [[noreturn]] static void sig_catcher(int);

public:
    template<class TryFn, class CatchFn, size_t N>
    static void try_catch_signal(const SignalT (&sigs) [N], TryFn &&fn, CatchFn &&cfn)
    {
        m_flag = false;

        decltype(SIG_DFL) prev_handlers[N];

        for (size_t i = 0; i < N; i++)
            prev_handlers[i] = std::signal(sigs[i], sig_catcher);

        auto restore = [&prev_handlers, &sigs] {
            for (size_t i = 0; i < N; i++)
                std::signal(sigs[i], prev_handlers[i]);
        };

        setjmp(m_jbuf);

        if (!m_flag) {
            fn();
            restore();
        } else {
            restore();
            cfn();
        }
    }
};

} // namespace detail

template<class TryFn, class CatchFn, int N>
void try_catch_signal(const SignalT (&sigs)[N], TryFn &&fn, CatchFn &&cfn)
{
    detail::TryCatchSignal::try_catch_signal(sigs,
                                             std::forward<TryFn>(fn),
                                             std::forward<CatchFn>(cfn));
}

} // namespace Slic3r

#endif // TRY_CATCH_SIGNAL_STD
