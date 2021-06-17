#ifndef TRY_CATCH_SIGNAL_HPP
#define TRY_CATCH_SIGNAL_HPP

#ifdef _MSC_VER
#include "TryCatchSignalSEH.hpp"
#else
#include "TryCatchSignalSTD.hpp"
#endif

#endif // TRY_CATCH_SIGNAL_HPP

