#include "TryCatchSignal.hpp"

#ifdef _MSC_VER
#include "TryCatchSignalSEH.cpp"
#else
#include "TryCatchSignalSTD.cpp"
#endif
