#ifndef DIRECT_HPP
#define DIRECT_HPP

#include "nlopt_boilerplate.hpp"

namespace libnest2d { namespace opt {

class DirectOptimizer: public NloptOptimizer {
public:
    inline explicit DirectOptimizer(const StopCriteria& scr = {}):
        NloptOptimizer(method2nloptAlg(Method::G_DIRECT), scr) {}

    inline DirectOptimizer& localMethod(Method m) {
        localmethod_ = m;
        return *this;
    }
};

template<>
struct OptimizerSubclass<Method::G_DIRECT> { using Type = DirectOptimizer; };

template<> inline DirectOptimizer GlobalOptimizer<Method::G_DIRECT>(
        Method localm, const StopCriteria& scr )
{
    return DirectOptimizer (scr).localMethod(localm);
}

}
}

#endif // DIRECT_HPP
