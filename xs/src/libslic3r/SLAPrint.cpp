#include "SLAPrint.hpp"

namespace Slic3r {

void SLAPrint::process()
{

}

bool SLAPrint::canceled() const
{
    return false;
}

void SLAPrint::cancel()
{

}

void SLAPrint::restart()
{

}

bool SLAPrint::apply_config(DynamicPrintConfig cfg)
{
    return false;
}

void SLAPrint::export_print_data(const std::string &path)
{

}

Polyline SLAPrint::bed_shape() const
{
    return Polyline();
}

}
