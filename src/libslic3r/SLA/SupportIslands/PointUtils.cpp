#include "PointUtils.hpp"

using namespace Slic3r::sla;

bool PointUtils::is_ccw(const Point &p1, const Point &p2, const Point &center)
{
    Slic3r::Point v1 = p1 - center;
    Slic3r::Point v2 = p2 - center;

    double cross_product = v1.x() * (double) v2.y() -
                           v2.x() * (double) v1.y();
    return cross_product > 0;
}
