#ifndef slic3r_SLA_SuppotstIslands_PointUtils_hpp_
#define slic3r_SLA_SuppotstIslands_PointUtils_hpp_

#include <libslic3r/Point.hpp>

namespace Slic3r::sla {

/// <summary>
/// Class which contain collection of static function
/// for work with Point and Points.
/// QUESTION: Is it only for SLA?
/// </summary>
class PointUtils
{
public:
    PointUtils() = delete;

    // is point p1 to p2 in counter clock wise order against center?
    static bool is_ccw(const Point &p1, const Point &p2, const Point &center);
};

} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_PointUtils_hpp_
