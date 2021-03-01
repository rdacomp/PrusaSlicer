#ifndef slic3r_SLA_SuppotstIslands_Parabola_hpp_
#define slic3r_SLA_SuppotstIslands_Parabola_hpp_

#include <libslic3r/Line.hpp>
#include <libslic3r/Point.hpp>

namespace Slic3r::sla {

/// <summary>
/// DTO store prabola params
/// A parabola can be defined geometrically as a set of points (locus of points) in the Euclidean plane:
/// Where distance from focus point is same as distance from line(directrix).
/// </summary>
struct Parabola
{
    Line  directrix;
    Point focus;

    Parabola(Line directrix, Point focus)
        : directrix(std::move(directrix)), focus(std::move(focus))
    {}
};

} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_Parabola_hpp_
