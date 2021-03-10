#ifndef slic3r_SLA_SuppotstIslands_SupportIslandPoint_hpp_
#define slic3r_SLA_SuppotstIslands_SupportIslandPoint_hpp_

#include <libslic3r/Point.hpp>

namespace Slic3r::sla {

/// <summary>
/// DTO position with information about source of support point
/// </summary>
struct SupportIslandPoint
{
    enum class Type : unsigned char {
        one_center_point,
        two_points,
        center_line,
        center_circle,
        center_circle_end,
        center_circle_end2,
        undefined
    };

    Slic3r::Point point; // 2 point in layer coordinate
    Type type;

    SupportIslandPoint(Slic3r::Point point, Type type = Type::undefined)
        : point(std::move(point)), type(type)
    {}
};

using SupportIslandPoints = std::vector<SupportIslandPoint>;

} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_SupportIslandPoint_hpp_
