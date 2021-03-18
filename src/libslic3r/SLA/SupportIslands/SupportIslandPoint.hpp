#ifndef slic3r_SLA_SuppotstIslands_SupportIslandPoint_hpp_
#define slic3r_SLA_SuppotstIslands_SupportIslandPoint_hpp_

#include <set>
#include <libslic3r/Point.hpp>
#include "VoronoiGraph.hpp"

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

    // Define position on voronoi graph
    // Lose data when voronoi graph does NOT exist
    VoronoiGraph::Position position;

    Type type;

    SupportIslandPoint(Slic3r::Point          point,
                       Type                   type,
                       VoronoiGraph::Position position)
        : point(std::move(point)), type(type), position(position)
    {}

    bool can_move() const{ 
        // use shorter list
        /*
        
        static const std::set<Type> can_move({
            Type::center_line, 
            Type::center_circle, 
            Type::center_circle_end,
            Type::center_circle_end2
        });
        return can_move.find(type) != can_move.end();
        
        /*/
        
        static const std::set<Type> cant_move({
            Type::one_center_point, 
            Type::two_points
            });
        return cant_move.find(type) == cant_move.end();
        
        //*/
    }
};

using SupportIslandPoints = std::vector<SupportIslandPoint>;

} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_SupportIslandPoint_hpp_
