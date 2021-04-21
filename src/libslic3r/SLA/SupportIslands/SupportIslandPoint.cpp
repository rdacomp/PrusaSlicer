#include "SupportIslandPoint.hpp"
#include "VoronoiGraphUtils.hpp"

using namespace Slic3r::sla;

SupportIslandPoint::SupportIslandPoint(Slic3r::Point point, Type type)
    : point(std::move(point)), type(type)
{}

bool SupportIslandPoint::can_move(const Type &type)
{
    // use shorter list
    /*
    static const std::set<Type> can_move({
        Type::center_line,
        Type::center_circle,
        Type::center_circle_end,
        Type::center_circle_end2, 
        Type::outline, 
        Type::inner
    });
    return can_move.find(type) != can_move.end();
    /*/     // switch comment center
    static const std::set<Type> cant_move({
        Type::one_center_point,
        Type::two_points,
        Type::center_line_end,
        Type::center_line_end2,
        Type::center_line_end3,
        Type::center_line_start
    });
    return cant_move.find(type) == cant_move.end();
    //*/
}

bool SupportIslandPoint::can_move() const { return can_move(type); }

coord_t SupportIslandPoint::move(const Point &destination)
{
    Point diff = destination - point;
    point      = destination;
    // TODO: check move out of island !!
    // + need island ExPolygon
    return diff.x() + diff.y(); // Manhatn distance
}

///////////////
// Point on VD
///////////////

SupportCenterIslandPoint::SupportCenterIslandPoint(
    VoronoiGraph::Position         position,
    const SampleConfig *   configuration,
    Type                           type)
    : SupportIslandPoint(VoronoiGraphUtils::create_edge_point(position), type)
    , configuration(configuration)
    , position(position)
{}

coord_t SupportCenterIslandPoint::move(const Point &destination)
{        
    // move only along VD
    // TODO: Start respect minimum distance from outline !!
    position =
        VoronoiGraphUtils::align(position, destination,
                                 configuration->max_align_distance);
    Point new_point  = VoronoiGraphUtils::create_edge_point(position);
    Point move       = new_point - point;
    point    = new_point;
    coord_t manhatn_distance = abs(move.x()) + abs(move.y());
    return manhatn_distance;
}

