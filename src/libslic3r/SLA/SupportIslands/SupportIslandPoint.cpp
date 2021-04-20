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

SupportCenterIslandPoint::SupportCenterIslandPoint(
    Point point, VoronoiGraph::Position position, Type type)
    : SupportIslandPoint(point, type), position(position)
{}
    
coord_t SupportIslandPoint::move(const Point &destination)
{
    Point diff = destination - point;
    point = destination;
    // TODO: check move out of island !!
    // + need island ExPolygon
    return diff.x() + diff.y(); // Manhatn distance
}

coord_t SupportCenterIslandPoint::move(const Point &destination)
{
    // TODO: For decide of move need information about
    // + search distance for allowed move over VG(count or distance)
        
    // move only along VD
    position = VoronoiGraphUtils::align(position, destination, max_distance);

    Point new_point  = VoronoiGraphUtils::create_edge_point(position);
    Point move       = new_point - point;
    point    = new_point;
    return abs(move.x()) + abs(move.y());
}

