#include "SupportIslandPoint.hpp"
#include "VoronoiGraphUtils.hpp"

using namespace Slic3r::sla;

SupportIslandPoint::SupportIslandPoint(Slic3r::Point point, Type type)
    : point(std::move(point)), type(type)
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

