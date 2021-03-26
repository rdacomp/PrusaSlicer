#ifndef slic3r_SLA_SuppotstIslands_SupportIslandPoint_hpp_
#define slic3r_SLA_SuppotstIslands_SupportIslandPoint_hpp_

#include <set>
#include <memory>
#include <libslic3r/Point.hpp>
#include "VoronoiGraph.hpp"

namespace Slic3r::sla {

/// <summary>
/// DTO position with information about source of support point
/// </summary>
struct SupportIslandPoint
{
    enum class Type: unsigned char {
        one_center_point,
        two_points,
        center_line,
        center_line_end,  // end of branch
        center_line_end2, // start of main path(only one per VD)
        center_circle,
        center_circle_end, // circle finish by one point (one end per circle -
                           // need allign)
        center_circle_end2, // circle finish by multi points (one end per
                            // circle - need allign)

        outline, // keep position align with island outline 
        undefined
    };

    Type type;
    Slic3r::Point point; // 2 coordinate point in a layer (in one slice)

    //SupportIslandPoint() : point(0, 0), type(Type::undefined) {}
    SupportIslandPoint(Slic3r::Point point, Type type = Type::undefined);
    virtual ~SupportIslandPoint() = default;

    static bool can_move(const Type &type)
    {
        // use shorter list
        /*
        static const std::set<Type> can_move({
            Type::center_line,
            Type::center_circle,
            Type::center_circle_end,
            Type::center_circle_end2});
        return can_move.find(type) != can_move.end();
        /*/     // switch comment center
        static const std::set<Type> cant_move({
            Type::one_center_point,
            Type::two_points,
            Type::center_line_end,
            Type::center_line_end2});
        return cant_move.find(type) == cant_move.end();
        //*/
    }

    virtual bool can_move() const { return can_move(type); }

    /// <summary>
    /// Move position of support point close to destination
    /// with support point restrictions
    /// </summary>
    /// <param name="destination">Wanted position</param>
    /// <returns>Move distance</returns>
    virtual coord_t move(const Point &destination);
};

using SupportIslandPointPtr = std::unique_ptr<SupportIslandPoint>;
using SupportIslandPoints = std::vector<SupportIslandPointPtr>;

struct SupportCenterIslandPoint : public SupportIslandPoint
{
    // Define position on voronoi graph
    // Lose data when voronoi graph does NOT exist
    VoronoiGraph::Position position;
    // IMPROVE: not need ratio, only neighbor
    // const VoronoiGraph::Node::Neighbor* neighbor;

    // TODO: should earn when created
    const double max_distance = 1e6; // [in nm] --> 1 mm

    SupportCenterIslandPoint(Slic3r::Point          point,
                             VoronoiGraph::Position position,
                             Type                   type = Type::center_line)
        : SupportIslandPoint(point, type), position(position)
    {}

    bool can_move() const override{ return true; }
    coord_t move(const Point &destination) override;
};

struct SupportOutlineIslandPoint : public SupportIslandPoint
{
    // index of line form island outline 
    size_t index;
    SupportOutlineIslandPoint(Slic3r::Point point,
                              size_t        index,
                              Type          type = Type::outline)
        : SupportIslandPoint(point, type), index(index)
    {}

    bool can_move() const override { return true; }

    coord_t move(const Point &destination) override
    { 
        // TODO: For decide of move need information about
        // + island outlines        \    May be 
        // + distance from outline  /    offseted outlines
        // + search distance for allowed move over outlines(count, distance)
        assert(false); // Not implemented
        return 0;
    }
};

} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_SupportIslandPoint_hpp_
