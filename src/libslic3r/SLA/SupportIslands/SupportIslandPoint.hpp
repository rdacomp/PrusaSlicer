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
class SupportIslandPoint
{
public:
    enum class Type: unsigned char {
        one_center_point,
        two_points,
        center_line,
        center_line_end,  // end of branch
        center_line_end2, // start of main path(only one per VD)
        center_line_end3, // end in continous sampling
        center_line_start, // first sample
        center_circle,
        center_circle_end, // circle finish by one point (one end per circle -
                           // need allign)
        center_circle_end2, // circle finish by multi points (one end per
                            // circle - need allign)
        outline, // keep position align with island outline 
        inner, // point inside wide part, without restriction on move

        undefined
    };

    Type type; 
    Point point;

public:
    /// <summary>
    /// constructor
    /// </summary>
    /// <param name="point">coordinate point inside a layer (in one slice)</param>
    /// <param name="type">type of support point</param>
    SupportIslandPoint(Point point, Type type = Type::undefined);

    /// <summary>
    /// virtual destructor to be inheritable
    /// </summary>
    virtual ~SupportIslandPoint() = default;

    /// <summary>
    /// static function to decide if type is possible to move or not
    /// </summary>
    /// <param name="type">type to distinguish</param>
    /// <returns>True when is possible to move, otherwise FALSE</returns>
    static bool can_move(const Type &type);
        
    /// <summary>
    /// static function to decide if type is possible to move or not
    /// </summary>
    /// <param name="type">type to distinguish</param>
    /// <returns>True when is possible to move, otherwise FALSE</returns>
    virtual bool can_move() const;

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

/// <summary>
/// DTO Support point laying on voronoi graph edge
/// Restriction to move only on Voronoi graph
/// </summary>
class SupportCenterIslandPoint : public SupportIslandPoint
{
public:
    // Define position on voronoi graph
    // Lose data when voronoi graph does NOT exist
    VoronoiGraph::Position position;
    // IMPROVE: not need ratio, only neighbor
    // const VoronoiGraph::Node::Neighbor* neighbor;

    // TODO: should earn when created
    const double max_distance = 1e6; // [in nm] --> 1 mm

public:
    SupportCenterIslandPoint(Point                  point,
                             VoronoiGraph::Position position,
                             Type                   type = Type::center_line);
    
    bool can_move() const override{ return true; }
    coord_t move(const Point &destination) override;
};

/// <summary>
/// DTO Support point laying on Outline of island
/// Restriction to move only on outline
/// </summary>
class SupportOutlineIslandPoint : public SupportIslandPoint
{
public:
    // index of line form island outline 
    size_t index;

public:
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
