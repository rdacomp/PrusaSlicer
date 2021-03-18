#ifndef slic3r_SLA_SuppotstIslands_PolygonUtils_hpp_
#define slic3r_SLA_SuppotstIslands_PolygonUtils_hpp_

#include <libslic3r/Polygon.hpp>

namespace Slic3r::sla {
/// <summary>
/// Class which contain collection of static function
/// for work with Polygon.
/// </summary>
class PolygonUtils
{
public:
    PolygonUtils() = delete;

    /// <summary>
    /// Create regular polygon with N points
    /// </summary>
    /// <param name="count_points">Count points of regular polygon</param>
    /// <param name="radius">Radius around center</param>
    /// <param name="center">Center point</param>
    /// <returns>Regular Polygon with CCW points</returns>
    static Polygon create_regular(size_t count_points, double radius = 10., const Point& center = Point(0,0));

    /// <summary>
    /// Create circle with N points
    /// alias for create regular
    /// </summary>
    /// <param name="radius">Radius of circle</param>
    /// <param name="count_points">Count points of circle</param>
    /// <param name="center">Center point</param>
    /// <returns>Regular Polygon with CCW points</returns>
    static Polygon create_circle(double radius, size_t count_points = 10, const Point& center = Point(0,0)){
        return create_regular(count_points, radius, center);
    }

    /// <summary>
    /// Create triangle with same length for all sides
    /// </summary>
    /// <param name="edge_size">triangel edge size</param>
    /// <returns>Equilateral triangle</returns>
    static Polygon create_equilateral_triangle(double edge_size);

    /// <summary>
    /// Create triangle with two side with same size
    /// </summary>
    /// <param name="side">Size of unique side</param>
    /// <param name="height">triangle height</param>
    /// <returns>Isosceles Triangle </returns>
    static Polygon create_isosceles_triangle(double side, double height);

    /// <summary>
    /// Create squar with center in [0,0]
    /// </summary>
    /// <param name="size"></param>
    /// <returns>Square</returns>
    static Polygon create_square(double size);

    /// <summary>
    /// Create rect with center in [0,0]
    /// </summary>
    /// <param name="width">width</param>
    /// <param name="height">height</param>
    /// <returns>Rectangle</returns>
    static Polygon create_rect(double width, double height);
};
} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_PolygonUtils_hpp_
