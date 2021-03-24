#ifndef slic3r_SLA_SuppotstIslands_LineUtils_hpp_
#define slic3r_SLA_SuppotstIslands_LineUtils_hpp_

#include <optional>
#include <tuple>
#include <string>
#include <libslic3r/Line.hpp>
#include <libslic3r/SVG.hpp>

namespace Slic3r::sla {

/// <summary>
/// Class which contain collection of static function
/// for work with Line and Lines.
/// QUESTION: Is it only for SLA?
/// </summary>
class LineUtils
{
public:
    LineUtils() = delete;

    /// <summary>
    /// Sort lines to be in counter clock wise order only by point Line::a and function std::atan2
    /// </summary>
    /// <param name="lines">Lines to be sort</param>
    /// <param name="center">Center for CCW order</param>
    static void sort_CCW(Lines &lines, const Point &center);

    /// <summary>
    /// Create line segment intersection of line and circle
    /// </summary>
    /// <param name="line">Input line.</param>
    /// <param name="center">Circle center.</param>
    /// <param name="radius">Circle radius.</param>
    /// <returns>Chord -> line segment inside circle</returns>
    static std::optional<Slic3r::Line> crop_line(const Line & line,
                                                 const Point &center,
                                                 double       radius);
    static std::optional<Slic3r::Linef> crop_line(const Linef & line,
                                                 const Point &center,
                                                 double       radius);
    /// <summary>
    /// Create line segment intersection of ray and circle, when exist
    /// </summary>
    /// <param name="ray">Input ray.</param>
    /// <param name="center">Circle center.</param>
    /// <param name="radius">Circle radius.</param>
    /// <returns>Chord -> line segment inside circle</returns>
    static std::optional<Slic3r::Line> crop_ray(const Line & ray,
                                                const Point &center,
                                                double       radius);
    static std::optional<Slic3r::Linef> crop_ray(const Linef & ray,
                                                const Point &center,
                                                double       radius);
    /// <summary>
    /// Create line segment intersection of half ray(start point and direction) and circle, when exist
    /// </summary>
    /// <param name="half_ray">Use Line::a as start point and Line::b as direction but no limit</param>
    /// <param name="center">Circle center.</param>
    /// <param name="radius">Circle radius.</param>
    /// <returns>Chord -> line segment inside circle</returns>
    static std::optional<Slic3r::Line> crop_half_ray(const Line & half_ray,
                                                     const Point &center,
                                                     double       radius);
    static std::optional<Slic3r::Linef> crop_half_ray(const Linef & half_ray,
                                                     const Point &center,
                                                     double       radius);

    /// <summary>
    /// check if line is parallel to Y
    /// </summary>
    /// <param name="line">Input line</param>
    /// <returns>True when parallel otherwise FALSE</returns>
    static bool is_parallel_y(const Line &line);
    static bool is_parallel_y(const Linef &line);

    /// <summary>
    /// Create parametric coeficient
    /// ax + by + c = 0
    /// Can't be parallel to Y axis - check by function is_parallel_y
    /// </summary>
    /// <param name="line">Input line - cant be parallel with y axis</param>
    /// <returns>a, b, c</returns>
    static std::tuple<double, double, double> get_param(const Line &line);
    static std::tuple<double, double, double> get_param(const Linef &line);

    static void draw(SVG &       svg,
                     const Line &line,
                     const char *color        = "gray",
                     coordf_t    stroke_width = 0,
                     const char *name         = nullptr,
                     bool        side_points  = false,
                     const char *color_a      = "lightgreen",
                     const char *color_b      = "lightblue");
    static void draw(SVG &       svg,
                     const Lines &lines,
                     const char *color        = "gray",
                     coordf_t    stroke_width = 0,
                     bool ord         = false, // write order as text
                     bool        side_points  = false,
                     const char *color_a      = "lightgreen",
                     const char *color_b      = "lightblue");
};

} // namespace Slic3r::sla
#endif // slic3r_SLA_SuppotstIslands_LineUtils_hpp_
