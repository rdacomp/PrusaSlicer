#include "LineUtils.hpp"
#include <libslic3r/Geometry.hpp>
#include <functional>
#include "VectorUtils.hpp"

using namespace Slic3r::sla;

// sort counter clock wise lines
void LineUtils::sort_CCW(Lines &lines, const Point& center)
{
    std::function<double(const Line &)> calc = [&center](const Line &line) {
        Point p = line.a - center;
        return std::atan2(p.y(), p.x());
    };
    VectorUtils::sort_by(lines, calc);
}

bool LineUtils::is_parallel_y(const Line &line) { 
    coord_t x_change = line.a.x() - line.b.x();
    return (x_change == 0);
}
bool LineUtils::is_parallel_y(const Linef &line)
{
    double x_change = line.a.x() - line.b.x();
    return (fabs(x_change) < std::numeric_limits<double>::epsilon());
}

std::optional<Slic3r::Line> LineUtils::crop_ray(const Line & ray,
                                                const Point &center,
                                                double       radius)
{
    if (is_parallel_y(ray)) {
        coord_t x        = ray.a.x();
        coord_t diff     = x - center.x();
        coord_t abs_diff = abs(diff);
        if (abs_diff > radius) return {};
        // create cross points
        double  move_y = sqrt(radius * radius - (double) x * x);
        coord_t y      = std::round(move_y);
        Point first(x, y);
        Point second(x,-y);
        return Line(first + center, second + center);
    } else {
        Line   moved_line(ray.a - center, ray.b - center);
        double a, b, c;
        std::tie(a, b, c) = get_param(moved_line);
        std::pair<Vec2d, Vec2d> points;
        int count = Slic3r::Geometry::ray_circle_intersections(
            radius, a, b, c, points);
        if (count != 2) return {};
        return Line(points.first.cast<coord_t>() + center,
                    points.second.cast<coord_t>() + center);
    }
}
std::optional<Slic3r::Linef> LineUtils::crop_ray(const Linef &ray,
                                                 const Point &center,
                                                 double       radius)
{
    Vec2d center_d = center.cast<double>();
    if (is_parallel_y(ray)) {
        double x        = ray.a.x();
        double diff     = x - center_d.x();
        double abs_diff = fabs(diff);
        if (abs_diff > radius) return {};
        // create cross points
        double  y = sqrt(radius * radius - x * x);
        Vec2d   first(x, y);
        Vec2d   second(x, -y);
        return Linef(first + center_d,
                     second + center_d);
    } else {
        Linef moved_line(ray.a - center_d, ray.b - center_d);
        double a, b, c;
        std::tie(a, b, c) = get_param(moved_line);
        std::pair<Vec2d, Vec2d> points;
        int count = Slic3r::Geometry::ray_circle_intersections(radius, a, b,
                                                               c, points);
        if (count != 2) return {};
        return Linef(points.first + center_d, points.second + center_d);
    }
}

std::optional<Slic3r::Line> LineUtils::crop_half_ray(const Line & half_ray,
                                                     const Point &center,
                                                     double       radius)
{
    auto segment = crop_ray(half_ray, center, radius);
    if (!segment.has_value()) return {};
    Point dir = half_ray.b - half_ray.a;
    using fnc = std::function<bool(const Point &)>;
    fnc use_point_x = [&half_ray, &dir](const Point &p) -> bool {
        return (p.x() > half_ray.a.x()) == (dir.x() > 0);
    };
    fnc use_point_y = [&half_ray, &dir](const Point &p) -> bool {
        return (p.y() > half_ray.a.y()) == (dir.y() > 0);
    };
    bool use_x = abs(dir.x()) > abs(dir.y());
    fnc use_point = (use_x) ? use_point_x : use_point_y;
    bool use_a = use_point(segment->a);
    bool use_b = use_point(segment->b);
    if (!use_a && !use_b) return {};
    if (use_a && use_b) return segment;
    return Line(half_ray.a, (use_a)?segment->a : segment->b);
}

std::optional<Slic3r::Linef> LineUtils::crop_half_ray(const Linef & half_ray,
                                                     const Point &center,
                                                     double       radius)
{
    auto segment = crop_ray(half_ray, center, radius);
    if (!segment.has_value()) return {};
    Vec2d dir       = half_ray.b - half_ray.a;
    using fnc       = std::function<bool(const Vec2d &)>;
    fnc use_point_x = [&half_ray, &dir](const Vec2d &p) -> bool {
        return (p.x() > half_ray.a.x()) == (dir.x() > 0);
    };
    fnc use_point_y = [&half_ray, &dir](const Vec2d &p) -> bool {
        return (p.y() > half_ray.a.y()) == (dir.y() > 0);
    };
    bool use_x     = fabs(dir.x()) > fabs(dir.y());
    fnc  use_point = (use_x) ? use_point_x : use_point_y;
    bool use_a     = use_point(segment->a);
    bool use_b     = use_point(segment->b);
    if (!use_a && !use_b) return {};
    if (use_a && use_b) return segment;
    return Linef(half_ray.a, (use_a) ? segment->a : segment->b);
}

std::optional<Slic3r::Line> LineUtils::crop_line(const Line & line,
                                                 const Point &center,
                                                 double       radius)
{
    auto segment = crop_ray(line, center, radius);
    if (!segment.has_value()) return {};

    Point dir       = line.b - line.a;
    using fnc       = std::function<bool(const Point &)>;
    fnc use_point_x = [&line, &dir](const Point &p) -> bool {
        return (dir.x() > 0) ? (p.x() > line.a.x()) && (p.x() < line.b.x()) :
                               (p.x() < line.a.x()) && (p.x() > line.b.x());
    };
    fnc use_point_y = [&line, &dir](const Point &p) -> bool {
        return (dir.y() > 0) ? (p.y() > line.a.y()) && (p.y() < line.b.y()) :
                               (p.y() < line.a.y()) && (p.y() > line.b.y());
    };
    bool use_x     = abs(dir.x()) > abs(dir.y());
    fnc  use_point = (use_x) ? use_point_x : use_point_y;
    bool use_a     = use_point(segment->a);
    bool use_b     = use_point(segment->b);
    if (!use_a && !use_b) return {};
    if (use_a && use_b) return segment;
    bool same_dir = (use_x) ?
        ((dir.x() > 0) == (segment->b.x() - segment->a.x()) > 0) :
        ((dir.y() > 0) == (segment->b.y() - segment->a.y()) > 0) ;
    if (use_a) { 
        if (same_dir) 
            return Line(segment->a, line.b);
        else 
            return Line(line.a, segment->a);
    } else { // use b
        if (same_dir)
            return Line(line.a, segment->b);
        else
            return Line(segment->b, line.b);
    }
}

std::optional<Slic3r::Linef> LineUtils::crop_line(const Linef & line,
                                                 const Point &center,
                                                 double       radius)
{
    auto segment = crop_ray(line, center, radius);
    if (!segment.has_value()) return {};

    Vec2d dir       = line.b - line.a;
    using fnc       = std::function<bool(const Vec2d &)>;
    fnc use_point_x = [&line, &dir](const Vec2d &p) -> bool {
        return (dir.x() > 0) ? (p.x() > line.a.x()) && (p.x() < line.b.x()) :
                               (p.x() < line.a.x()) && (p.x() > line.b.x());
    };
    fnc use_point_y = [&line, &dir](const Vec2d &p) -> bool {
        return (dir.y() > 0) ? (p.y() > line.a.y()) && (p.y() < line.b.y()) :
                               (p.y() < line.a.y()) && (p.y() > line.b.y());
    };
    bool use_x     = abs(dir.x()) > abs(dir.y());
    fnc  use_point = (use_x) ? use_point_x : use_point_y;
    bool use_a     = use_point(segment->a);
    bool use_b     = use_point(segment->b);
    if (!use_a && !use_b) return {};
    if (use_a && use_b) return segment;
    bool same_dir = (use_x) ? ((dir.x() > 0) ==
                               (segment->b.x() - segment->a.x()) > 0) :
                              ((dir.y() > 0) ==
                               (segment->b.y() - segment->a.y()) > 0);
    if (use_a) {
        if (same_dir)
            return Linef(segment->a, line.b);
        else
            return Linef(line.a, segment->a);
    } else { // use b
        if (same_dir)
            return Linef(line.a, segment->b);
        else
            return Linef(segment->b, line.b);
    }
}


std::tuple<double, double, double> LineUtils::get_param(const Line &line) {
    Vector normal = line.normal();
    double a = normal.x();
    double b = normal.y();
    double c = -a * line.a.x() - b * line.a.y();
    return {a, b, c};
}

std::tuple<double, double, double> LineUtils::get_param(const Linef &line)
{
    Vec2d  direction = line.b - line.a;
    Vec2d  normal(-direction.y(), direction.x());
    double a = normal.x();
    double b = normal.y();
    double c = -a * line.a.x() - b * line.a.y();
    return {a, b, c};
}

void LineUtils::draw(SVG &       svg,
                     const Line &line,
                     const char *color,
                     coordf_t stroke_width,
                     const char *name,
                     bool        side_points,
                     const char *color_a,
                     const char *color_b)
{
    svg.draw(line, color, stroke_width);
    bool use_name = name != nullptr;
    if (use_name) {
        Point middle = line.a/2 + line.b/2;
        svg.draw_text(middle, name, color);
    }
    if (side_points) {
        std::string name_a = (use_name) ? "A" : (std::string("A_") + name);            
        std::string name_b = (use_name) ? "B" : (std::string("B_") + name);
        svg.draw_text(line.a, name_a.c_str(), color_a);
        svg.draw_text(line.b, name_b.c_str(), color_b);
    }
}

double LineUtils::perp_distance(const Linef &line, Vec2d p)
{
    Vec2d v  = line.b - line.a; // direction
    Vec2d va = p - line.a;
    return std::abs(cross2(v, va)) / v.norm();
}

void LineUtils::draw(SVG &        svg,
                     const Lines &lines,
                     const char * color,
                     coordf_t     stroke_width,
                     bool         ord,
                     bool         side_points,
                     const char * color_a,
                     const char * color_b)
{
    for (const auto &line : lines) {
        draw(svg, line, color, stroke_width,
            (ord) ? std::to_string(&line - &lines.front()).c_str() : nullptr,
            side_points, color_a, color_b);
    }
}