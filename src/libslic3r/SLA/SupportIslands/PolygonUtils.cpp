#include "PolygonUtils.hpp"
#include "libslic3r/Geometry.hpp"

using namespace Slic3r::sla;

Slic3r::Polygon PolygonUtils::create_regular(size_t       count_points,
                                             double       radius,
                                             const Point &center)
{
    assert(radius >= 1.);
    assert(count_points >= 3);
    auto is_in_limits = [](double value) {
        return (value < std::numeric_limits<coord_t>::max()) &&
               (value > std::numeric_limits<coord_t>::min());
    };
    Points points;
    points.reserve(count_points);
    double increase_angle = 2 * M_PI / count_points;
    for (int i = 0; i < count_points; ++i) {
        double angle = i * increase_angle;
        double x = cos(angle) * radius + center.x();
        assert(is_in_limits(x));
        double y = sin(angle) * radius + center.y();
        assert(is_in_limits(y));
        points.emplace_back(x, y);
    }
    return Polygon(points);
}

Slic3r::Polygon PolygonUtils::create_equilateral_triangle(double edge_size)
{
    return {{.0, .0},
            {edge_size, .0},
            {edge_size / 2., sqrt(edge_size * edge_size - edge_size * edge_size / 4)}};
}

Slic3r::Polygon PolygonUtils::create_isosceles_triangle(double side, double height)
{
    return {{-side / 2, 0.}, {side / 2, 0.}, {.0, height}};
}

Slic3r::Polygon PolygonUtils::create_square(double size)
{
    double size_2 = size / 2;
    return {{-size_2, size_2},
            {-size_2, -size_2},
            {size_2, -size_2},
            {size_2, size_2}};
}

Slic3r::Polygon PolygonUtils::create_rect(double width, double height)
{
    double x_2 = width / 2;
    double y_2 = height / 2;
    return {{-x_2, y_2}, {-x_2, -y_2}, {x_2, -y_2}, {x_2, y_2}};
}

bool PolygonUtils::is_ccw(const Polygon &polygon, const Point &center) {
    const Point *prev = &polygon.points.back();
    for (const Point &point : polygon.points) { 
        Geometry::Orientation o = Geometry::orient(center, *prev, point);
        if (o != Geometry::Orientation::ORIENTATION_CCW) return false;
        prev = &point;
    }
    return true;
}

bool PolygonUtils::is_not_self_intersect(const Polygon &polygon,
                                         const Point &  center)
{
    auto get_angle = [&center](const Point &point) {
        Point diff_point = point - center;
        return atan2(diff_point.y(), diff_point.x());
    };
    bool         found_circle_end = false; // only one can be on polygon
    double prev_angle = get_angle(polygon.points.back());
    for (const Point &point : polygon.points) {
        double angle = get_angle(point);
        if (angle < prev_angle) { 
            if (found_circle_end) return false;
            found_circle_end = true;
        }
        prev_angle = angle;
    }
    return true;
}
