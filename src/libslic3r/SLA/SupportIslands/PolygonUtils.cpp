#include "PolygonUtils.hpp"

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