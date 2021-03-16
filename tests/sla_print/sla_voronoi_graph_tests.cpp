#include "sla_test_utils.hpp"

#include <libslic3r/SLA/SupportIslands/VoronoiGraphUtils.hpp>

using namespace Slic3r;
using namespace Slic3r::sla;

void check(Slic3r::Points points, double max_distance) {
    using VD = Slic3r::Geometry::VoronoiDiagram;
    VD             vd;
    construct_voronoi(points.begin(), points.end(), &vd);
    double max_area = 3.15 * 4 * max_distance*max_distance; // circle
    for (const VD::cell_type &cell : vd.cells()) {
        Slic3r::Polygon polygon = VoronoiGraphUtils::to_polygon(cell, points, max_distance);
        CHECK(polygon.area() < max_area);
    }
}

TEST_CASE("Polygon from cell", "[Voronoi]")
{
    double  max_distance = 1e7;
    coord_t size         = (int) (4e6);
    coord_t half_size         = size/2;

    Slic3r::Points two_cols({Point(0, 0), Point(size, 0)});
    check(two_cols, max_distance);

    Slic3r::Points two_rows({Point(0, 0), Point(0, size)});
    check(two_rows, max_distance);

    Slic3r::Points two_diag({Point(0, 0), Point(size, size)});
    check(two_diag, max_distance);

    Slic3r::Points three({Point(0, 0), Point(size, 0), Point(half_size, size)});
    check(three, max_distance);

    Slic3r::Points middle_point({Point(0, 0), Point(size, half_size),
                                 Point(-size, half_size), Point(0, -size)});
    check(middle_point, max_distance);

    Slic3r::Points middle_point2({Point(half_size, half_size), Point(-size, -size), Point(-size, size),
                                  Point(size, -size), Point(size, size)});
    check(middle_point2, max_distance);
} 


