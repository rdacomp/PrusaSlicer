#include <catch2/catch.hpp>
#include <test_utils.hpp>

#include <libslic3r/ExPolygon.hpp>
#include <libslic3r/BoundingBox.hpp>
#include <libslic3r/ClipperUtils.hpp>

#include <libslic3r/SLA/SupportIslands/SampleConfig.hpp>
#include <libslic3r/SLA/SupportIslands/VoronoiGraphUtils.hpp>
#include <libslic3r/SLA/SupportIslands/SampleIslandUtils.hpp>
#include <libslic3r/SLA/SupportIslands/PolygonUtils.hpp>

#include "sla_test_utils.hpp"

using namespace Slic3r;
using namespace Slic3r::sla;

TEST_CASE("Overhanging point should be supported", "[SupGen]") {

    // Pyramid with 45 deg slope
    TriangleMesh mesh = make_pyramid(10.f, 10.f);
    mesh.rotate_y(float(PI));
    mesh.require_shared_vertices();
    mesh.WriteOBJFile("Pyramid.obj");

    sla::SupportPoints pts = calc_support_pts(mesh);

    // The overhang, which is the upside-down pyramid's edge
    Vec3f overh{0., 0., -10.};

    REQUIRE(!pts.empty());

    float dist = (overh - pts.front().pos).norm();

    for (const auto &pt : pts)
        dist = std::min(dist, (overh - pt.pos).norm());

    // Should require exactly one support point at the overhang
    REQUIRE(pts.size() > 0);
    REQUIRE(dist < 1.f);
}

double min_point_distance(const sla::SupportPoints &pts)
{
    sla::PointIndex index;

    for (size_t i = 0; i < pts.size(); ++i)
        index.insert(pts[i].pos.cast<double>(), i);

    auto d = std::numeric_limits<double>::max();
    index.foreach([&d, &index](const sla::PointIndexEl &el) {
        auto res = index.nearest(el.first, 2);
        for (const sla::PointIndexEl &r : res)
            if (r.second != el.second)
                d = std::min(d, (el.first - r.first).norm());
    });

    return d;
}

TEST_CASE("Overhanging horizontal surface should be supported", "[SupGen]") {
    double width = 10., depth = 10., height = 1.;

    TriangleMesh mesh = make_cube(width, depth, height);
    mesh.translate(0., 0., 5.); // lift up
    mesh.require_shared_vertices();
    mesh.WriteOBJFile("Cuboid.obj");

    sla::SupportPointGenerator::Config cfg;
    sla::SupportPoints pts = calc_support_pts(mesh, cfg);

    double mm2 = width * depth;

    REQUIRE(!pts.empty());
    REQUIRE(pts.size() * cfg.support_force() > mm2 * cfg.tear_pressure());
    REQUIRE(min_point_distance(pts) >= cfg.minimal_distance);
}

template<class M> auto&& center_around_bb(M &&mesh)
{
    auto bb = mesh.bounding_box();
    mesh.translate(-bb.center().template cast<float>());

    return std::forward<M>(mesh);
}

TEST_CASE("Overhanging edge should be supported", "[SupGen]") {
    float width = 10.f, depth = 10.f, height = 5.f;

    TriangleMesh mesh = make_prism(width, depth, height);
    mesh.rotate_y(float(PI)); // rotate on its back
    mesh.translate(0., 0., height);
    mesh.require_shared_vertices();
    mesh.WriteOBJFile("Prism.obj");

    sla::SupportPointGenerator::Config cfg;
    sla::SupportPoints pts = calc_support_pts(mesh, cfg);

    Linef3 overh{ {0.f, -depth / 2.f, 0.f}, {0.f, depth / 2.f, 0.f}};

    // Get all the points closer that 1 mm to the overhanging edge:
    sla::SupportPoints overh_pts; overh_pts.reserve(pts.size());

    std::copy_if(pts.begin(), pts.end(), std::back_inserter(overh_pts),
                 [&overh](const sla::SupportPoint &pt){
                     return line_alg::distance_to(overh, Vec3d{pt.pos.cast<double>()}) < 1.;
                 });

    REQUIRE(overh_pts.size() * cfg.support_force() > overh.length() * cfg.tear_pressure());
    double ddiff = min_point_distance(pts) - cfg.minimal_distance;
    REQUIRE(ddiff > - 0.1 * cfg.minimal_distance);
}

TEST_CASE("Hollowed cube should be supported from the inside", "[SupGen][Hollowed]") {
    TriangleMesh mesh = make_cube(20., 20., 20.);

    hollow_mesh(mesh, HollowingConfig{});

    mesh.WriteOBJFile("cube_hollowed.obj");

    auto bb = mesh.bounding_box();
    auto h  = float(bb.max.z() - bb.min.z());
    Vec3f mv = bb.center().cast<float>() - Vec3f{0.f, 0.f, 0.5f * h};
    mesh.translate(-mv);
    mesh.require_shared_vertices();

    sla::SupportPointGenerator::Config cfg;
    sla::SupportPoints pts = calc_support_pts(mesh, cfg);
    sla::remove_bottom_points(pts, mesh.bounding_box().min.z() + EPSILON);

    REQUIRE(!pts.empty());
}

TEST_CASE("Two parallel plates should be supported", "[SupGen][Hollowed]")
{
    double width = 20., depth = 20., height = 1.;

    TriangleMesh mesh = center_around_bb(make_cube(width + 5., depth + 5., height));
    TriangleMesh mesh_high = center_around_bb(make_cube(width, depth, height));
    mesh_high.translate(0., 0., 10.); // lift up
    mesh.merge(mesh_high);
    mesh.require_shared_vertices();

    mesh.WriteOBJFile("parallel_plates.obj");

    sla::SupportPointGenerator::Config cfg;
    sla::SupportPoints pts = calc_support_pts(mesh, cfg);
    sla::remove_bottom_points(pts, mesh.bounding_box().min.z() + EPSILON);

    REQUIRE(!pts.empty());
}


Slic3r::Polygon create_cross_roads(double size, double width)
{
    auto r1 = PolygonUtils::create_rect(5.3 * size, width);
    r1.rotate(3.14/4);
    r1.translate(2 * size, width / 2);
    auto r2 = PolygonUtils::create_rect(6.1 * size, 3 / 4. * width);
    r2.rotate(-3.14 / 5);
    r2.translate(3 * size, width / 2);
    auto r3 = PolygonUtils::create_rect(7.9 * size, 4 / 5. * width);
    r3.translate(2*size, width/2);
    auto r4 = PolygonUtils::create_rect(5 / 6. * width, 5.7 * size);
    r4.translate(-size,3*size);
    Polygons rr = union_(Polygons({r1, r2, r3, r4}));
    return rr.front();
}

ExPolygon create_trinagle_with_hole(double size)
{
    return ExPolygon(PolygonUtils::create_equilateral_triangle(size),
                     {{size / 4, size / 4},
                                                  {size / 2, size / 2},
                                                  {size / 2, size / 4}});
}

ExPolygon create_square_with_hole(double size, double hole_size)
{
    assert(sqrt(hole_size *hole_size / 2) < size);
    auto hole = PolygonUtils::create_square(hole_size);
    hole.rotate(M_PI / 4.); // 45
    hole.reverse();
    return ExPolygon(PolygonUtils::create_square(size), hole);
}

ExPolygon create_square_with_4holes(double size, double hole_size) {
    auto hole = PolygonUtils::create_square(hole_size);
    hole.reverse();
    double size_4 = size / 4;
    auto h1 = hole;
    h1.translate(size_4, size_4);
    auto h2 = hole;
    h2.translate(-size_4, size_4);
    auto h3   = hole;
    h3.translate(size_4, -size_4);
    auto h4   = hole;
    h4.translate(-size_4, -size_4);
    ExPolygon result(PolygonUtils::create_square(size));
    result.holes = Polygons({h1, h2, h3, h4});
    return result;
}

// boudary of circle
ExPolygon create_disc(double radius, double width, size_t count_line_segments)
{
    double width_2 = width / 2;
    auto   hole    = PolygonUtils::create_circle(radius - width_2,
                                            count_line_segments);
    hole.reverse();
    return ExPolygon(PolygonUtils::create_circle(radius + width_2,
                                                 count_line_segments),
                     hole);
}

Slic3r::Polygon create_V_shape(double height, double line_width, double angle = M_PI/4) {
    double angle_2 = angle / 2;
    auto   left_side  = PolygonUtils::create_rect(line_width, height);
    auto   right_side = left_side;
    right_side.rotate(-angle_2);
    double small_move = cos(angle_2) * line_width / 2;
    double side_move  = sin(angle_2) * height / 2 + small_move;
    right_side.translate(side_move,0);
    left_side.rotate(angle_2);
    left_side.translate(-side_move, 0);
    auto bottom = PolygonUtils::create_rect(4 * small_move, line_width);
    bottom.translate(0., -cos(angle_2) * height / 2 + line_width/2);
    Polygons polygons = union_(Polygons({left_side, right_side, bottom}));
    return polygons.front();
}

ExPolygons createTestIslands(double size)
{
    bool      useFrogLeg = false;    
    // need post reorganization of longest path
    ExPolygon mountains({{0., 0.},
                         {size, 0.},
                         {5 * size / 6, size},
                         {4 * size / 6, size / 6},
                         {3 * size / 7, 2 * size},
                         {2 * size / 7, size / 6},
                         {size / 7, size}});
    ExPolygons result = {
        // one support point
        ExPolygon(PolygonUtils::create_equilateral_triangle(size)), 
        ExPolygon(PolygonUtils::create_square(size)),
        ExPolygon(PolygonUtils::create_rect(size / 2, size)),
        ExPolygon(PolygonUtils::create_isosceles_triangle(size / 2, 3 * size / 2)), // small sharp triangle
        ExPolygon(PolygonUtils::create_circle(size / 2, 10)),
        create_square_with_4holes(size, size / 4),
        create_disc(size/4, size / 4, 10),
        ExPolygon(create_V_shape(2*size/3, size / 4)),

        // two support points
        ExPolygon(PolygonUtils::create_isosceles_triangle(size / 2, 3 * size)), // small sharp triangle
        ExPolygon(PolygonUtils::create_rect(size / 2, 3 * size)),
        ExPolygon(create_V_shape(1.5*size, size/3)),

        // tiny line support points
        ExPolygon(PolygonUtils::create_rect(size / 2, 10 * size)), // long line
        ExPolygon(create_V_shape(size*4, size / 3)),
        ExPolygon(create_cross_roads(size, size / 3)),
        create_disc(3*size, size / 4, 30),
        create_square_with_4holes(5 * size, 5 * size / 2 - size / 3),

        // still problem
        // three support points
        ExPolygon(PolygonUtils::create_equilateral_triangle(3 * size)), 
        ExPolygon(PolygonUtils::create_circle(size, 20)),

        mountains, 
        create_trinagle_with_hole(size),
        create_square_with_hole(size, size / 2),
        create_square_with_hole(size, size / 3)
    };
    
    if (useFrogLeg) {
        TriangleMesh            mesh = load_model("frog_legs.obj");
        TriangleMeshSlicer      slicer{&mesh};
        std::vector<float>      grid({0.1f});
        std::vector<ExPolygons> slices;
        slicer.slice(grid, SlicingMode::Regular, 0.05f, &slices, [] {});
        ExPolygon frog_leg = slices.front()[1];
        result.push_back(frog_leg);
    }
    return result;
}

Points createNet(const BoundingBox& bounding_box, double distance)
{ 
    Point  size       = bounding_box.size();
    double distance_2 = distance / 2;
    int    cols1 = static_cast<int>(floor(size.x() / distance))+1;
    int    cols2 = static_cast<int>(floor((size.x() - distance_2) / distance))+1;
    // equilateral triangle height with side distance
    double h      = sqrt(distance * distance - distance_2 * distance_2);
    int    rows   = static_cast<int>(floor(size.y() / h)) +1;
    int    rows_2 = rows / 2;
    size_t count_points = rows_2 * (cols1 + static_cast<size_t>(cols2));
    if (rows % 2 == 1) count_points += cols2;
    Points result;
    result.reserve(count_points);
    bool   isOdd = true;
    Point offset = bounding_box.min;
    double x_max = offset.x() + static_cast<double>(size.x());
    double y_max  = offset.y() + static_cast<double>(size.y());
    for (double y = offset.y(); y <= y_max; y += h) {
        double x_offset = offset.x();
        if (isOdd) x_offset += distance_2;
        isOdd = !isOdd;
        for (double x = x_offset; x <= x_max; x += distance) {
            result.emplace_back(x, y);
        }
    }
    assert(result.size() == count_points);
    return result; 
}

// create uniform triangle net and return points laying inside island
Points rasterize(const ExPolygon &island, double distance) {
    BoundingBox bb;
    for (const Point &pt : island.contour.points) bb.merge(pt);
    Points      fullNet = createNet(bb, distance);
    Points result;
    result.reserve(fullNet.size());
    std::copy_if(fullNet.begin(), fullNet.end(), std::back_inserter(result),
                 [&island](const Point &p) { return island.contains(p); });
    return result;
}

SupportIslandPoints test_island_sampling(const ExPolygon &   island,
                                        const SampleConfig &config)
{
    auto points = SupportPointGenerator::uniform_cover_island(island, config);
    Points chck_points = rasterize(island, config.head_radius); // TODO: Use resolution of printer

    bool is_ok = true;
    double              max_distance = config.max_distance;
    std::vector<double> point_distances(chck_points.size(),
                                        {max_distance + 1});
    for (size_t index = 0; index < chck_points.size(); ++index) { 
        const Point &chck_point  = chck_points[index];
        double &min_distance = point_distances[index];
        bool         exist_close_support_point = false;
        for (auto &island_point : points) {
            Point& p = island_point.point;
            Point abs_diff(fabs(p.x() - chck_point.x()),
                           fabs(p.y() - chck_point.y()));
            if (abs_diff.x() < min_distance && abs_diff.y() < min_distance) {
                double distance = sqrt((double) abs_diff.x() * abs_diff.x() +
                                       (double) abs_diff.y() * abs_diff.y());
                if (min_distance > distance) {
                    min_distance = distance;
                    exist_close_support_point = true;
                };
            }
        }
        if (!exist_close_support_point) is_ok = false;
    }

    if (!is_ok) { // visualize
        static int  counter              = 0;
        BoundingBox bb;
        for (const Point &pt : island.contour.points) bb.merge(pt);
        SVG svg("Error" + std::to_string(++counter) + ".svg", bb);
        svg.draw(island, "blue", 0.5f);
        for (auto p : points)
            svg.draw(p.point, "lightgreen", config.head_radius);
        for (size_t index = 0; index < chck_points.size(); ++index) {
            const Point &chck_point = chck_points[index];
            double       distance   = point_distances[index];
            bool         isOk       = distance < max_distance;
            std::string  color      = (isOk) ? "gray" : "red";
            svg.draw(chck_point, color, config.head_radius / 4);
        }
    }
    CHECK(!points.empty());
    //CHECK(is_ok);

    // all points must be inside of island
    for (const auto &point : points) { CHECK(island.contains(point.point)); }
    return points;
}

SampleConfig create_sample_config(double size) {
    SampleConfig cfg;
    cfg.max_distance   = 3 * size + 0.1;
    cfg.head_radius = size / 4;
    cfg.minimal_distance_from_outline = cfg.head_radius + size/10;
    cfg.max_length_for_one_support_point = 2*size;
    cfg.max_length_for_two_support_points = 4*size;
    cfg.max_width_for_center_supportr_line = size;
    cfg.max_width_for_zig_zag_supportr_line = 2*size;
    return cfg;
}

#include <libslic3r/Geometry.hpp>
#include <libslic3r/VoronoiOffset.hpp>
TEST_CASE("Sampling speed test on FrogLegs", "[VoronoiSkeleton]")
{
    TriangleMesh            mesh = load_model("frog_legs.obj");
    TriangleMeshSlicer      slicer{&mesh};
    std::vector<float>      grid({0.1f});
    std::vector<ExPolygons> slices;
    slicer.slice(grid, SlicingMode::Regular, 0.05f, &slices, [] {});
    ExPolygon frog_leg = slices.front()[1];
    SampleConfig cfg = create_sample_config(3e7);

    using VD = Slic3r::Geometry::VoronoiDiagram;
    VD    vd;
    Lines lines = to_lines(frog_leg);
    construct_voronoi(lines.begin(), lines.end(), &vd);
    Slic3r::Voronoi::annotate_inside_outside(vd, lines);
    
    for (int i = 0; i < 100; ++i) {
        VoronoiGraph::ExPath longest_path;
        VoronoiGraph skeleton = VoronoiGraphUtils::getSkeleton(vd, lines);
        auto samples = SampleIslandUtils::sample_voronoi_graph(skeleton, cfg, longest_path);
    }
}

TEST_CASE("Small islands should be supported in center", "[SupGen][VoronoiSkeleton]")
{
    double       size = 3e7;
    SampleConfig cfg  = create_sample_config(size);
    ExPolygons islands = createTestIslands(size);
    for (auto &island : islands) {
        auto   points = test_island_sampling(island, cfg);
        //cgal_test(points, island);
        double angle  = 3.14 / 3; // cca 60 degree

        island.rotate(angle);
        auto pointsR = test_island_sampling(island, cfg);
        //for (Point &p : pointsR) p.rotate(-angle);
        // points should be equal to pointsR
    }
}
