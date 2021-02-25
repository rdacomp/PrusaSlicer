#include <catch2/catch.hpp>
#include <test_utils.hpp>

#include <libslic3r/ExPolygon.hpp>
#include <libslic3r/BoundingBox.hpp>

#include <libslic3r/SLA/SupportIslands/SampleConfig.hpp>

#include "sla_test_utils.hpp"

namespace Slic3r { namespace sla {

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

ExPolygons createTestIslands(double size)
{
    ExPolygon triangle(
        Polygon{{.0, .0},
                {size, .0},
                {size / 2., sqrt(size * size - size * size / 4)}});
    ExPolygon sharp_triangle(
        Polygon{{.0, size / 2}, {.0, .0}, {2 * size, .0}});
    ExPolygon triangle_with_hole({{.0, .0},
                                  {size, .0},
                                  {size / 2.,
                                   sqrt(size * size - size * size / 4)}},
                                 {{size / 4, size / 4},
                                  {size / 2, size / 2},
                                  {size / 2, size / 4}});
    ExPolygon square(Polygon{{.0, size}, {.0, .0}, {size, .0}, {size, size}});
    ExPolygon rect(
        Polygon{{.0, size}, {.0, .0}, {2 * size, .0}, {2 * size, size}});
    ExPolygon rect_with_hole({{-size, size}, // rect CounterClockWise
                              {-size, -size},
                              {size, -size},
                              {size, size}},
                             {{0., size / 2}, // inside rect ClockWise
                              {size / 2, 0.},
                              {0., -size / 2},
                              {-size / 2, 0.}});
    // need post reorganization of longest path
    ExPolygon mountains({{0., 0.},
                         {size, 0.},
                         {5 * size / 6, size},
                         {4 * size / 6, size / 6},
                         {3 * size / 7, 2 * size},
                         {2 * size / 7, size / 6},
                         {size / 7, size}});
    ExPolygon rect_with_4_hole(Polygon{{0., size}, // rect CounterClockWise
                                       {0., 0.},
                                       {size, 0.},
                                       {size, size}});
    // inside rects ClockWise
    double size5           = size / 5.;
    rect_with_4_hole.holes = Polygons{{{size5, 4 * size5},
                                       {2 * size5, 4 * size5},
                                       {2 * size5, 3 * size5},
                                       {size5, 3 * size5}},
                                      {{3 * size5, 4 * size5},
                                       {4 * size5, 4 * size5},
                                       {4 * size5, 3 * size5},
                                       {3 * size5, 3 * size5}},
                                      {{size5, 2 * size5},
                                       {2 * size5, 2 * size5},
                                       {2 * size5, size5},
                                       {size5, size5}},
                                      {{3 * size5, 2 * size5},
                                       {4 * size5, 2 * size5},
                                       {4 * size5, size5},
                                       {3 * size5, size5}}};

    size_t count_cirlce_lines = 1000; // test stack overfrow
    double r_CCW              = size / 2;
    double r_CW               = r_CCW - size / 6;
    // CCW: couter clock wise, CW: clock wise
    Points circle_CCW, circle_CW;
    circle_CCW.reserve(count_cirlce_lines);
    circle_CW.reserve(count_cirlce_lines);
    for (size_t i = 0; i < count_cirlce_lines; ++i) {
        double alpha = (2 * M_PI * i) / count_cirlce_lines;
        double sina  = sin(alpha);
        double cosa  = cos(alpha);
        circle_CCW.emplace_back(-r_CCW * sina, r_CCW * cosa);
        circle_CW.emplace_back(r_CW * sina, r_CW * cosa);
    }
    ExPolygon double_circle(circle_CCW, circle_CW);

    TriangleMesh            mesh = load_model("frog_legs.obj");
    TriangleMeshSlicer      slicer{&mesh};
    std::vector<float>      grid({0.1f});
    std::vector<ExPolygons> slices;
    slicer.slice(grid, SlicingMode::Regular, 0.05f, &slices, [] {});
    ExPolygon frog_leg = slices.front()[1]; //

    return {
        triangle,         square,
        sharp_triangle,   rect,
        rect_with_hole,   triangle_with_hole,
        rect_with_4_hole, mountains,
        double_circle
        //, frog_leg
    };
}

std::vector<Point> test_island_sampling(const ExPolygon &   island,
                                        const SampleConfig &config)
{
    auto points = SupportPointGenerator::uniform_cover_island(island, config);
    CHECK(!points.empty());

    // all points must be inside of island
    for (const auto &point : points) { CHECK(island.contains(point)); }
    return points;
}


TEST_CASE("Sampling speed test on FrogLegs", "[VoronoiSkeleton]")
{
    TriangleMesh            mesh = load_model("frog_legs.obj");
    TriangleMeshSlicer      slicer{&mesh};
    std::vector<float>      grid({0.1f});
    std::vector<ExPolygons> slices;
    slicer.slice(grid, SlicingMode::Regular, 0.05f, &slices, [] {});
    ExPolygon frog_leg = slices.front()[1];

    double       size = 3e7;
    SampleConfig cfg;
    cfg.max_distance   = size + 0.1;
    cfg.sample_size    = size / 5;
    cfg.start_distance = 0.2 * size; // radius of support head
    cfg.curve_sample   = 0.1 * size;
    cfg.max_length_for_one_support_point = 3 * size;

    for (int i = 0; i < 100; ++i) {
        auto points = SupportPointGenerator::uniform_cover_island(
            frog_leg, cfg);
    }
}


TEST_CASE("Small islands should be supported in center", "[SupGen][VoronoiSkeleton]")
{
    double size = 3e7;
    SampleConfig cfg;
    cfg.max_distance   = size + 0.1;
    cfg.sample_size    = size / 5;
    cfg.start_distance = 0.2 * size; // radius of support head
    cfg.curve_sample   = 0.1 * size;
    cfg.max_length_for_one_support_point = 3 * size;

    ExPolygons islands = createTestIslands(size);
    for (auto &island : islands) {
        auto   points = test_island_sampling(island, cfg);
        double angle  = 3.14 / 3; // cca 60 degree

        island.rotate(angle);
        auto pointsR = test_island_sampling(island, cfg);
        for (Point &p : pointsR) p.rotate(-angle);

        // points should be equal to pointsR
    }
}

}} // namespace Slic3r::sla
