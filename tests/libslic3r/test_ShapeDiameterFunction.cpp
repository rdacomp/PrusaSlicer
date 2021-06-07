#include <catch2/catch.hpp>
#include <test_utils.hpp>
#include <libslic3r/ShapeDiameterFunction.hpp>
#include <libslic3r/SVG.hpp> // visualization

using namespace Slic3r;

void itsXY_to_svg(SVG& svg, const indexed_triangle_set &its, const std::string& color, coord_t stroke_width, float scale) {
    for (const auto &face : its.indices) {
        Point        points[3];
        Point sum(0,0);
        for (int i = 0; i < 3; i++) {
            const Vec3f &v = its.vertices[face[i]];
            points[i] = Point(v.x()*scale, v.y()*scale);
            sum += points[i];
        }
        Point center = sum / 3;
        size_t face_index = &face - &its.indices.front();
        svg.draw_text(center, ("t"+std::to_string(face_index)).c_str(), color.c_str());

        for (int i = 0; i < 3; i++) { 
            Line l(points[i], points[(i+1)%3]);
            svg.draw(l, color, stroke_width);
        }
    }
    for (const auto &v : its.vertices) {
        Point p(v.x() * scale, v.y() * scale);
        size_t vertex_index = &v - &its.vertices.front();
        svg.draw_text(p, ("v" + std::to_string(vertex_index)).c_str(), color.c_str());
    }
}

void storeXY_to_svg(const indexed_triangle_set &in,
                    const indexed_triangle_set &out,
                    const std::string &         name)
{
    std::string COLOR_IN = "green";
    std::string COLOR_OUT = "blue";
    float scale  = 1e7;

    Vec3f point = in.vertices.front() * scale;
    Point min(point.x(), point.y());
    Point max = min;
    for (auto v : in.vertices) {
        v = v * scale;
        if (min[0] > v[0]) min[0] = v[0];
        if (min[1] > v[1]) min[1] = v[1];
        if (max[0] < v[0]) max[0] = v[0];
        if (max[1] < v[1]) max[1] = v[1];
    }
    BoundingBox bb(min, max);
    SVG svg(name, bb);
    coord_t     line_height = scale / 6;
    svg.draw_text(max + Point(0, line_height), "triangle set OUT", COLOR_OUT.c_str());
    svg.draw_text(max + Point(0, 2 * line_height), "triangle set IN", COLOR_IN.c_str());
    svg.draw_text(max + Point(0, 3 * line_height), "t .. triangle, v .. vertex", "gray");
    itsXY_to_svg(svg, in, COLOR_IN, scale / 30, scale);
    itsXY_to_svg(svg, out, COLOR_OUT, scale /100, scale);    
    svg.Close();

    // do not call visualization on test server
    CHECK(false);
}

bool exist_twin_vertices(const std::vector<stl_vertex> &vertices, float epsilon)
{
    for (size_t i = 0; i < vertices.size(); i++) {
        const stl_vertex &v0 = vertices[i];
        for (size_t j = i+1; j < vertices.size(); j++) {
            const stl_vertex &v1 = vertices[j];
            float             distance = (v0 - v1).norm();
            if (distance < epsilon) { 
                return true; 
            }
        }
    }
    return false;
}

bool exist_greater_edge(const indexed_triangle_set &its, float threshold) {
    for (auto &face : its.indices) {
        for (int i = 0; i < 3; i++) { 
            int next_i = (i == 2) ? 0 : (i + 1);
            const stl_vertex &v0     = its.vertices[face[i]];
            const stl_vertex &v1     = its.vertices[face[next_i]];
            float             length = (v0 - v1).norm();
            if (length > threshold){
                return true;
            }
        }
    }
    return false;
}

TEST_CASE("Divide triangle - once", "[SDF]") 
{ 
	indexed_triangle_set its;
    its.vertices = {
        Vec3f(1.f, 1.f, 1.f),
        Vec3f(1.f, 8.5f, 1.f),
        Vec3f(5.f, 1.f, 1.f)
    };
    its.indices = {Vec3i(0, 1, 2)};
    float threshold = 8.f;
    indexed_triangle_set its_out = ShapeDiameterFunction::subdivide(its, threshold);
    //storeXY_to_svg(its, its_out, "Divide_triangle.svg");
    CHECK(its_out.indices.size() == 2);
    CHECK(its_out.vertices.size() == 4);
    CHECK(!exist_twin_vertices(its_out.vertices, 1e-3f));
    CHECK(!exist_greater_edge(its_out, threshold));

    its_rotate_x(its, 30);
    its_rotate_y(its, 40);
    its_rotate_z(its, 50);
    its_out = ShapeDiameterFunction::subdivide(its, threshold);
    //storeXY_to_svg(its, its_out, "Divide_triangle_rotate.svg");
    CHECK(its_out.indices.size() == 2);
    CHECK(its_out.vertices.size() == 4);
    CHECK(!exist_twin_vertices(its_out.vertices, 1e-3f));
    CHECK(!exist_greater_edge(its_out, threshold));
}

// one edge in triangle is divided to 3 segments 
// --> 2 new vertices 
// --> pair caunt of vertices
// --> need decide which is close to shortest edge
// Test not duplicate point
TEST_CASE("Divide triangle - side to 3 segments", "[SDF]")
{
    indexed_triangle_set its;
    its.vertices = {
        Vec3f(1.f, 1.f, 1.f), 
        Vec3f(1.f, 8.99f, 1.f),
        Vec3f(2.f, 1.f, 1.f)
    };
    its.indices     = {Vec3i(0, 1, 2)};
    float threshold = 4.f;

    indexed_triangle_set its_out = ShapeDiameterFunction::subdivide(its, threshold);
    CHECK(its_out.indices.size() == 4);
    CHECK(its_out.vertices.size() == 6);
    CHECK(!exist_twin_vertices(its_out.vertices, 1e-3f));
    CHECK(!exist_greater_edge(its_out, threshold));

    its_rotate_x(its, 30);
    its_rotate_y(its, 40);
    its_rotate_z(its, 50);
    its_out = ShapeDiameterFunction::subdivide(its, threshold);
    CHECK(its_out.indices.size() == 4);
    CHECK(its_out.vertices.size() == 6);
    CHECK(!exist_twin_vertices(its_out.vertices, 1e-3f));
    CHECK(!exist_greater_edge(its_out, threshold));
}


TEST_CASE("Divide triangle - Two triangles with same edge", "[SDF]")
{
    indexed_triangle_set its;
    its.vertices = {
        Vec3f(1.f, 1.f, 1.f), 
        Vec3f(1.f, 8.99f, 1.f),
        Vec3f(2.f, 1.f, 1.f),
        Vec3f(2.f, 8.99f, 1.f)
    };
    its.indices  = {
        Vec3i(0, 1, 2),
        Vec3i(2, 1, 3),
    };
    float threshold = 4.f;

    indexed_triangle_set its_out = ShapeDiameterFunction::subdivide(its, threshold);
    //storeXY_to_svg(its, its_out, "Two_triangles_test.svg");
    CHECK(its_out.indices.size() == 8);
    CHECK(its_out.vertices.size() == 8);
    CHECK(!exist_twin_vertices(its_out.vertices, 1e-3f));
    CHECK(!exist_greater_edge(its_out, threshold));

    its_rotate_x(its, 170);
    its_rotate_y(its, 5);
    its_rotate_z(its, 50);
    its_out = ShapeDiameterFunction::subdivide(its, threshold);
    //storeXY_to_svg(its, its_out, "Two_triangles_rotated_test.svg");
    CHECK(its_out.indices.size() == 8);
    CHECK(its_out.vertices.size() == 8);
    CHECK(!exist_twin_vertices(its_out.vertices, 1e-3f));
    CHECK(!exist_greater_edge(its_out, threshold));
}

TEST_CASE("Divide triangle - Big triangle inside others", "[SDF]")
{
    indexed_triangle_set its;
    its.vertices = {
        Vec3f(1.f, 1.f, 1.f), 
        Vec3f(1.f, 18.f, 1.f),
        Vec3f(14.f, 1.f, 1.f), 

        Vec3f(1.f, 0.f, 1.f), 
        Vec3f(0.f, 1.f, 1.f), 
        Vec3f(0.f, 19.f, 1.f), 
        Vec3f(1.f, 19.f, 1.f), 
        Vec3f(15.f, 1.f, 1.f), 
        Vec3f(15.f, 0.f, 1.f)
    };
    its.indices  = {
        Vec3i(0, 1, 2), // Main triangle
        // surround
        Vec3i(0, 3, 4),
        Vec3i(0, 4, 1),
        Vec3i(1, 4, 5),
        Vec3i(1, 5, 6),
        Vec3i(1, 6, 2),
        Vec3i(2, 6, 7),
        Vec3i(2, 7, 8),
        Vec3i(2, 8, 3),
        Vec3i(0, 2, 3),
    };
    float                threshold = 4.f;
    indexed_triangle_set its_out = ShapeDiameterFunction::subdivide(its, threshold);
    storeXY_to_svg(its, its_out, "Big_triangle_inside.svg");
    CHECK(!exist_twin_vertices(its_out.vertices, 1e-3f));
    CHECK(!exist_greater_edge(its_out, threshold));
    CHECK(its_out.indices.size() <= 114);
    CHECK(its_out.vertices.size() <= 67);    
}