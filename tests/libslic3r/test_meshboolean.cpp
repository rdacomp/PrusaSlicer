#include <catch2/catch.hpp>
#include <test_utils.hpp>

#include <libslic3r/TriangleMesh.hpp>
#include <libslic3r/MeshBoolean.hpp>
#include <libslic3r/SimplifyMesh.hpp>

using namespace Slic3r;

TEST_CASE("CGAL and TriangleMesh conversions", "[MeshBoolean]") {
    TriangleMesh sphere = make_sphere(1.);
    
    auto cgalmesh_ptr = MeshBoolean::cgal::triangle_mesh_to_cgal(sphere);
    
    REQUIRE(cgalmesh_ptr);
    REQUIRE(! MeshBoolean::cgal::does_self_intersect(*cgalmesh_ptr));
    
    TriangleMesh M = MeshBoolean::cgal::cgal_to_triangle_mesh(*cgalmesh_ptr);
    
    REQUIRE(M.its.vertices.size() == sphere.its.vertices.size());
    REQUIRE(M.its.indices.size() == sphere.its.indices.size());
    
    REQUIRE(M.volume() == Approx(sphere.volume()));
    
    REQUIRE(! MeshBoolean::cgal::does_self_intersect(M));
}

TEST_CASE("Self boolean for two spheres", "[MeshBoolean], [NotWorking]")
{
    TriangleMesh s1 = make_sphere(1.);
    TriangleMesh s2 = make_sphere(1.);
    
    s1.translate(-0.25, 0., 0.);
    s2.translate(0.25, 0., 0.);
    
    TriangleMesh twospheres(s1);
    twospheres.merge(s2);
    twospheres.require_shared_vertices();
    
    REQUIRE(MeshBoolean::cgal::does_self_intersect(twospheres));
    
    try {
        MeshBoolean::self_union(twospheres);
        twospheres.WriteOBJFile("twospheres_igl.obj");
    } catch (...) {
        REQUIRE(false);
    }
    
    REQUIRE(! MeshBoolean::cgal::does_self_intersect(twospheres));
}
