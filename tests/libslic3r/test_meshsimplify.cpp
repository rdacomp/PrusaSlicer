#include <catch2/catch.hpp>
#include <test_utils.hpp>

#include <libslic3r/SimplifyMesh.hpp>

TEST_CASE("Simplifying an empty mesh should result in an empty mesh", "[mesh_simplify]") {
    indexed_triangle_set mesh;

    Slic3r::simplify_mesh(mesh);

    REQUIRE(mesh.vertices.empty());
    REQUIRE(mesh.indices.empty());
}


// TODO: find or generate models with redundant faces and test the alg
TEST_CASE("Simplified mesh should have less primitives and have about the same volume", "[mesh_simplify]") {
    using namespace Slic3r;
    TriangleMesh msh = load_model("frog_legs.obj");
    msh.require_shared_vertices();

    indexed_triangle_set &its = msh.its;

    size_t vcount = its.vertices.size(), fcount = its.indices.size();
    float vol = its_volume(its);
    simplify_mesh(its);

    REQUIRE(its.vertices.size() <= vcount);
    REQUIRE(its.indices.size() <= fcount);

    REQUIRE(std::abs(its_volume(its) - vol) < vol * 0.01);

#ifndef NDEBUG
    its_write_obj(its, "zaba_simplified.obj");
    std::cout << "Vertices removed: " << vcount - its.vertices.size() << "\n";
    std::cout << "Faces removed: " << fcount - its.indices.size() << std::endl;
#endif
}

