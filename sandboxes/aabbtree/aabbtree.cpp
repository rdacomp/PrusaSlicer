#include <iostream>
#include <fstream>
#include <string>

#include <libslic3r/libslic3r.h>
#include <libslic3r/TriangleMesh.hpp>
#include <libslic3r/SLA/EigenMesh3D.hpp>
#include <libnest2d/tools/benchmark.h>

const std::string USAGE_STR = {
    "Usage: aabbtree stlfilename.stl"
};

int main(const int argc, const char *argv[]) {
    using namespace Slic3r;
    using std::cout; using std::endl;

    if(argc < 2) {
        cout << USAGE_STR << endl;
        return EXIT_SUCCESS;
    }

    Benchmark bench;

    bench.start();
    TriangleMesh mesh;
    mesh.ReadSTLFile(argv[1]);
    mesh.require_shared_vertices();
    bench.stop();

    std::cout << "Mesh load time: " << bench.getElapsedSec() << std::endl;

    bench.start();
    sla::EigenMesh3D emesh(mesh);
    bench.stop();

    std::cout << "AABB construction time: " << bench.getElapsedSec() << std::endl;
    std::cout << "Ground level: " << emesh.ground_level() << std::endl;

    return EXIT_SUCCESS;
}
