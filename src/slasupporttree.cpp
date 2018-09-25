#include <iostream>
#include <string>

#include <libslic3r.h>
#include "TriangleMesh.hpp"
#include "Model.hpp"
#include "callback.hpp"
#include "SLASupportTree.hpp"
#include "benchmark.h"

const std::string USAGE_STR = {
    "Usage: slasupporttree stlfilename.stl"
};

void confess_at(const char * /*file*/,
                int /*line*/,
                const char * /*func*/,
                const char * /*pat*/,
                ...) {}

namespace Slic3r {

void PerlCallback::deregister_callback() {}
}

int main(const int argc, const char *argv[]) {
    using namespace Slic3r;
    using std::cout; using std::endl;

    if(argc < 2) {
        cout << USAGE_STR << endl;
        return EXIT_SUCCESS;
    }

    TriangleMesh tmesh;
    Benchmark bench;

    tmesh.ReadSTLFile(argv[1]);
    tmesh.align_to_origin();

    TriangleMesh result;
    Model model;

    bench.start();
    sla::create_support_tree(model, result, {});
    bench.stop();

    cout << "Support tree creation time: " << std::setprecision(10)
         << bench.getElapsedSec() << " seconds." << endl;

    result.write_ascii("out.stl");

    return EXIT_SUCCESS;
}
