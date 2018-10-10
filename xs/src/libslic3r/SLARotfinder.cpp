#include <SLARotfinder.hpp>
#include <libnest2d/optimizers/genetic.hpp>
#include "SLABoilerPlate.hpp"
#include "Model.hpp"

namespace Slic3r {
namespace sla {

EigenMesh3D to_eigenmesh(const ModelObject& modelobj) {
    TriangleMesh tmesh = modelobj.raw_mesh();

    const stl_file& stl = tmesh.stl;

    EigenMesh3D outmesh;
    auto& V = outmesh.V;
    auto& F = outmesh.F;

    V.resize(3*stl.stats.number_of_facets, 3);
    F.resize(stl.stats.number_of_facets, 3);
    for (unsigned int i=0; i<stl.stats.number_of_facets; ++i) {
        const stl_facet* facet = stl.facet_start+i;
        V(3*i+0, 0) = facet->vertex[0](0); V(3*i+0, 1) =
                facet->vertex[0](1); V(3*i+0, 2) = facet->vertex[0](2);
        V(3*i+1, 0) = facet->vertex[1](0); V(3*i+1, 1) =
                facet->vertex[1](1); V(3*i+1, 2) = facet->vertex[1](2);
        V(3*i+2, 0) = facet->vertex[2](0); V(3*i+2, 1) =
                facet->vertex[2](1); V(3*i+2, 2) = facet->vertex[2](2);

        F(i, 0) = 3*i+0;
        F(i, 1) = 3*i+1;
        F(i, 2) = 3*i+2;
    }

    return outmesh;
}

std::array<double, 3> find_best_rotation(const ModelObject &modelobj,
                                         std::function<bool ()> stopcond,
                                         unsigned max_tries)
{
    using libnest2d::opt::Method;
    using libnest2d::opt::bound;
    using libnest2d::opt::Optimizer;
    using libnest2d::opt::TOptimizer;
    using libnest2d::opt::StopCriteria;
    using Quaternion = Eigen::Quaternion<double>;

    std::array<double, 3> rot;

    EigenMesh3D emesh = to_eigenmesh(modelobj);

    auto objfunc = [&emesh](double polar, double azimuth) {
        EigenMesh3D m = emesh;
        Vec3d nn(std::cos(azimuth) * std::sin(polar),
                 std::sin(azimuth) * std::sin(polar),
                 std::cos(polar));

        auto quatern = Quaternion::FromTwoVectors(Vec3d{0, 0, 1}, nn);
        for(auto& p : m.points) p = quatern * p;

        return 0;
    };

    TOptimizer<Method::G_GENETIC> solver;
    auto result = solver.optimize_max(objfunc,
                                      libnest2d::opt::initvals(0.0, 0.0),
                                      bound(0.0, PI), bound(0.0, 2.0*PI));


    return rot;
}

}
}
