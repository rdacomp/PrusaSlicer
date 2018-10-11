#include <limits>
#include <exception>

#include <SLARotfinder.hpp>
#include <libnest2d/optimizers/genetic.hpp>
#include "SLABoilerPlate.hpp"
#include "Model.hpp"

namespace Slic3r {
namespace sla {

EigenMesh3D to_eigenmesh(TriangleMesh& tmesh) {

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

inline EigenMesh3D to_eigenmesh(const ModelObject& modelobj) {
    return to_eigenmesh(modelobj.raw_mesh());
}

std::array<double, 3> find_best_rotation(const ModelObject& modelobj,
                                         float accuracy,
                                         std::function<void(unsigned)> statuscb,
                                         std::function<bool()> stopcond)
{
    using libnest2d::opt::Method;
    using libnest2d::opt::bound;
    using libnest2d::opt::Optimizer;
    using libnest2d::opt::TOptimizer;
    using libnest2d::opt::StopCriteria;
    using Quaternion = Eigen::Quaternion<double>;

    static const unsigned MAX_TRIES = 100000;

    std::array<double, 3> rot;

    EigenMesh3D emesh = to_eigenmesh(modelobj);

    unsigned status = 0;
    auto max_tries = unsigned(accuracy * MAX_TRIES);

    statuscb(status);

    auto objfunc = [&emesh, &status, &statuscb, max_tries]
            (double rx, double ry, double rz)
    {
        EigenMesh3D& m = emesh;

        Transform3d rt = Transform3d::Identity();

        rt.rotate(Eigen::AngleAxisd(rz, Vec3d::UnitZ()));
        rt.rotate(Eigen::AngleAxisd(ry, Vec3d::UnitY()));
        rt.rotate(Eigen::AngleAxisd(rx, Vec3d::UnitX()));

        double score = 0;
        for(int i = 0; i < m.F.rows(); i++) {
            auto idx = m.F.row(i);

            Vec3d p1 = m.V.row(idx(0));
            Vec3d p2 = m.V.row(idx(1));
            Vec3d p3 = m.V.row(idx(2));

            Eigen::Vector3d U = p2 - p1;
            Eigen::Vector3d V = p3 - p1;

            // So this is the normal
            auto n = U.cross(V).normalized();
            n =  rt * n;

            // We should measure against the three reference planes
            score += std::abs(n.dot(Vec3d::UnitX()));
            score += std::abs(n.dot(Vec3d::UnitY()));
            score += std::abs(n.dot(Vec3d::UnitZ()));
        }

        statuscb( unsigned(++status * 100.0/max_tries) );

        return score;
    };

    StopCriteria stc;
    stc.max_iterations = max_tries;
    stc.relative_score_difference = 1e-3;
    stc.stop_condition = stopcond;
    TOptimizer<Method::G_GENETIC> solver(stc);

    auto b = bound(-PI/2, PI/2);

    auto result = solver.optimize_max(objfunc,
                                      libnest2d::opt::initvals(0.0, 0.0, 0.0),
                                      b, b, b);

    rot[0] = std::get<0>(result.optimum);
    rot[1] = std::get<1>(result.optimum);
    rot[2] = std::get<2>(result.optimum);

    return rot;
}

}
}
