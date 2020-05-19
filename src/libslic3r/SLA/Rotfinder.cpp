#include <limits>
#include <exception>

#include <libnest2d/optimizers/nlopt/genetic.hpp>
#include <libslic3r/SLA/Common.hpp>
#include <libslic3r/SLA/Rotfinder.hpp>
#include <libslic3r/SLA/SupportTree.hpp>
#include "Model.hpp"

namespace Slic3r {
namespace sla {

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

    static const unsigned MAX_TRIES = 100000;

    // return value
    std::array<double, 3> rot;

    // We will use only one instance of this converted mesh to examine different
    // rotations
    EigenMesh3D emesh(modelobj.raw_mesh());

    // For current iteration number
    unsigned status = 0;

    // The maximum number of iterations
    auto max_tries = unsigned(accuracy * MAX_TRIES);

    // call status callback with zero, because we are at the start
    statuscb(status);

    // So this is the object function which is called by the solver many times
    // It has to yield a single value representing the current score. We will
    // call the status callback in each iteration but the actual value may be
    // the same for subsequent iterations (status goes from 0 to 100 but
    // iterations can be many more)
    auto objfunc = [&emesh, &status, &statuscb, &stopcond, max_tries]
            (float rx, float ry, float rz)
    {
        EigenMesh3D& m = emesh;

        // prepare the rotation transformation
        Transform3f rt = Transform3f::Identity();

        rt.rotate(Eigen::AngleAxisf(rz, Vec3f::UnitZ()));
        rt.rotate(Eigen::AngleAxisf(ry, Vec3f::UnitY()));
        rt.rotate(Eigen::AngleAxisf(rx, Vec3f::UnitX()));

        double score = 0;

        // For all triangles we calculate the normal and sum up the dot product
        // (a scalar indicating how much are two vectors aligned) with each axis
        // this will result in a value that is greater if a normal is aligned
        // with all axes. If the normal is aligned than the triangle itself is
        // orthogonal to the axes and that is good for print quality.

        // TODO: some applications optimize for minimum z-axis cross section
        // area. The current function is only an example of how to optimize.

        // Later we can add more criteria like the number of overhangs, etc...
        for(int i = 0; i < m.F().rows(); i++) {
            auto idx = m.F().row(i);

            Vec3f p1 = m.V().row(idx(0));
            Vec3f p2 = m.V().row(idx(1));
            Vec3f p3 = m.V().row(idx(2));

            Eigen::Vector3f U = p2 - p1;
            Eigen::Vector3f V = p3 - p1;

            // So this is the normal
            auto n = U.cross(V).normalized();

            // rotate the normal with the current rotation given by the solver
            n = rt * n;

            // We should score against the alignment with the reference planes
            score += std::abs(n.dot(Vec3f::UnitX()));
            score += std::abs(n.dot(Vec3f::UnitY()));
            score += std::abs(n.dot(Vec3f::UnitZ()));
        }

        // report status
        if(!stopcond()) statuscb( unsigned(++status * 100.0/max_tries) );

        return score;
    };

    // Firing up the genetic optimizer. For now it uses the nlopt library.
    StopCriteria stc;
    stc.max_iterations = max_tries;
    stc.relative_score_difference = 1e-3;
    stc.stop_condition = stopcond;      // stop when stopcond returns true
    TOptimizer<Method::G_GENETIC> solver(stc);

    // We are searching rotations around the three axes x, y, z. Thus the
    // problem becomes a 3 dimensional optimization task.
    // We can specify the bounds for a dimension in the following way:
    auto b = bound(float(-PI/2), float(PI/2));

    // Now we start the optimization process with initial angles (0, 0, 0)
    auto result = solver.optimize_max(objfunc,
                                      libnest2d::opt::initvals(0.f, 0.f, 0.f),
                                      b, b, b);

    // Save the result and fck off
    rot[0] = std::get<0>(result.optimum);
    rot[1] = std::get<1>(result.optimum);
    rot[2] = std::get<2>(result.optimum);

    return rot;
}

}
}
