#include <limits>
#include <exception>

#include <libnest2d/optimizers/nlopt/genetic.hpp>
#include <libnest2d/optimizers/nlopt/direct.hpp>
#include <libnest2d/optimizers/nlopt/subplex.hpp>
#include <libnest2d/tools/benchmark.h>

#include "SLABoilerPlate.hpp"
#include "SLARotfinder.hpp"
#include "SLASupportTree.hpp"
#include "Model.hpp"

namespace Slic3r {

CrossSection find_max_cross_section_gen(TriangleMesh &mesh)
{
    namespace opt = libnest2d::opt;

    auto bb = mesh.bounding_box();
    Vec3d bsize = bb.size();

    opt::StopCriteria stc;
    stc.max_iterations = 100;
    stc.relative_score_difference = 1e-3;
    stc.stop_score = bsize(X) * bsize(Y);
    opt::GeneticOptimizer solver(stc);

    double sf = std::pow(SCALING_FACTOR, 2);

    TriangleMeshSlicer slicer(&mesh);

    auto result = solver.optimize_max(
        [&slicer, sf](float h) {
            std::vector<Polygons> slices;

            try { slicer.slice({h}, &slices, [](){}); } catch(...) {}

            if(slices.empty()) return std::nan("");

            auto& sl = slices.front();

            return std::accumulate(sl.begin(), sl.end(), 0.0,
                                   [sf](double a, const Polygon &p) {
                return a + p.area() * sf;
            });
        },
        opt::initvals(float(bb.min(Z) + bsize(Z)/2)),
        opt::bound(float(bb.min(Z)), float(bb.max(Z)))
    );

    CrossSection cs;
    cs.area = result.score;
    cs.slice_h = std::get<0>(result.optimum);

    return cs;
}

CrossSection find_max_cross_section_direct(TriangleMesh &mesh)
{
    namespace opt = libnest2d::opt;

    auto bb = mesh.bounding_box();
    Vec3d bsize = bb.size();

    opt::StopCriteria stc;
    stc.max_iterations = 100;
    stc.relative_score_difference = 1e-3;
    stc.stop_score = bsize(X) * bsize(Y);
    opt::DirectOptimizer solver(stc);

    double sf = std::pow(SCALING_FACTOR, 2);

    TriangleMeshSlicer slicer(&mesh);

    auto result = solver.optimize_max(
        [&slicer, sf](float h) {
            std::vector<Polygons> slices;

            try { slicer.slice({h}, &slices, [](){}); } catch(...) {}

            if(slices.empty()) return std::nan("");

            auto& sl = slices.front();

            return std::accumulate(sl.begin(), sl.end(), 0.0,
                                   [sf](double a, const Polygon &p) {
                return a + p.area() * sf;
            });
        },
        opt::initvals(float(bb.min(Z) + bsize(Z)/2)),
        opt::bound(float(bb.min(Z)), float(bb.max(Z)))
    );

    CrossSection cs;
    cs.area = result.score;
    cs.slice_h = std::get<0>(result.optimum);

    return cs;
}

CrossSection find_max_cross_section_subplx(TriangleMesh &mesh,
                                           float presample_dist = 10.f,
                                           unsigned localtries = 10)
{
    namespace opt = libnest2d::opt;

    auto bb = mesh.bounding_box();
    Vec3d bsize = bb.size();

    double sf = std::pow(SCALING_FACTOR, 2);

    // pre-sample with 1cm grid (max model height is 15 cm)
    float halfstride = presample_dist / 2.f;

    CrossSection cs; cs.area = 0.0;

    opt::StopCriteria stc;
    stc.max_iterations = localtries;
    stc.relative_score_difference = 1e-3;
    stc.stop_score = bsize(X) * bsize(Y);
    opt::SubplexOptimizer solver(stc);

    TriangleMeshSlicer slicer(&mesh);

    for(float h = float(bb.min(Z)) + halfstride;
        h < float(bb.max(Z));
        h += presample_dist)
    {
        auto result = solver.optimize_max(
            [&slicer, sf](float h) {
                std::vector<Polygons> slices;

                try { slicer.slice({h}, &slices, [](){}); } catch(...) {}

                if(slices.empty()) return std::nan("");

                auto& sl = slices.front();

                return std::accumulate(sl.begin(), sl.end(), 0.0,
                                       [sf](double a, const Polygon &p) {
                    return a + p.area() * sf;
                });
            },
            opt::initvals(h),
            opt::bound(h - halfstride, h + halfstride)
        );

        if(result.score > cs.area) { cs.area = result.score; cs.slice_h = h; }
    };

    return cs;
}

namespace sla {

std::array<double, 2> find_best_rotation_cr(const ModelObject& modelobj,
                                         float accuracy,
                                         std::function<void(unsigned)> statuscb,
                                         std::function<bool()> stopcond)
{
    namespace opt = libnest2d::opt;
    static const unsigned MAX_TRIES = 100000;

    // return value
    std::array<double, 2> rot;

    TriangleMesh mesh = modelobj.raw_mesh();

    if(!modelobj.instances.empty()) {
        mesh.scale(modelobj.instances.front()->get_scaling_factor());
    }

    // The maximum number of iterations
    auto max_tries = unsigned(accuracy * MAX_TRIES);

    double status = 0.0;
    double statusinc = 100.0 / max_tries;

    statuscb(0);

    // Firing up the genetic optimizer. For now it uses the nlopt library.
    opt::StopCriteria stc;
    stc.max_iterations = max_tries;
    stc.relative_score_difference = 1e-3;
    stc.stop_condition = stopcond;      // stop when stopcond returns true
    opt::GeneticOptimizer solver(stc);

    // We are searching rotations around the three axes x, y, z. Thus the
    // problem becomes a 3 dimensional optimization task.
    // We can specify the bounds for a dimension in the following way:
    auto b = opt::bound(-PI/2, PI/2);
    auto iv = opt::initvals(0.0, 0.0);

    std::vector<double> areas_subpl;
    std::vector<double> dur_subplx;

    std::vector<double> areas_gen;
    std::vector<double> dur_gen;


    auto result = solver.optimize_min(
        [&mesh, &status, statusinc, statuscb, &areas_gen, &areas_subpl, &dur_gen, &dur_subplx](double rx, double ry)
    {
        TriangleMesh m = mesh;
        m.rotate_x(float(rx)); m.rotate_y(float(ry));

        auto bb = m.bounding_box();
        Vec3d bsize = bb.size();

        double hnorm = 150; // mm;
        double areanorm = 120.96 * 68.040;

        Benchmark bench;

        bench.start();
        CrossSection cs1 = find_max_cross_section_subplx(m);
//        CrossSection cs1 = find_max_cross_section_direct(m);
        bench.stop();

        areas_subpl.emplace_back(cs1.area);
        dur_subplx.emplace_back(bench.getElapsedSec());

        std::cout << "subplex value: " << cs1.area << " duration: " << bench.getElapsedSec() << std::endl;

        bench.start();
//        CrossSection cs2 = find_max_cross_section_gen(m);
        CrossSection cs2 = find_max_cross_section_direct(m);
        bench.stop();

        areas_gen.emplace_back(cs2.area);
        dur_gen.emplace_back(bench.getElapsedSec());

        std::cout << "genetic value: " << cs2.area << " duration: " << bench.getElapsedSec() << std::endl;


        double h = bsize(Z) / hnorm;
        double a = cs1.area / areanorm;

        double st = status + statusinc;
        auto ist = unsigned(std::round(st));

        if(ist > std::round(status)) { statuscb(ist); }
        status = st;

        return std::max(cs1.area, cs2.area);
//        return 0.8 * a + 0.2 * h;
//        return cs1.area;

    }, iv, b, b);

    double avg_subplx = std::accumulate(dur_subplx.begin(), dur_subplx.end(), 0.0);
    double avg_gen    = std::accumulate(dur_gen.begin(), dur_gen.end(), 0.0);

    size_t gen_wins = 0;

    for(size_t i = 0; i < areas_gen.size(); i++) {
        if(areas_gen[i] > areas_subpl[i]) gen_wins++;
    }

    std::cout << "avg subplex = " << avg_subplx << " wins = " << areas_gen.size() - gen_wins << std::endl;
    std::cout << "avg gen = " << avg_gen << " wins = " << gen_wins << std::endl;

    // Save the result and fck off
    rot[0] = std::get<0>(result.optimum);
    rot[1] = std::get<1>(result.optimum);

    return rot;
}

std::array<double, 2> find_best_rotation(const ModelObject& modelobj,
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
    std::array<double, 2> rot;

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
    auto objfunc = [&emesh, &status, &statuscb, max_tries]
            (double rx, double ry/*, double rz*/)
    {
        EigenMesh3D& m = emesh;

        // prepare the rotation transformation
        Transform3d rt = Transform3d::Identity();

//        rt.rotate(Eigen::AngleAxisd(rz, Vec3d::UnitZ()));
        rt.rotate(Eigen::AngleAxisd(ry, Vec3d::UnitY()));
        rt.rotate(Eigen::AngleAxisd(rx, Vec3d::UnitX()));

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

            Vec3d p1 = m.V().row(idx(0));
            Vec3d p2 = m.V().row(idx(1));
            Vec3d p3 = m.V().row(idx(2));

            Eigen::Vector3d U = p2 - p1;
            Eigen::Vector3d V = p3 - p1;

            // So this is the normal
            auto n = U.cross(V).normalized();

            // rotate the normal with the current rotation given by the solver
            n = rt * n;

            // We should score against the alignment with the reference planes
            score += std::abs(n.dot(Vec3d::UnitX()));
            score += std::abs(n.dot(Vec3d::UnitY()));
            score += std::abs(n.dot(Vec3d::UnitZ()));
        }

        // report status
        statuscb( unsigned(++status * 100.0/max_tries) );

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
    auto b = bound(-PI/2, PI/2);

    // Now we start the optimization process with initial angles (0, 0, 0)
    auto result = solver.optimize_max(objfunc,
                                      libnest2d::opt::initvals(0.0, 0.0),
                                      b, b);

    // Save the result and fck off
    rot[0] = std::get<0>(result.optimum);
    rot[1] = std::get<1>(result.optimum);
//    rot[2] = std::get<2>(result.optimum);

    return rot;
}

}
}
