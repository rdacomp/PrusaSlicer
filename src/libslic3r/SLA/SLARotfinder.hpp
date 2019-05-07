#ifndef SLAROTFINDER_HPP
#define SLAROTFINDER_HPP

#include <functional>
#include <array>
#include <vector>

namespace Slic3r {

class ModelObject;
class TriangleMesh;
class ExPolygon;

struct CrossSection {
    double area;
    float slice_h;
};

CrossSection find_max_cross_section_subplx(const TriangleMesh &,
                                           float presample_dist = 10.f,
                                           unsigned localtries = 10);
CrossSection find_max_cross_section_gen(const TriangleMesh &);

namespace sla {

/**
  * The function should find the best rotation for SLA upside down printing.
  *
  * @param modelobj The model object representing the 3d mesh.
  * @param accuracy The optimization accuracy from 0.0f to 1.0f. Currently,
  * the nlopt genetic optimizer is used and the number of iterations is
  * accuracy * 100000. This can change in the future.
  * @param statuscb A status indicator callback called with the unsigned
  * argument spanning from 0 to 100. May not reach 100 if the optimization finds
  * an optimum before max iterations are reached.
  * @param stopcond A function that if returns true, the search process will be
  * terminated and the best solution found will be returned.
  *
  * @return Returns the rotations around each axis (x, y, z)
  */
std::array<double, 2> find_best_rotation(
        const ModelObject& modelobj,
        float accuracy = 1.0f,
        std::function<void(unsigned)> statuscb = [] (unsigned) {},
        std::function<bool()> stopcond = [] () { return false; }
        );

std::array<double, 2> find_best_rotation_cr(
        const ModelObject& modelobj,
        float accuracy = 1.0f,
        std::function<void(unsigned)> statuscb = [] (unsigned) {},
        std::function<bool()> stopcond = [] () { return false; }
        );

}
}

#endif // SLAROTFINDER_HPP
