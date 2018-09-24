#ifndef SLASUPPORTTREE_HPP
#define SLASUPPORTTREE_HPP

#include <vector>
#include <array>
#include <cstdint>
#include <Eigen/Geometry>

namespace Slic3r {

// Needed types from Point.hpp
typedef int32_t coord_t;
typedef Eigen::Matrix<double,   3, 1, Eigen::DontAlign> Vec3d;
typedef Eigen::Matrix<coord_t,  3, 1, Eigen::DontAlign> Vec3crd;
typedef std::vector<Vec3d>                              Pointf3s;
typedef std::vector<Vec3crd>                            Points3;

class TriangleMesh;
class Model;

namespace sla {

struct SupportConfig {
    // Radius in mm of the pointing side of the head.
    double head_front_radius_mm = 0.3;

    // Radius of the back side of the 3d arrow.
    double head_back_radius_mm = 1.0;

    // Width in mm from the back sphere center to the front sphere center.
    double head_width_mm = 2.0;

    // Radius in mm of the support pillars.
    double pillar_radius_mm = 1.0;

    // Radius in mm of the pillar base.
    double base_radius_mm = 3.0;

    // The height of the pillar base cone in mm.
    double base_height_mm = 0.5;
};

/// Generate the 3D support rods for a model intended for SLA print.
void create_support_tree(const Model& model,
                         TriangleMesh& output,
                         const SupportConfig& cfg);

}
}

#endif // SLASUPPORTTREE_HPP
