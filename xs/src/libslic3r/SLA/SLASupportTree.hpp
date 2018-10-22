#ifndef SLASUPPORTTREE_HPP
#define SLASUPPORTTREE_HPP

#include <vector>
#include <array>
#include <cstdint>
#include <memory>
#include <Eigen/Geometry>

namespace Slic3r {

// Needed types from Point.hpp
typedef int32_t coord_t;
typedef Eigen::Matrix<double,   3, 1, Eigen::DontAlign> Vec3d;
typedef Eigen::Matrix<float,    3, 1, Eigen::DontAlign> Vec3f;
typedef Eigen::Matrix<coord_t,  3, 1, Eigen::DontAlign> Vec3crd;
typedef std::vector<Vec3d>                              Pointf3s;
typedef std::vector<Vec3crd>                            Points3;

class TriangleMesh;
class Model;
class ModelInstance;
class ExPolygon;

using SliceLayer = std::vector<ExPolygon>;
using SlicedSupports = std::vector<SliceLayer>;

namespace sla {

struct SupportConfig {
    // Radius in mm of the pointing side of the head.
    double head_front_radius_mm = 0.2;

    // Radius of the back side of the 3d arrow.
    double head_back_radius_mm = 0.5;

    // Width in mm from the back sphere center to the front sphere center.
    double head_width_mm = 1.0;

    // Radius in mm of the support pillars.
    // Warning: this value will be at most 65% of head_back_radius_mm
    double pillar_radius_mm = 0.8;

    // Radius in mm of the pillar base.
    double base_radius_mm = 1.0;

    // The height of the pillar base cone in mm.
    double base_height_mm = 1.0;

    // The default angle for connecting support sticks and junctions.
    double tilt = M_PI/4;

    double junction_distance = 1.5;
};

/// A Control structure for the support calculation. The algorithm can query a
/// a start (restart), pause or stop (cancel) command through the nextcmd
/// function. It can also report its status through the statuscb function.
struct Controller {
    enum class Cmd { START_RESUME, PAUSE, STOP, SYNCH };

    std::function<void(unsigned, const std::string&)> statuscb =
            [](unsigned, const std::string&){};

    std::function<Cmd(bool)> nextcmd = [](bool){ return Cmd::START_RESUME; };
};

class SLASupportTree {
public:
    class Impl;
private:
    std::unique_ptr<Impl> m_impl;
public:

    SLASupportTree();
    SLASupportTree(const SLASupportTree&);
    SLASupportTree& operator=(const SLASupportTree&);

    /// Generate the 3D supports for a model intended for SLA print.
    bool generate(const Model& model, const SupportConfig& cfg = {},
                  const Controller& ctl = {});

    Impl& get() { return *m_impl; }

    ~SLASupportTree();
};

/// Slice up the support tree for printing
SlicedSupports slice(const SLASupportTree& stree, const Controller& ctl);

/* ************************************************************************** */
/* TODO: May not be needed:                                                   */
/* ************************************************************************** */

void create_head(TriangleMesh&, double r1_mm, double r2_mm, double width_mm);

/// Add support volumes to the model directly
void add_sla_supports(Model& model, const SupportConfig& cfg = {},
                      const Controller& ctl = {});

}

}

#endif // SLASUPPORTTREE_HPP
