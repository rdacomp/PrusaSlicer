#ifndef slic3r_FillAdaptive_hpp_
#define slic3r_FillAdaptive_hpp_

#include "../AABBTreeIndirect.hpp"

#include "FillBase.hpp"

namespace Slic3r {

class PrintObject;
class TriangleMesh;

namespace FillAdaptive_Internal
{
    struct CubeProperties
    {
        double edge_length;     // Lenght of edge of a cube
        double height;          // Height of rotated cube (standing on the corner)
        double diagonal_length; // Length of diagonal of a cube a face
        double line_z_distance; // Defines maximal distance from a center of a cube on Z axis on which lines will be created
        double line_xy_distance;// Defines maximal distance from a center of a cube on X and Y axis on which lines will be created
    };

    struct Cube;
    struct Octree;
    // To keep the definition of Octree opaque, we have to define a custom deleter.
    struct OctreeDeleter {
        void operator()(Octree *p);
    };
    using OctreePtr = std::unique_ptr<Octree, OctreeDeleter>;
}; // namespace FillAdaptive_Internal

//
// Some of the algorithms used by class FillAdaptive were inspired by
// Cura Engine's class SubDivCube
// https://github.com/Ultimaker/CuraEngine/blob/master/src/infill/SubDivCube.h
//
class FillAdaptive : public Fill
{
public:
    virtual ~FillAdaptive() {}

protected:
    virtual Fill* clone() const { return new FillAdaptive(*this); };
	virtual void _fill_surface_single(
	    const FillParams                &params,
	    unsigned int                     thickness_layers,
	    const std::pair<float, Point>   &direction,
	    ExPolygon                       &expolygon,
	    Polylines                       &polylines_out);

	virtual bool no_sort() const { return true; }

    void generate_infill(const FillParams &             params,
                         unsigned int                   thickness_layers,
                         const std::pair<float, Point> &direction,
                         ExPolygon &                    expolygon,
                         Polylines &                    polylines_out,
                         FillAdaptive_Internal::Octree *octree);

public:
    static FillAdaptive_Internal::OctreePtr build_octree(
        // Mesh is rotated to the coordinate system of the octree.
        const indexed_triangle_set  &triangle_mesh, 
        // Up vector of the mesh rotated to the coordinate system of the octree.
        const Vec3d                 &up_vector, 
        coordf_t                     line_spacing, 
        // If true, octree is densified below internal overhangs only.
        bool                         support_overhangs_only);
};

class FillSupportCubic : public FillAdaptive
{
public:
    virtual ~FillSupportCubic() = default;

protected:
    virtual Fill* clone() const { return new FillSupportCubic(*this); };

    virtual void _fill_surface_single(
        const FillParams                &params,
        unsigned int                     thickness_layers,
        const std::pair<float, Point>   &direction,
        ExPolygon                       &expolygon,
        Polylines                       &polylines_out);
};

// Calculate line spacing for
// 1) adaptive cubic infill
// 2) adaptive internal support cubic infill
// Returns zero for a particular infill type if no such infill is to be generated.
std::pair<double, double> adaptive_fill_line_spacing(const PrintObject &print_object);

} // namespace Slic3r

#endif // slic3r_FillAdaptive_hpp_
