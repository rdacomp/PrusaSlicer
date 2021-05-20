#ifndef slic3r_ShapeDiameterFunction_hpp_
#define slic3r_ShapeDiameterFunction_hpp_

#include "Point.hpp"
#include "Model.hpp"
#include "AABBTreeIndirect.hpp"

namespace Slic3r {

class ShapeDiameterFunction
{
public:
    /// <summary>
    /// Rays into z direction.
    /// For calculation SDF function with multi rays.
    /// Weighted directions.
    /// </summary>
    struct Direction
    {
        Vec3f dir;
        float weight;
    };
    using Directions = std::vector<Direction>;

    /// <summary>
    /// Only static functions
    /// </summary>
    ShapeDiameterFunction() = delete;

    /// <summary>
    /// Calculate width in point given by weighted average width
    /// </summary>
    /// <param name="point">Surface point of model</param>
    /// <param name="normal">Normal in surface point</param>
    /// <param name="dirs">Direction to cast rays</param>
    /// <param name="its">Needed by tree function</param>
    /// <param name="tree">AABB tree to fast detect first intersection</param>
    /// <returns>Width of model for surface point</returns>
    static float calc_width(const Vec3f &                   point,
                            const Vec3f &                   normal,
                            const Directions &              dirs,
                            const indexed_triangle_set &    its,
                            const AABBTreeIndirect::Tree3f &tree);

    /// <summary>
    /// Concurrent calculation of width for each vertex
    /// </summary>
    /// <param name="dirs">Direction to cast rays</param>
    /// <param name="its">Needed by tree function</param>
    /// <param name="normals">Normal direction in vertex - same size as its vertices</param>
    /// <param name="tree">AABB tree to fast detect first intersection</param>
    /// <returns></returns>
    static std::vector<float> calc_widths(
                            const Directions &              dirs,
                            const indexed_triangle_set &    its,
                            const std::vector<Vec3f>& normals,
                            const AABBTreeIndirect::Tree3f &tree);

    /// <summary>
    /// Create points on unit sphere surface. with weight by z value
    /// </summary>
    /// <param name="angle"></param>
    /// <param name="count_samples"></param>
    /// <returns></returns>
    static Directions create_fibonacci_sphere_samples(double angle, size_t count_samples);

private:
    // for debug purpose
    // store direction to STL file
    static bool store(const Directions &unit_z_rays);

};// class ShapeDiameterFunction
} // namespace Slic3r

#endif // slic3r_ShapeDiameterFunction_hpp_
