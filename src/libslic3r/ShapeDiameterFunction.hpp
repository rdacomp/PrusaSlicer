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
    /// DTO extends indexed_triangle_set with normals of triangles
    /// </summary>
    struct IndexTriangleNormals : public indexed_triangle_set
    {
        std::vector<Vec3f> triangle_normals; // same count as indexed_triangle_set.indices
        std::vector<Vec3f> vertex_normals;  // same count as indexed_triangle_set.vertuces     
    };

    struct AABBTree
    {
        AABBTreeIndirect::Tree3f tree;

        // for measure angle between ray and hit surface normal
        // condition of 90 deg from oposit direction of normal
        std::vector<Vec3f> triangle_normals; 

        // needed by function AABBTreeIndirect::intersect_ray_first_hit
        indexed_triangle_set vertices_indices;
    };

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
    /// <param name="allowed_deviation">Multiplicator of standart deviation to be result of ray counted,
    /// negative number means no filtration by deviation, 
    /// be carefull with value 1. std_dev and means are calculated with float precision</param>
    /// <param name="allowed_angle">Maximal angle between normal and hitted triangle normal [in Radians]
    /// negative number means no filtration by angle</param>
    /// <returns>Width of model for surface point</returns>
    static float calc_width(const Vec3f &     point,
                            const Vec3f &     normal,
                            const Directions &dirs,
                            const AABBTree &  tree,
                            float             allowed_deviation,
                            float             allowed_angle);

    /// <summary>
    /// Concurrent calculation of width for each vertex
    /// </summary>
    /// <param name="triangles">Vertices, indices and normals</param>
    /// <param name="dirs">Direction to cast rays</param>
    /// <param name="tree">AABB tree to fast detect first intersection</param>
    /// <param name="allowed_deviation">Multiplicator of standart deviation to be result of ray counted,
    /// negative number means no filtration by deviation, 
    /// be carefull with value 1. std_dev and means are calculated with float precision</param>
    /// <param name="allowed_angle">Maximal angle between normal and hitted triangle normal [in Radians]
    /// negative number means no filtration by angle</param>
    /// <returns>Width for each vertex</returns>
    static std::vector<float> calc_widths(const std::vector<Vec3f> &points,
                                          const std::vector<Vec3f> &normals,
                                          const Directions &        dirs,
                                          const AABBTree &          tree,
                                          float allowed_deviation = 1.5f,
                                          float allowed_angle     = M_PI_2);
    /// <summary>
    /// DTO represents point on surface of triangle mesh
    /// </summary>
    struct SurfacePoint
    {
        // position in space
        Vec3f pos;

        // triangle index
        // index to vector indexed_triangle_set::indices
        size_t ti; 
        SurfacePoint(Vec3f pos, size_t ti) : pos(pos), ti(ti) {}
    };
    using SurfacePoints = std::vector<SurfacePoint>;

    /// <summary>
    /// Create sample points on surface of model
    /// </summary>
    /// <param name="its">triangles</param>
    /// <param name="distance">maximal distance of samples</param>
    /// <returns>samples from surfaces</returns>
    static SurfacePoints sample(const indexed_triangle_set& its, float distance);

    /// <summary>
    /// Create neighbor for each edge of triangle
    /// </summary>
    /// <param name="indices">input Triangle indices</param>
    /// <param name="vertices_size">count vertices</param>
    /// <returns>vector of indices to trinagle indices, same size as indices(count triangle)</returns>
    static std::vector<Vec3crd> create_neighbor(
        const std::vector<stl_triangle_vertex_indices> &indices,
        size_t                                          vertices_size);

    /// <summary>
    /// Create points on unit sphere surface. with weight by z value
    /// </summary>
    /// <param name="angle"></param>
    /// <param name="count_samples"></param>
    /// <returns></returns>
    static Directions create_fibonacci_sphere_samples(double angle, size_t count_samples);

    /// <summary>
    /// divide each triangle with biger side length than max_length
    /// </summary>
    /// <param name="its"></param>
    /// <param name="max_length"></param>
    /// <returns>new triangle_set</returns>
    static indexed_triangle_set subdivide(const indexed_triangle_set &its, float max_length);

    /// <summary>
    /// Find shortest edge in index triangle set
    /// TODO: move to its utils
    /// </summary>
    /// <param name="its">input definition of triangle --> every triangle has 3 edges to explore</param>
    /// <returns>Length of shortest edge</returns>
    static float min_triangle_side_length(const indexed_triangle_set &its);

    /// <summary>
    /// Find shortest edge in index triangle set
    /// TODO: move to triangle utils
    /// </summary>
    /// <param name="v0">Triangle vertex</param>
    /// <param name="v1">Triangle vertex</param>
    /// <param name="v2">Triangle vertex</param> 
    /// <returns>Area of triangle</returns>
    static float triangle_area(const Vec3f &v0,
                               const Vec3f &v1,
                               const Vec3f &v2);

    /// <summary>
    /// Calculate area
    /// TODO: move to its utils
    /// </summary>
    /// <param name="inices">Triangle indices - index to vertices</param>
    /// <param name="vertices">Mesh vertices</param>
    /// <returns>Area of triangles</returns>
    static float triangle_area(const Vec3crd &inices, const std::vector<Vec3f> &vertices);

    /// <summary>
    /// Calculate surface area of all triangles defined by indices.
    /// TODO: move to its utils
    /// </summary>
    /// <param name="its">Indices and Vertices</param>
    /// <returns>Surface area as sum of triangle areas</returns>
    static float area(const indexed_triangle_set &its);

private:
    // for debug purpose
    // store direction to STL file
    static bool store(const Directions &unit_z_rays);

};// class ShapeDiameterFunction
} // namespace Slic3r

#endif // slic3r_ShapeDiameterFunction_hpp_
