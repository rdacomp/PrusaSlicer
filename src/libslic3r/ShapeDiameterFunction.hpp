#ifndef slic3r_ShapeDiameterFunction_hpp_
#define slic3r_ShapeDiameterFunction_hpp_

#include "Point.hpp"
#include "Model.hpp"
#include "AABBTreeIndirect.hpp"

#include <random> // sampling

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
    /// DTO with valus for calculate width by SDF
    /// </summary>
    struct Config
    {
        // Multiplicator of standart deviation to be result of ray counted,
        // negative number means no filtration by deviation
        // be carefull with value close 1. Standart deviation and means are calculated with float precision only.
        // when use only 2 values non of them needs to be in deviation
        float allowed_deviation = 1.5f; // [in std dev multiplication]

        // Maximal angle between normal and hitted triangle normal[in Radians]
        // negative number means no filtration by angle
        float allowed_angle = -1.f; 
        //for no oposit direction use value: static_cast<float>(M_PI_2) + std::numeric_limits<float>::epsilon();

        // Direction to cast rays with unit size
        // Before use z direction is rotated to negative normal of point
        Directions dirs =
            ShapeDiameterFunction::create_fibonacci_sphere_samples(120., 60);

        // create config with default values
        Config() = default;

        void set_no_deviation_filtering() { allowed_deviation = -1.f; }
        bool is_deviation_filtering() const { return allowed_deviation > 0; }
        void set_no_angle_filtering() { allowed_angle = -1.f; }
        bool is_angle_filtering() const { return allowed_angle > 0; }
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
    /// <param name="tree">AABB tree to fast detect first intersection</param>
    /// <param name="config">Calculation configuration</param>
    /// <returns>Width of model for surface point</returns>
    static float calc_width(const Vec3f &     point,
                            const Vec3f &     normal,
                            const AABBTree &  tree,
                            const Config &config);

    /// <summary>
    /// Concurrent calculation of width for each vertex
    /// </summary>
    /// <param name="triangles">Vertices, indices and normals</param>
    /// <param name="dirs">Direction to cast rays</param>
    /// <param name="tree">AABB tree to fast detect first intersection</param>
    /// <returns>Width for each vertex</returns>
    static std::vector<float> calc_widths(const std::vector<Vec3f> &points,
                                          const std::vector<Vec3f> &normals,
                                          const AABBTree &          tree,
                                          const Config &            config);

    struct SampleConfig
    {
        // range of width to support for linear distributiion of count supports
        float min_width = 0.1f; 
        float max_width = 10.f;
        // range of generated support count
        float max_area_support = 20.f; // [in mm^2]
        float min_area_support = 2.f; // [in mm^2]

        // min_width is supported by max_area_support
        // max_width is supported by min_area_support

        SampleConfig() = default;
    };

    /// <summary>
    /// Generate surface points on tiny part of model
    /// Traingle should have normalized size of side
    /// </summary>
    /// <param name="its">Define vertices and indices of triangle mesh</param>
    /// <param name="widths">Width for each vertex(same size as its::vetices)</param>
    /// <param name="cfg">configuration where to generate support</param>
    /// <returns>Vector of surface points</returns>
    static std::vector<Vec3f> generate_support_points(
        const indexed_triangle_set &its, const std::vector<float> &widths,
        const SampleConfig& cfg);

    /// <summary>
    /// Create points on unit sphere surface. with weight by z value
    /// </summary>
    /// <param name="angle">Cone angle in DEG, filtrate uniform sample of half sphere</param>
    /// <param name="count_samples">Count samples for half sphere</param>
    /// <returns>Unit vectors lay inside cone with direction to Z axis</returns>
    static Directions create_fibonacci_sphere_samples(double angle, size_t count_samples);

    /// <summary>
    /// divide each triangle with biger side length than max_length
    /// </summary>
    /// <param name="its">Input vertices and faces</param>
    /// <param name="max_length">Maximal length</param>
    /// <returns>new triangle_set</returns>
    static indexed_triangle_set subdivide(const indexed_triangle_set &its, float max_length);

    /// <summary>
    /// Minimal length of triangle side, smaller side will be removed
    /// </summary>
    /// <param name="its">Input vertices and faces</param>
    /// <param name="min_length">Minimal length of triangle side</param>
    /// <param name="max_error">Maximal error during reduce of triangle side</param>
    /// <returns>Re-meshed</returns>
    static void connect_small_triangles(indexed_triangle_set &its, float min_length, float max_error); 

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
