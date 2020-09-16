#include "../ClipperUtils.hpp"
#include "../ExPolygon.hpp"
#include "../Surface.hpp"
#include "../Geometry.hpp"
#include "../Layer.hpp"
#include "../Print.hpp"
#include "../ShortestPath.hpp"

#include "FillAdaptive.hpp"

#include <cstdlib>
#include <cmath>

// Boost pool: Don't use mutexes to synchronize memory allocation.
#define BOOST_POOL_NO_MT
#include <boost/pool/object_pool.hpp>

namespace Slic3r {

// Derived from https://github.com/juj/MathGeoLib/blob/master/src/Geometry/Triangle.cpp
// The AABB-Triangle test implementation is based on the pseudo-code in
// Christer Ericson's Real-Time Collision Detection, pp. 169-172. It is
// practically a standard SAT test.
//
// Original MathGeoLib benchmark:
//    Best: 17.282 nsecs / 46.496 ticks, Avg: 17.804 nsecs, Worst: 18.434 nsecs
//
//FIXME Vojtech: The MathGeoLib contains a vectorized implementation.
template<typename Vector> 
bool triangle_AABB_intersects(const Vector &a, const Vector &b, const Vector &c, const BoundingBoxBase<Vector> &aabb)
{
    using Scalar = typename Vector::Scalar;

    Vector tMin = a.cwiseMin(b.cwiseMin(c));
    Vector tMax = a.cwiseMax(b.cwiseMax(c));

    if (tMin.x() >= aabb.max.x() || tMax.x() <= aabb.min.x()
        || tMin.y() >= aabb.max.y() || tMax.y() <= aabb.min.y()
        || tMin.z() >= aabb.max.z() || tMax.z() <= aabb.min.z())
        return false;

    Vector center = (aabb.min + aabb.max) * 0.5f;
    Vector h = aabb.max - center;

    const Vector t[3] { b-a, c-a, c-b };

    Vector ac = a - center;

    Vector n = t[0].cross(t[1]);
    Scalar s = n.dot(ac);
    Scalar r = std::abs(h.dot(n.cwiseAbs()));
    if (abs(s) >= r)
        return false;

    const Vector at[3] = { t[0].cwiseAbs(), t[1].cwiseAbs(), t[2].cwiseAbs() };

    Vector bc = b - center;
    Vector cc = c - center;

    // SAT test all cross-axes.
    // The following is a fully unrolled loop of this code, stored here for reference:
    /*
    Scalar d1, d2, a1, a2;
    const Vector e[3] = { DIR_VEC(1, 0, 0), DIR_VEC(0, 1, 0), DIR_VEC(0, 0, 1) };
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j)
        {
            Vector axis = Cross(e[i], t[j]);
            ProjectToAxis(axis, d1, d2);
            aabb.ProjectToAxis(axis, a1, a2);
            if (d2 <= a1 || d1 >= a2) return false;
        }
    */

    // eX <cross> t[0]
    Scalar d1 = t[0].y() * ac.z() - t[0].z() * ac.y();
    Scalar d2 = t[0].y() * cc.z() - t[0].z() * cc.y();
    Scalar tc = (d1 + d2) * 0.5f;
    r = std::abs(h.y() * at[0].z() + h.z() * at[0].y());
    if (r + std::abs(tc - d1) < std::abs(tc))
        return false;

    // eX <cross> t[1]
    d1 = t[1].y() * ac.z() - t[1].z() * ac.y();
    d2 = t[1].y() * bc.z() - t[1].z() * bc.y();
    tc = (d1 + d2) * 0.5f;
    r = std::abs(h.y() * at[1].z() + h.z() * at[1].y());
    if (r + std::abs(tc - d1) < std::abs(tc))
        return false;

    // eX <cross> t[2]
    d1 = t[2].y() * ac.z() - t[2].z() * ac.y();
    d2 = t[2].y() * bc.z() - t[2].z() * bc.y();
    tc = (d1 + d2) * 0.5f;
    r = std::abs(h.y() * at[2].z() + h.z() * at[2].y());
    if (r + std::abs(tc - d1) < std::abs(tc))
        return false;

    // eY <cross> t[0]
    d1 = t[0].z() * ac.x() - t[0].x() * ac.z();
    d2 = t[0].z() * cc.x() - t[0].x() * cc.z();
    tc = (d1 + d2) * 0.5f;
    r = std::abs(h.x() * at[0].z() + h.z() * at[0].x());
    if (r + std::abs(tc - d1) < std::abs(tc))
        return false;

    // eY <cross> t[1]
    d1 = t[1].z() * ac.x() - t[1].x() * ac.z();
    d2 = t[1].z() * bc.x() - t[1].x() * bc.z();
    tc = (d1 + d2) * 0.5f;
    r = std::abs(h.x() * at[1].z() + h.z() * at[1].x());
    if (r + std::abs(tc - d1) < std::abs(tc))
        return false;

    // eY <cross> t[2]
    d1 = t[2].z() * ac.x() - t[2].x() * ac.z();
    d2 = t[2].z() * bc.x() - t[2].x() * bc.z();
    tc = (d1 + d2) * 0.5f;
    r = std::abs(h.x() * at[2].z() + h.z() * at[2].x());
    if (r + std::abs(tc - d1) < std::abs(tc))
        return false;

    // eZ <cross> t[0]
    d1 = t[0].x() * ac.y() - t[0].y() * ac.x();
    d2 = t[0].x() * cc.y() - t[0].y() * cc.x();
    tc = (d1 + d2) * 0.5f;
    r = std::abs(h.y() * at[0].x() + h.x() * at[0].y());
    if (r + std::abs(tc - d1) < std::abs(tc))
        return false;

    // eZ <cross> t[1]
    d1 = t[1].x() * ac.y() - t[1].y() * ac.x();
    d2 = t[1].x() * bc.y() - t[1].y() * bc.x();
    tc = (d1 + d2) * 0.5f;
    r = std::abs(h.y() * at[1].x() + h.x() * at[1].y());
    if (r + std::abs(tc - d1) < std::abs(tc))
        return false;

    // eZ <cross> t[2]
    d1 = t[2].x() * ac.y() - t[2].y() * ac.x();
    d2 = t[2].x() * bc.y() - t[2].y() * bc.x();
    tc = (d1 + d2) * 0.5f;
    r = std::abs(h.y() * at[2].x() + h.x() * at[2].y());
    if (r + std::abs(tc - d1) < std::abs(tc))
        return false;

    // No separating axis exists, the AABB and triangle intersect.
    return true;
}

namespace FillAdaptive_Internal
{
    struct Cube
    {
        Vec3d center;
        std::array<Cube*, 8> children {}; // initialized to nullptrs
        Cube(const Vec3d &center) : center(center) {}
    };

    struct Octree
    {
        // Octree will allocate its Cubes from the pool. The pool only supports deletion of the complete pool,
        // perfect for building up our octree.
        boost::object_pool<Cube>    pool;
        Cube*                       root_cube { nullptr };
        Vec3d                       origin;
        std::vector<CubeProperties> cubes_properties;

        Octree(const Vec3d &origin, const std::vector<CubeProperties> &cubes_properties)
            : root_cube(pool.construct(origin)), origin(origin), cubes_properties(cubes_properties) {}

        void insert_triangle(const Vec3d &a, const Vec3d &b, const Vec3d &c, Cube *current_cube, BoundingBoxf3 &current_bbox, int depth);
    };

    void OctreeDeleter::operator()(Octree *p) {
        delete p;
    }
}; // namespace FillAdaptive_Internal

std::pair<double, double> adaptive_fill_line_spacing(const PrintObject &print_object)
{
    // Output, spacing for icAdaptiveCubic and icSupportCubic
    double  adaptive_line_spacing = 0.;
    double  support_line_spacing = 0.;

    enum class Tristate {
        Yes,
        No,
        Maybe
    };
    struct RegionFillData {
        Tristate        has_adaptive_infill;
        Tristate        has_support_infill;
        double          density;
        double          extrusion_width;
    };
    std::vector<RegionFillData> region_fill_data;
    region_fill_data.reserve(print_object.print()->regions().size());
    bool                        build_octree = false;
    for (const PrintRegion *region : print_object.print()->regions()) {
        const PrintRegionConfig &config   = region->config();
        bool                     nonempty = config.fill_density > 0;
        bool                     has_adaptive_infill = nonempty && config.fill_pattern == ipAdaptiveCubic;
        bool                     has_support_infill  = nonempty && config.fill_pattern == ipSupportCubic;
        region_fill_data.push_back(RegionFillData({
            has_adaptive_infill ? Tristate::Maybe : Tristate::No,
            has_support_infill ? Tristate::Maybe : Tristate::No,
            config.fill_density,
            config.infill_extrusion_width
        }));
        build_octree |= has_adaptive_infill || has_support_infill;
    }

    if (build_octree) {
        // Compute the average of above parameters over all layers
        for (const Layer *layer : print_object.layers())
            for (size_t region_id = 0; region_id < layer->regions().size(); ++ region_id) {
                RegionFillData &rd = region_fill_data[region_id];
                if (rd.has_adaptive_infill == Tristate::Maybe && ! layer->regions()[region_id]->fill_surfaces.empty())
                    rd.has_adaptive_infill = Tristate::Yes;
                if (rd.has_support_infill == Tristate::Maybe && ! layer->regions()[region_id]->fill_surfaces.empty())
                    rd.has_support_infill = Tristate::Yes;
            }

        double  adaptive_fill_density           = 0.;
        double  adaptive_infill_extrusion_width = 0.;
        int     adaptive_cnt                    = 0;
        double  support_fill_density            = 0.;
        double  support_infill_extrusion_width  = 0.;
        int     support_cnt                     = 0;

        for (const RegionFillData &rd : region_fill_data) {
            if (rd.has_adaptive_infill == Tristate::Yes) {
                adaptive_fill_density           += rd.density;
                adaptive_infill_extrusion_width += rd.extrusion_width;
                ++ adaptive_cnt;
            } else if (rd.has_support_infill == Tristate::Yes) {
                support_fill_density           += rd.density;
                support_infill_extrusion_width += rd.extrusion_width;
                ++ support_cnt;
            }
        }

        auto to_line_spacing = [](int cnt, double density, double extrusion_width) {
            if (cnt) {
                density         /= double(cnt);
                extrusion_width /= double(cnt);
                return extrusion_width / ((density / 100.0f) * 0.333333333f);
            } else
                return 0.;
        };
        adaptive_line_spacing = to_line_spacing(adaptive_cnt, adaptive_fill_density, adaptive_infill_extrusion_width);
        support_line_spacing  = to_line_spacing(support_cnt, support_fill_density, support_infill_extrusion_width);
    }

    return std::make_pair(adaptive_line_spacing, support_line_spacing);
}

void FillAdaptive::_fill_surface_single(const FillParams &             params,
                                        unsigned int                   thickness_layers,
                                        const std::pair<float, Point> &direction,
                                        ExPolygon &                    expolygon,
                                        Polylines &                    polylines_out)
{
    if (this->adapt_fill_octree != nullptr)
        this->generate_infill(params, thickness_layers, direction, expolygon, polylines_out, this->adapt_fill_octree);
}

void FillAdaptive::generate_infill(const FillParams &                  params,
                                        unsigned int                   thickness_layers,
                                        const std::pair<float, Point> &direction,
                                        ExPolygon &                    expolygon,
                                        Polylines &                    polylines_out,
                                        FillAdaptive_Internal::Octree *octree)
{
    Vec3d rotation = Vec3d((5.0 * M_PI) / 4.0, Geometry::deg2rad(215.264), M_PI / 6.0);
    Transform3d rotation_matrix = Geometry::assemble_transform(Vec3d::Zero(), rotation, Vec3d::Ones(), Vec3d::Ones());

    // Store grouped lines by its direction (multiple of 120°)
    std::vector<Lines> infill_lines_dir(3);
    this->generate_infill_lines(octree->root_cube,
                                this->z, octree->origin, rotation_matrix,
                                infill_lines_dir, octree->cubes_properties,
                                int(octree->cubes_properties.size()) - 1);

    Polylines all_polylines;
    all_polylines.reserve(infill_lines_dir[0].size() * 3);
    for (Lines &infill_lines : infill_lines_dir)
    {
        for (const Line &line : infill_lines)
        {
            all_polylines.emplace_back(line.a, line.b);
        }
    }

    if (params.dont_connect)
    {
        // Crop all polylines
        polylines_out = intersection_pl(all_polylines, to_polygons(expolygon));
    }
    else
    {
        // Crop all polylines
        all_polylines = intersection_pl(all_polylines, to_polygons(expolygon));

        Polylines boundary_polylines;
        Polylines non_boundary_polylines;
        for (const Polyline &polyline : all_polylines)
        {
            // connect_infill required all polylines to touch the boundary.
            if(polyline.lines().size() == 1 && expolygon.has_boundary_point(polyline.lines().front().a) && expolygon.has_boundary_point(polyline.lines().front().b))
            {
                boundary_polylines.push_back(polyline);
            }
            else
            {
                non_boundary_polylines.push_back(polyline);
            }
        }

        if (!boundary_polylines.empty())
        {
            boundary_polylines = chain_polylines(boundary_polylines);
            FillAdaptive::connect_infill(std::move(boundary_polylines), expolygon, polylines_out, this->spacing, params);
        }

        append(polylines_out, std::move(non_boundary_polylines));
    }

#ifdef SLIC3R_DEBUG_SLICE_PROCESSING
    {
        static int iRuna = 0;
        BoundingBox bbox_svg = this->bounding_box;
        {
            ::Slic3r::SVG svg(debug_out_path("FillAdaptive-%d.svg", iRuna), bbox_svg);
            for (const Polyline &polyline : polylines_out)
            {
                for (const Line &line : polyline.lines())
                {
                    Point from = line.a;
                    Point to = line.b;
                    Point diff = to - from;

                    float shrink_length = scale_(0.4);
                    float line_slope = (float)diff.y() / diff.x();
                    float shrink_x = shrink_length / (float)std::sqrt(1.0 + (line_slope * line_slope));
                    float shrink_y = line_slope * shrink_x;

                    to.x() -= shrink_x;
                    to.y() -= shrink_y;
                    from.x() += shrink_x;
                    from.y() += shrink_y;

                    svg.draw(Line(from, to));
                }
            }
        }

        iRuna++;
    }
#endif /* SLIC3R_DEBUG */
}

void FillAdaptive::generate_infill_lines(
        const FillAdaptive_Internal::Cube *cube,
        double z_position,
        const Vec3d &origin,
        const Transform3d &rotation_matrix,
        std::vector<Lines> &dir_lines_out,
        const std::vector<FillAdaptive_Internal::CubeProperties> &cubes_properties,
        int depth)
{
    using namespace FillAdaptive_Internal;

    assert(cube != nullptr);

    Vec3d cube_center_tranformed = rotation_matrix * cube->center;
    double z_diff = std::abs(z_position - cube_center_tranformed.z());

    if (z_diff > cubes_properties[depth].height / 2)
    {
        return;
    }

    if (z_diff < cubes_properties[depth].line_z_distance)
    {
        Point from(
                scale_((cubes_properties[depth].diagonal_length / 2) * (cubes_properties[depth].line_z_distance - z_diff) / cubes_properties[depth].line_z_distance),
                scale_(cubes_properties[depth].line_xy_distance - ((z_position - (cube_center_tranformed.z() - cubes_properties[depth].line_z_distance)) / sqrt(2))));
        Point to(-from.x(), from.y());
        // Relative to cube center

        double rotation_angle = (2.0 * M_PI) / 3.0;
        Vec3d  offset3 = cube_center_tranformed - rotation_matrix * origin;
        auto   offset  = (Vec2d(offset3.x(), offset3.y()) * (1. / SCALING_FACTOR)).cast<coord_t>();
        for (Lines &lines : dir_lines_out)
        {
            this->connect_lines(lines, Line(from + offset, to + offset));
            from.rotate(rotation_angle);
            to.rotate(rotation_angle);
        }
    }

    for (const Cube *child : cube->children)
        if (child != nullptr)
            generate_infill_lines(child, z_position, origin, rotation_matrix, dir_lines_out, cubes_properties, depth - 1);
}

void FillAdaptive::connect_lines(Lines &lines, Line new_line)
{
    for (Line &line : lines)
        if ((new_line.a - line.b).cwiseAbs().maxCoeff() < SCALED_EPSILON) {
            line.b = new_line.b;
            return;
        }
    lines.emplace_back(new_line);
}

static double bbox_max_radius(const BoundingBoxf3 &bbox, const Vec3d &center)
{
    const auto p = (bbox.min - center);
    const auto s = bbox.size();
    double r2max = 0.;
    for (int i = 0; i < 8; ++ i)
        r2max = std::max(r2max, (p + Vec3d(s.x() * double(i & 1), s.y() * double(i & 2), s.z() * double(i & 4))).squaredNorm());
    return sqrt(r2max);
}

static std::vector<FillAdaptive_Internal::CubeProperties> make_cubes_properties(double max_cube_edge_length, double line_spacing)
{
    max_cube_edge_length += EPSILON;

    std::vector<FillAdaptive_Internal::CubeProperties> cubes_properties;
    for (double edge_length = line_spacing * 2.;; edge_length *= 2.)
    {
        FillAdaptive_Internal::CubeProperties props{};
        props.edge_length = edge_length;
        props.height = edge_length * sqrt(3);
        props.diagonal_length = edge_length * sqrt(2);
        props.line_z_distance = edge_length / sqrt(3);
        props.line_xy_distance = edge_length / sqrt(6);
        cubes_properties.emplace_back(props);
        if (edge_length > max_cube_edge_length)
            break;
    }
    return cubes_properties;
}

static inline bool is_overhang_triangle(const Vec3d &a, const Vec3d &b, const Vec3d &c, const Vec3d &up)
{
    // Calculate triangle normal.
    auto n = (b - a).cross(c - b);
    return n.dot(up) > 0.707 * n.norm();
}

FillAdaptive_Internal::OctreePtr FillAdaptive::build_octree(const indexed_triangle_set &triangle_mesh, const Vec3d &up_vector, coordf_t line_spacing, bool support_overhangs_only)
{
    using namespace FillAdaptive_Internal;

    assert(line_spacing > 0);
    assert(! std::isnan(line_spacing));

    BoundingBox3Base<Vec3f>     bbox(triangle_mesh.vertices);
    Vec3d                       cube_center      = bbox.center().cast<double>();
    std::vector<CubeProperties> cubes_properties = make_cubes_properties(double(bbox.size().maxCoeff()), line_spacing);
    auto                        octree           = OctreePtr(new Octree(cube_center, cubes_properties));

    if (cubes_properties.size() > 1) {
        for (auto &tri : triangle_mesh.indices) {
            auto a = triangle_mesh.vertices[tri[0]].cast<double>();
            auto b = triangle_mesh.vertices[tri[1]].cast<double>();
            auto c = triangle_mesh.vertices[tri[2]].cast<double>();
            if (support_overhangs_only && ! is_overhang_triangle(a, b, c, up_vector))
                continue;
            double edge_length_half = 0.5 * cubes_properties.back().edge_length;
            Vec3d  diag_half(edge_length_half, edge_length_half, edge_length_half);
            octree->insert_triangle(
                a, b, c,
                octree->root_cube, 
                BoundingBoxf3(octree->root_cube->center - diag_half, octree->root_cube->center + diag_half),
                int(cubes_properties.size()) - 1);
        }
    }

    return octree;
}

// Children are ordered so that the lower index is always traversed before the higher index.
// This is important when chaining the generated lines, as one will have to chain the end of an existing line
// to the start of a new line only.
static const std::array<Vec3d, 8> child_centers {
    Vec3d(-1, -1, -1), Vec3d( 1, -1, -1), Vec3d(-1,  1, -1), Vec3d( 1,  1, -1),
    Vec3d(-1, -1,  1), Vec3d( 1, -1,  1), Vec3d(-1,  1,  1), Vec3d( 1,  1,  1)
};

void FillAdaptive_Internal::Octree::insert_triangle(const Vec3d &a, const Vec3d &b, const Vec3d &c, Cube *current_cube, BoundingBoxf3 &current_bbox, int depth)
{
    assert(current_cube);
    assert(depth > 0);

    for (size_t i = 0; i < 8; ++ i) {
        const Vec3d &child_center = child_centers[i];
        // Calculate a slightly expanded bounding box of a child cube to cope with triangles touching a cube wall and other numeric errors.
        // We will rather densify the octree a bit more than necessary instead of missing a triangle.
        BoundingBoxf3 bbox;
        for (int k = 0; k < 3; ++ k) {
            if (child_center[k] == -1.) {
                bbox.min[k] = current_bbox.min[k];
                bbox.max[k] = current_cube->center[k] + EPSILON;
            } else {
                bbox.min[k] = current_cube->center[k] - EPSILON;
                bbox.max[k] = current_bbox.max[k];
            }
        }
        if (triangle_AABB_intersects(a, b, c, bbox)) {
            if (! current_cube->children[i])
                current_cube->children[i] = this->pool.construct(current_cube->center + (child_center * (this->cubes_properties[depth].edge_length / 4)));
            if (depth > 1)
                this->insert_triangle(a, b, c, current_cube->children[i], bbox, depth - 1);
        }
    }
}

void FillSupportCubic::_fill_surface_single(const FillParams &             params,
                                            unsigned int                   thickness_layers,
                                            const std::pair<float, Point> &direction,
                                            ExPolygon &                    expolygon,
                                            Polylines &                    polylines_out)
{
    assert(this->support_fill_octree);
    this->generate_infill(params, thickness_layers, direction, expolygon, polylines_out, this->support_fill_octree);
}

} // namespace Slic3r
