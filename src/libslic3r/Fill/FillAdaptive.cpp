#include "../ClipperUtils.hpp"
#include "../ExPolygon.hpp"
#include "../Surface.hpp"
#include "../Geometry.hpp"
#include "../AABBTreeIndirect.hpp"
#include "../Layer.hpp"
#include "../Print.hpp"
#include "../ShortestPath.hpp"

#include "FillAdaptive.hpp"

// Boost pool: Don't use mutexes to synchronize memory allocation.
#define BOOST_POOL_NO_MT
#include <boost/pool/object_pool.hpp>

namespace Slic3r {

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

        inline static int find_octant(const Vec3d &i_cube, const Vec3d &current)
        {
            return (i_cube.z() > current.z()) * 4 + (i_cube.y() > current.y()) * 2 + (i_cube.x() > current.x());
        }

        static void propagate_point(
            Octree                            &octree,
            Vec3d                              point,
            Cube                              *current_cube,
            int                                depth);
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

static std::vector<FillAdaptive_Internal::CubeProperties> make_cubes_properties(const BoundingBoxf3 &bbox, const Vec3d &center, double line_spacing)
{
    double max_cube_edge_length = bbox_max_radius(bbox, center) * 2. + EPSILON;

    std::vector<FillAdaptive_Internal::CubeProperties> cubes_properties;
    for (double edge_length = line_spacing * 2.; edge_length < max_cube_edge_length; edge_length *= 2.)
    {
        FillAdaptive_Internal::CubeProperties props{};
        props.edge_length = edge_length;
        props.height = edge_length * sqrt(3);
        props.diagonal_length = edge_length * sqrt(2);
        props.line_z_distance = edge_length / sqrt(3);
        props.line_xy_distance = edge_length / sqrt(6);
        cubes_properties.emplace_back(props);
    }
    return cubes_properties;
}

FillAdaptive_Internal::OctreePtr FillAdaptive::build_octree(
    TriangleMesh &triangle_mesh,
    coordf_t line_spacing,
    const Vec3d &cube_center)
{
    using namespace FillAdaptive_Internal;

    assert(line_spacing > 0);
    assert(! std::isnan(line_spacing));

    std::vector<CubeProperties> cubes_properties = make_cubes_properties(triangle_mesh.bounding_box(), cube_center, line_spacing);

    if (triangle_mesh.its.vertices.empty())
    {
        triangle_mesh.require_shared_vertices();
    }

    AABBTreeIndirect::Tree3f aabbTree = AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(
            triangle_mesh.its.vertices, triangle_mesh.its.indices);
    auto octree = OctreePtr(new Octree(cube_center, cubes_properties));

    FillAdaptive::expand_cube(*octree.get(), octree->root_cube, aabbTree, triangle_mesh, int(cubes_properties.size()) - 1);

    return octree;
}

// Children are ordered so that the lower index is always traversed before the higher index.
// This is important when chaining the generated lines, as one will have to chain the end of an existing line
// to the start of a new line only.
static const std::array<Vec3d, 8> child_centers {
    Vec3d(-1, -1, -1), Vec3d( 1, -1, -1), Vec3d(-1,  1, -1), Vec3d( 1,  1, -1),
    Vec3d(-1, -1,  1), Vec3d( 1, -1,  1), Vec3d(-1,  1,  1), Vec3d( 1,  1,  1)
};

void FillAdaptive::expand_cube(
    FillAdaptive_Internal::Octree& octree,
    FillAdaptive_Internal::Cube *cube,
    const AABBTreeIndirect::Tree3f &distance_tree,
    const TriangleMesh &triangle_mesh, int depth)
{
    using namespace FillAdaptive_Internal;

    if (cube == nullptr || depth == 0)
    {
        return;
    }

    double cube_radius_squared = (octree.cubes_properties[depth].height * octree.cubes_properties[depth].height) / 16;

    for (size_t i = 0; i < 8; ++i)
    {
        const Vec3d &child_center = child_centers[i];
        Vec3d child_center_transformed = cube->center + (child_center * (octree.cubes_properties[depth].edge_length / 4));

        if(AABBTreeIndirect::is_any_triangle_in_radius(triangle_mesh.its.vertices, triangle_mesh.its.indices,
            distance_tree, child_center_transformed, cube_radius_squared))
        {
            cube->children[i] = octree.pool.construct(child_center_transformed);
            FillAdaptive::expand_cube(octree, cube->children[i], distance_tree, triangle_mesh, depth - 1);
        }
    }
}

void FillAdaptive_Internal::Octree::propagate_point(
    Octree&                                                   octree,
    Vec3d                                                     point,
    FillAdaptive_Internal::Cube *                             current,
    int                                                       depth)
{
    using namespace FillAdaptive_Internal;

    if(depth <= 0)
    {
        return;
    }

    size_t octant_idx = Octree::find_octant(point, current->center);
    Cube * child = current->children[octant_idx];

    // Octant not exists, then create it
    if (child == nullptr) {
        const Vec3d &child_center = child_centers[octant_idx];
        Vec3d child_center_transformed = current->center + (child_center * (octree.cubes_properties[depth].edge_length / 4.));

        current->children[octant_idx] = octree.pool.construct(child_center_transformed);
        child = current->children[octant_idx];
    }

    Octree::propagate_point(octree, point, child, depth - 1);
}

FillAdaptive_Internal::OctreePtr FillSupportCubic::build_octree(
    TriangleMesh &     triangle_mesh,
    coordf_t           line_spacing,
    const Vec3d &      cube_center,
    const Transform3d &rotation_matrix)
{
    using namespace FillAdaptive_Internal;

    assert(line_spacing > 0);
    assert(! std::isnan(line_spacing));

    const BoundingBoxf3 mesh_bb = triangle_mesh.bounding_box();
    const std::vector<CubeProperties> cubes_properties = make_cubes_properties(mesh_bb, cube_center, line_spacing);

    if (triangle_mesh.its.vertices.empty())
    {
        triangle_mesh.require_shared_vertices();
    }

    auto octree = OctreePtr(new Octree(cube_center, cubes_properties));

    double cube_edge_length = line_spacing / 2.0;
    int max_depth = int(octree->cubes_properties.size()) - 1;
    const Vec3d cube_center_shift = Vec3d(cube_edge_length / 2., cube_edge_length / 2., 0.) + mesh_bb.min;

    for (const stl_facet &facet : triangle_mesh.stl.facet_start)
        if (facet.normal.z() > 0.707) {
            // Internal overhang, face angle is smaller than PI/4.
            // Generate dense cubes.
            const Vec3d       triangle_vertices[3] { facet.vertex[0].cast<double>(), facet.vertex[1].cast<double>(), facet.vertex[2].cast<double>() };
            BoundingBoxf3     triangle_bb;
            for (size_t i = 0; i < 3; ++ i)
                triangle_bb.merge(triangle_vertices[i]);

            Vec3d triangle_start_idx(((triangle_bb.min - mesh_bb.min) / cube_edge_length).array().floor());
            Vec3d triangle_end_idx  (((triangle_bb.max - mesh_bb.min) / cube_edge_length).array().floor() + Eigen::Array3d(EPSILON, EPSILON, EPSILON));
            Vec3d cube_idx;

            for (cube_idx.z() = triangle_start_idx.z(); cube_idx.z() < triangle_end_idx.z(); cube_idx.z() += 1.)
                for (cube_idx.y() = triangle_start_idx.y(); cube_idx.y() < triangle_end_idx.y(); cube_idx.y() += 1.)
                    for (cube_idx.x() = triangle_start_idx.x(); cube_idx.x() < triangle_end_idx.x(); cube_idx.x() += 1.) {
                        Vec3d cube_center_absolute(cube_idx * cube_edge_length + cube_center_shift);
                        double distance, cord_u, cord_v;
                        if (AABBTreeIndirect::detail::intersect_triangle(cube_center_absolute, Vec3d(0., 0., 1.), triangle_vertices[0], triangle_vertices[1], triangle_vertices[2], distance, cord_u, cord_v) && 
                            distance > 0. && distance <= cube_edge_length)
                        {
                            cube_center_absolute.z() += cube_edge_length / 2.0;
                            Octree::propagate_point(*octree.get(), rotation_matrix * cube_center_absolute, octree->root_cube, max_depth);
                        }
                    }
        }

    return octree;
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
