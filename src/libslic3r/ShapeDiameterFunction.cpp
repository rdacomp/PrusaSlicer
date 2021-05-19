#include "ShapeDiameterFunction.hpp"

using namespace Slic3r;

float ShapeDiameterFunction::calc_width(const Vec3f &               point,
                                        const Vec3f &               normal,
                                        const Directions &          dirs,
                                        const indexed_triangle_set &its,
                                        const AABBTreeIndirect::Tree3f &tree)
{
    // safe against ray intersection with origin trinagle made by source vertex
    const double safe_move = 1e-5;
    // value for width when no intersection
    const float no_width = -1.;

    Vec3f ray_dir   = -normal;
    Vec3d ray_point = (point + ray_dir * safe_move).cast<double>();

    const Vec3f z_axe(0.f, 0.f, 1.f);
    Vec3f       axis  = z_axe.cross(ray_dir);
    float       angle = std::acos(z_axe.dot(ray_dir));

    auto  tr_mat     = Eigen::AngleAxis<float>(angle, axis).matrix();
    float sum_width  = 0.f;
    float sum_weight = 0.f;
    for (const auto &dir : dirs) {
        const Vec3f &ray     = dir.dir;
        Vec3f        ray_tr  = tr_mat * ray;
        Vec3d        ray_trd = ray_tr.cast<double>();
        igl::Hit     hit;
        if (AABBTreeIndirect::intersect_ray_first_hit(its.vertices,
                                                      its.indices, tree,
                                                      ray_point, ray_trd,
                                                      hit)) {
            sum_width += hit.t * dir.weight;
            sum_weight += dir.weight;
        }
    }
    if (sum_weight <= 0.) return no_width;
    return sum_width / sum_weight + safe_move;
}

Vec3f ShapeDiameterFunction::create_triangle_normal(
    const stl_triangle_vertex_indices &indices,
    const std::vector<stl_vertex> &    vertices)
{
    const stl_vertex &v0        = vertices[indices[0]];
    const stl_vertex &v1        = vertices[indices[1]];
    const stl_vertex &v2        = vertices[indices[2]];
    Vec3f             direction = (v1 - v0).cross(v2 - v0);
    direction.normalize();
    return direction;
}

std::vector<Vec3f> ShapeDiameterFunction::create_triangle_normals(
    const indexed_triangle_set &its)
{
    std::vector<Vec3f> normals;
    normals.reserve(its.indices.size());
    for (const Vec3crd &index : its.indices) {
        normals.push_back(create_triangle_normal(index, its.vertices));
    }
    return normals;
}

// calculate normals by averaging normals of neghbor triangles
std::vector<Vec3f> ShapeDiameterFunction::create_normals(
    const indexed_triangle_set &its)
{
    size_t count_vertices = its.vertices.size();
    assert(its.indices.size() == triangle_normals.size());
    std::vector<Vec3f>         normals(count_vertices, Vec3f(.0, .0, .0));
    std::vector<unsigned char> count(count_vertices, 0);
    for (const Vec3crd &indice : its.indices) {
        size_t index  = &indice - &its.indices.front();
        Vec3f  normal = create_triangle_normal(indice, its.vertices);
        for (int i = 0; i < 3; ++i) {
            normals[indice[i]] += normal;
            ++count[indice[i]];
        }
    }
    // normalize to size 1
    for (auto &normal : normals) {
        size_t index = &normal - &normals.front();
        normal /= static_cast<float>(count[index]);
    }
    return normals;
}

// create points on unit sphere surface
ShapeDiameterFunction::Directions
ShapeDiameterFunction::create_fibonacci_sphere_samples(double angle,
                                                         size_t count_samples)
{
    if (count_samples <= 1) {
        Direction d;
        d.dir    = Vec3f(0.f, 0.f, 1.f);
        d.weight = 1.f;
        return {d};
    }
    assert(angle < 180);
    assert(angle > 1);
    double min_z = cos(angle / 2. * M_PI / 180.);
    assert(min_z > 0.);

    Directions points;
    points.reserve(count_samples);
    const double phi = M_PI * (3. - sqrt(5.)); // golden angle in radians
    for (size_t i = 0; i < count_samples; ++i) {
        double z = 1. - (i / double(count_samples - 1));
        if (z < min_z) break;
        double    radius = sqrt(1. - z * z); // radius at z
        double    theta  = phi * i;          // golden angle increment
        double    x      = cos(theta) * radius;
        double    y      = sin(theta) * radius;
        Direction d;
        d.dir    = Vec3f(static_cast<float>(x), static_cast<float>(y),
                      static_cast<float>(z));
        d.weight = d.dir.z();
        points.push_back(d);
    }
    // store(points);
    return points;
}

bool ShapeDiameterFunction::store(const Directions &unit_z_rays)
{
    TriangleMesh tm;
    stl_file &   stl = tm.stl;
    stl.facet_start.reserve(2 * unit_z_rays.size());
    float triangle_size   = 1e-1f;
    float triangle_length = 1.f + 2.f;
    for (const auto &dir : unit_z_rays) {
        Vec3f     ray = dir.dir;
        stl_facet facet;
        facet.normal    = Vec3f(0.f, 1.f, 0.f);
        facet.vertex[0] = ray * triangle_length;
        facet.vertex[1] = ray + Vec3f(triangle_size / 2.f, 0.f, 0.f);
        facet.vertex[2] = ray + Vec3f(-triangle_size / 2.f, 0.f, 0.f);
        stl.facet_start.push_back(facet);
        stl_facet facet2;
        facet2.normal    = Vec3f(1.f, 0.f, 0.f);
        facet2.vertex[0] = facet.vertex[0];
        facet2.vertex[1] = ray + Vec3f(0.f, triangle_size / 2.f, 0.f);
        facet2.vertex[2] = ray + Vec3f(0.f, -triangle_size / 2.f, 0.f);
        stl.facet_start.push_back(facet2);
    }
    stl.stats.number_of_facets = stl.facet_start.size();
    return tm.write_ascii("unit_z_rays.stl");
}