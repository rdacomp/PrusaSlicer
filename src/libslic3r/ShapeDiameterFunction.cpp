#include "ShapeDiameterFunction.hpp"
#include <random>
#include <tbb/parallel_for.h>
#include "TriangleMesh.hpp"

using namespace Slic3r;

float ShapeDiameterFunction::calc_width(const Vec3f &     point,
                                        const Vec3f &     normal,
                                        const Directions &dirs,
                                        const AABBTree &  tree,
                                        float             allowed_deviation,
                                        float             allowed_angle)
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
    std::vector<float> widths;
    widths.reserve(dirs.size());
    std::vector<float> weights;
    weights.reserve(dirs.size());

    float sum_width  = 0.f;
    // squer sum of widths
    float sq_sum_width = 0.f; 
    const std::vector<stl_vertex>& vertices  = tree.vertices_indices.vertices;
    const std::vector<stl_triangle_vertex_indices>& indices = tree.vertices_indices.indices;
    for (const auto &dir : dirs) {
        const Vec3f &ray     = dir.dir;
        Vec3f        ray_tr  = tr_mat * ray;
        Vec3d        ray_trd = ray_tr.cast<double>();
        igl::Hit     hit;
        if (!AABBTreeIndirect::intersect_ray_first_hit(vertices, indices, tree.tree,
                                                       ray_point, ray_trd, hit))
            continue;

        // check angle of hitted traingle
        Vec3f hit_normal = tree.triangle_normals[hit.id];
        float dot        = normal.dot(hit_normal);
        if (dot < -1.f) dot = -1.f;
        if (dot > 1.f) dot = 1.f;
        float angle = std::acos(dot);
        // IMPROVE: Test correct border angle It could be bigger than 90 DEG
        if (angle < allowed_angle) continue;  

        float width = hit.t;
        widths.push_back(width);
        weights.push_back(dir.weight);
        sum_width += width;
        sq_sum_width += width * width;
    }
    if (widths.empty()) return no_width;
    if (widths.size() == 1) return widths.front();

    // statistics of widths - mean and standart deviation
    float mean = sum_width / widths.size();
    float standard_deviation = std::sqrt(sq_sum_width / widths.size() - mean * mean);
    float threshold_deviation = standard_deviation * allowed_deviation;
    sum_width = 0.f;
    float sum_weight = 0.f;
    for (size_t i = 0; i < widths.size(); i++) {
        const float &width = widths[i];
        // skip values out of standart deviation
        if (fabs(width - mean) > threshold_deviation) continue;
        const float &weight = weights[i];
        sum_width += width * weight;
        sum_weight += weight;
    }
    if (sum_weight <= 0.) return mean;
    return sum_width / sum_weight + safe_move;
}

std::vector<float> ShapeDiameterFunction::calc_widths(
    const std::vector<Vec3f> &points,
    const std::vector<Vec3f> &normals,
    const Directions &        dirs,
    const AABBTree &          tree,
    float                     allowed_deviation,
    float                     allowed_angle)
{
    // check input
    assert(!points.empty());
    assert(!dirs.empty());
    assert(points.size() == normals.size());
    if (points.empty() || dirs.empty() ||
        points.size() != normals.size()) return {};

    static constexpr size_t granularity = 64;
    size_t                  size        = points.size();
    std::vector<float>      widths(size);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, size),
        [&](const tbb::blocked_range<size_t> &range) {
            for (size_t index = range.begin();
            index < range.end(); ++index) {
            const Vec3f &vertex = points[index];
            const Vec3f &normal = normals[index];
            widths[index] = calc_width(vertex, normal, dirs, tree,
                                        allowed_deviation, allowed_angle);
        }
    });
    return widths;
}

ShapeDiameterFunction::SurfacePoints ShapeDiameterFunction::sample(
    const indexed_triangle_set &its, float distance)
{
    // count samples per area of distance
    int   random_count = 10; 
    // random generator
    std::mt19937 m_rng;
    std::random_device rd;
    m_rng.seed(rd());
    std::uniform_real_distribution<float> xDist(0.f, 1.f);
    std::uniform_real_distribution<float> yDist(0.f, 1.f);
    std::uniform_real_distribution<float> generate_point(0.f, 1.f);
    float sample_area  = distance * distance / random_count;
    // random sample triangle
    float sum_areas = 0;
    SurfacePoints sPts;
    for (const Vec3crd &triangle_indices : its.indices) {
        size_t       index = &triangle_indices - &its.indices.front();
        float area = triangle_area(triangle_indices, its.vertices);
        float        countf = area / sample_area;
        int count = static_cast<int>(std::floor(area / sample_area));
        float generate = generate_point(m_rng);
        if (generate < (countf-count)) ++count;
        if (count == 0) continue;

        const Vec3f &v0 = its.vertices[triangle_indices[0]];
        const Vec3f &v1 = its.vertices[triangle_indices[1]];
        const Vec3f &v2 = its.vertices[triangle_indices[2]];
        for (int c = 0; c < count; c++) {
            // barycentric coordinate
            Vec3f b;
            b[0] = xDist(m_rng);
            b[1] = yDist(m_rng);
            if ((b[0] + b[1]) > 1.f) {
                b[0] = 1.f - b[0];
                b[1] = 1.f - b[1];
            }
            b[2] = 1.f - b[0] - b[1];
            Vec3f pos;
            for (int i = 0; i < 3; i++) {
                pos[i] = b[0] * v0[i] + b[1] * v1[i] + b[2] * v2[i];
            }
            sPts.emplace_back(pos, index);
        }
    }
    return sPts;
}

Vec2crd get_edge_indices(int edge_index, const stl_triangle_vertex_indices &triangle_indices)
{
    int next_edge_index = (edge_index == 2) ? 0 : edge_index + 1;
    coord_t vi0             = triangle_indices[edge_index];
    coord_t vi1             = triangle_indices[next_edge_index];
    return Vec2crd(vi0, vi1);
}

int get_vertex_index(size_t vertex_index, const stl_triangle_vertex_indices &triangle_indices) {
    if (vertex_index == triangle_indices[0]) return 0;
    if (vertex_index == triangle_indices[1]) return 1;
    if (vertex_index == triangle_indices[2]) return 2;
    return -1;
}

/*
std::vector<Vec3crd> ShapeDiameterFunction::create_neighbor(
    const std::vector<stl_triangle_vertex_indices> &indices, size_t vertices_size)
{
    if (indices.empty() || vertices_size == 0) return {};
    std::vector<std::vector<size_t>> vertex_triangles = create_vertex_faces_index(indices, vertices_size);
    // IMPROVE1: sort index of faces for faster search
    coord_t              no_value = -1;
    std::vector<Vec3crd> neighbors(indices.size(), Vec3crd(no_value, no_value, no_value));    

    for (const stl_triangle_vertex_indices& triangle_indices : indices) {
        coord_t index = &triangle_indices - &indices.front();
        Vec3crd& neighbor = neighbors[index];
        for (int edge_index = 0; edge_index < 3; ++edge_index) {            
            // check if done
            coord_t& neighbor_edge = neighbor[edge_index];
            if (neighbor_edge != no_value) continue;
            Vec2crd edge_indices = get_edge_indices(edge_index, triangle_indices);
            // IMPROVE: use same vector for 2 sides of triangle
            const std::vector<size_t> &faces = vertex_triangles[edge_indices[0]];
            for (const size_t &face : faces) {
                if (face <= index) continue;
                const stl_triangle_vertex_indices &face_indices = indices[face];                
                int vertex_index = get_vertex_index(edge_indices[1], face_indices);
                // NOT Contain second vertex?
                if (vertex_index < 0) continue;
                // Has NOT oposit direction?
                if (edge_indices[0] != face_indices[(vertex_index + 1) % 3]) continue;

                neighbor_edge = face;
                neighbors[face][vertex_index] = index;
                break;
            }
            // must be paired
            assert(neighbor_edge != no_value);
        }
    }
    return neighbors;
}*/

std::vector<Vec3crd> ShapeDiameterFunction::create_neighbor(
    const std::vector<stl_triangle_vertex_indices> &indices,
    size_t                                          vertices_size)
{
    if (indices.empty() || vertices_size == 0) return {};
    std::vector<std::vector<size_t>> vertex_triangles =
        create_vertex_faces_index(indices, vertices_size);
    // IMPROVE1: sort index of faces for faster search
    coord_t              no_value = -1;
    std::vector<Vec3crd> neighbors(indices.size(),
                                   Vec3crd(no_value, no_value, no_value));

    for (const stl_triangle_vertex_indices &triangle_indices : indices) {
        coord_t  index    = &triangle_indices - &indices.front();
        Vec3crd &neighbor = neighbors[index];
        for (int edge_index = 0; edge_index < 3; ++edge_index) {
            // check if done
            coord_t &neighbor_edge = neighbor[edge_index];
            if (neighbor_edge != no_value) continue;
            Vec2crd edge_indices = get_edge_indices(edge_index,
                                                    triangle_indices);
            // IMPROVE: use same vector for 2 sides of triangle
            const std::vector<size_t> &faces =
                vertex_triangles[edge_indices[0]];
            for (const size_t &face : faces) {
                if (face <= index) continue;
                const stl_triangle_vertex_indices &face_indices = indices[face];
                int vertex_index = get_vertex_index(edge_indices[1],
                                                    face_indices);
                // NOT Contain second vertex?
                if (vertex_index < 0) continue;
                // Has NOT oposit direction?
                if (edge_indices[0] != face_indices[(vertex_index + 1) % 3])
                    continue;

                neighbor_edge                 = face;
                neighbors[face][vertex_index] = index;
                break;
            }
            // must be paired
            assert(neighbor_edge != no_value);
        }
    }
    return neighbors;
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

#include <igl/qslim.h>
indexed_triangle_set ShapeDiameterFunction::subdivide(
    const indexed_triangle_set &its, float max_length)
{    
    using MapMatrixXfUnaligned = Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor | Eigen::DontAlign>>;
    using MapMatrixXiUnaligned = Eigen::Map<const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor | Eigen::DontAlign>>;
    Eigen::MatrixXd vertices = MapMatrixXfUnaligned(its.vertices.front().data(),
                             Eigen::Index(its.vertices.size()), 3).cast<double>();
    Eigen::MatrixXi indices = MapMatrixXiUnaligned(its.indices.front().data(),
                             Eigen::Index(its.indices.size()), 3);

    Eigen::MatrixXd verticesO;
    Eigen::MatrixXi indicesO;
    Eigen::VectorXi j, i;
    bool c = igl::qslim(vertices, indices, 1000, verticesO, indicesO, j,i);


    struct TriangleLengths
    {
        Vec3crd indices;
        Vec3f l; // lengths
        TriangleLengths(const Vec3crd &           indices,
                        const std::vector<Vec3f> &vertices): indices(indices)
        {
            const Vec3f &v0 = vertices[indices[0]];
            const Vec3f &v1 = vertices[indices[1]];
            const Vec3f &v2 = vertices[indices[2]];
            Vec3f e0 = v0 - v1;
            Vec3f e1 = v1 - v2;
            Vec3f e2 = v2 - v0;

            l[0] = e0.norm();
            l[1] = e1.norm();
            l[2] = e2.norm();
        }

        TriangleLengths(const Vec3crd &indices, const Vec3f &lengths)
            : indices(indices), l(lengths)
        {}

        int get_divide_index(float max_length) {
            if (l[0] > l[1] && l[0] > l[2]) {
                if (l[0] > max_length) return 0;
            } else if (l[1] > l[2]) {
                if (l[1] > max_length) return 1;
            } else {
                if (l[2] > max_length) return 2;
            }
            return -1;
        }

        float calc_divide_ratio(int divide_index, float max_length)
        {
            float  length = l[divide_index];
            int    count  = static_cast<int>(floor(length / max_length));
            if (count % 2 != 0) {
                return .5f;
            }

            float divider = static_cast<float>(count + 1);
            int half_count = count / 2;
            if (l[(divide_index + 1) % 3] > l[(divide_index + 2) % 3]) {
                return half_count / divider;
            } else {
                return (half_count + 1) / divider;
            }
        }

        // divide triangle add new vertex to vertices
        std::pair<TriangleLengths, TriangleLengths> divide(
            int divide_index, float max_length, std::vector<Vec3f> &vertices)
        {
            float ratio = calc_divide_ratio(divide_index, max_length);

            // index to lengths and indices
            size_t i0 = divide_index;
            size_t i1 = (divide_index + 1) % 3;
            size_t i2 = (divide_index + 2) % 3;

            size_t vi0   = indices[i0];
            size_t vi1   = indices[i1];
            size_t vi2   = indices[i2];

            const Vec3f &v0 = vertices[vi0];
            const Vec3f &v1 = vertices[vi1];

            Vec3f   dir        = v1 - v0;
            Vec3f   new_vertex = v0 + dir * ratio;
            coord_t new_index  = vertices.size();
            vertices.push_back(new_vertex);

            const Vec3f &v2 = vertices[vi2];
            Vec3f        new_edge = v2 - new_vertex;
            float        new_len  = new_edge.norm();
            float len1 = l[i0] * ratio;
            float len2 = l[i0] - len1;

            Vec3crd indices1(vi0, new_index, vi2);
            Vec3f lengths1(len1, new_len, l[i2]);
            
            Vec3crd indices2(new_index, vi1, vi2);
            Vec3f lengths2(len2, l[i1], new_len);

            return {TriangleLengths(indices1, lengths1),
                    TriangleLengths(indices2, lengths2)};
        }
    };
    indexed_triangle_set result;
    result.indices.reserve(its.indices.size());
    const std::vector<Vec3f> &vertices = its.vertices;
    result.vertices = vertices; // copy
    std::queue<TriangleLengths> tls;
    for (const Vec3crd &indices : its.indices) {
        TriangleLengths tl(indices, vertices);
        do {
            int divide_index = tl.get_divide_index(max_length);
            if (divide_index < 0) {
                // no dividing
                result.indices.push_back(tl.indices);
                if (tls.empty()) break;
                tl = tls.front(); // copy
                tls.pop();
            } else {
                auto [tl1, tl2] = tl.divide(divide_index, max_length,
                                            result.vertices);
                tl = tl1;
                tls.push(tl2);                
            }
        } while (true);
    }
    return result;
}


float ShapeDiameterFunction::min_triangle_side_length(
    const indexed_triangle_set &its)
{
    const Vec3crd &it = its.indices.front();
    const Vec3f &  v0 = its.vertices[it.x()];
    const Vec3f &  v1 = its.vertices[it.y()];
    float          min = (v0 - v1).norm();
    for (const Vec3crd &it : its.indices) {
        for (int i = 0; i < 3; ++i) { 
            int i2 = i + 1;
            if (i2 == 2) i2 = 0;
            const Vec3f &v0 = its.vertices[i];
            const Vec3f &v1 = its.vertices[i2];
            Vec3f edge = v0 - v1;
            if (fabs(edge.x()) > min) continue;
            if (fabs(edge.y()) > min) continue;
            float length = edge.norm();
            if (min > length) 
                min = length;
        }
    }
    return min;
}

float ShapeDiameterFunction::triangle_area(const Vec3f &v0,
                                           const Vec3f &v1,
                                           const Vec3f &v2)
{
    Vec3f ab = v1 - v0;
    Vec3f ac = v2 - v0;
    return ab.cross(ac).norm() / 2.f;
}

float ShapeDiameterFunction::triangle_area(const Vec3crd &triangle_inices,
                                  const std::vector<Vec3f>& vertices)
{
    return triangle_area(vertices[triangle_inices[0]],
                         vertices[triangle_inices[1]],
                         vertices[triangle_inices[2]]);
}


float ShapeDiameterFunction::area(const indexed_triangle_set &its)
{
    float sum_areas = 0;
    for (const Vec3crd &it : its.indices) {
        sum_areas += triangle_area(it, its.vertices);
    }
    return sum_areas;
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