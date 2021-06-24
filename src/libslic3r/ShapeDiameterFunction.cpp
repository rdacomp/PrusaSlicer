#include "ShapeDiameterFunction.hpp"
#include <random>
#include <tbb/parallel_for.h>
#include "TriangleMesh.hpp"
#include "SimplifyMesh.hpp" // reduction by edge size

using namespace Slic3r;

float ShapeDiameterFunction::calc_width(const Vec3f &     point,
                                        const Vec3f &     normal,
                                        const AABBTree &  tree,
                                        const Config &  config)
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

    const Directions &dirs = config.dirs;
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

        if (config.is_angle_filtering()) {
            // check angle ray of hitted traingle
            Vec3f hit_normal = tree.triangle_normals[hit.id];
            float dot        = ray_dir.dot(hit_normal);
            if (dot < -1.f) dot = -1.f;
            if (dot > 1.f) dot = 1.f;
            float angle = std::acos(dot);
            // when angle between ray direction and hitted triangle normal
            // is more than 90deg that means the face is hitted from BAD side
            
            if (angle > config.allowed_angle) continue; 
            // face is propably inside of model or
            // ray fly throw edge of triangles - numeric issue
        }

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
    float threshold_deviation = standard_deviation * config.allowed_deviation;
    sum_width = 0.f;
    float sum_weight = 0.f;
    for (size_t i = 0; i < widths.size(); i++) {
        const float &width = widths[i];
        // skip values out of standart deviation
        if (config.is_deviation_filtering() && 
            fabs(width - mean) > threshold_deviation) continue;
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
    const AABBTree &          tree,
    const Config &config)
{
    // check input
    assert(!points.empty());
    assert(!config.dirs.empty());
    assert(points.size() == normals.size());
    if (points.empty() || config.dirs.empty() ||
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
            widths[index] = calc_width(vertex, normal, tree, config);
        }
    });
    return widths;
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

indexed_triangle_set ShapeDiameterFunction::subdivide(
    const indexed_triangle_set &its, float max_length)
{
    // same order as key order in Edge Divides
    struct VerticesSequence
    {
        size_t   start_index;
        bool     positive_order;
        VerticesSequence(size_t start_index, bool positive_order = true)
            : start_index(start_index), positive_order(positive_order){}
    };
    //                         vertex index small, big      vertex index from key.first to key.second
    using EdgeDivides = std::map<std::pair<size_t, size_t>, VerticesSequence>;
    struct Edges
    {
        Vec3f data[3];
        Vec3f lengths;
        Edges(const Vec3crd &indices, const std::vector<Vec3f> &vertices)
            : lengths(-1.f,-1.f,-1.f)
        {
            const Vec3f &v0 = vertices[indices[0]];
            const Vec3f &v1 = vertices[indices[1]];
            const Vec3f &v2 = vertices[indices[2]];
            data[0] = v0 - v1;
            data[1] = v1 - v2;
            data[2] = v2 - v0;
        }
        float abs_sum(const Vec3f &v)
        {
            return abs(v[0]) + abs(v[1]) + abs(v[2]);
        }
        bool is_dividable(const float& max_length) {
            Vec3f sum(abs_sum(data[0]), abs_sum(data[1]), abs_sum(data[2]));
            Vec3i biggest_index = (sum[0] > sum[1]) ?
                                      ((sum[0] > sum[2]) ?
                                           ((sum[2] > sum[1]) ?
                                                Vec3i(0, 2, 1) :
                                                Vec3i(0, 1, 2)) :
                                           Vec3i(2, 0, 1)) :
                                      ((sum[1] > sum[2]) ?
                                           ((sum[2] > sum[0]) ?
                                                Vec3i(1, 2, 0) :
                                                Vec3i(1, 0, 2)) :
                                           Vec3i(2, 1, 0));
            for (int i = 0; i < 3; i++) {
                int index = biggest_index[i];
                if (sum[index] <= max_length) return false;
                lengths[index] = data[index].norm();
                if (lengths[index] <= max_length) continue;

                // calculate rest of lengths
                for (int j = i + 1; j < 3; j++) {
                    index     = biggest_index[j];
                    lengths[index] = data[index].norm();
                }
                return true;
            }
            return false;
        }
    };
    struct TriangleLengths
    {
        Vec3crd indices;
        Vec3f l; // lengths
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

        // divide triangle add new vertex to vertices
        std::pair<TriangleLengths, TriangleLengths> divide(
            int divide_index, float max_length, 
            std::vector<Vec3f> &vertices,
            EdgeDivides &edge_divides)
        {
            // index to lengths and indices
            size_t i0 = divide_index;
            size_t i1 = (divide_index + 1) % 3;
            size_t vi0   = indices[i0];
            size_t vi1   = indices[i1];
            std::pair<size_t, size_t> key(vi0, vi1);
            bool key_swap = false;
            if (key.first > key.second) {
                std::swap(key.first, key.second);
                key_swap = true;
            }           

            float length = l[divide_index];
            size_t count_edge_vertices  = static_cast<size_t>(floor(length / max_length));
            float count_edge_segments = static_cast<float>(count_edge_vertices + 1);

            auto it = edge_divides.find(key);
            if (it == edge_divides.end()) {
                // Create new vertices
                VerticesSequence new_vs(vertices.size());
                Vec3f vf = vertices[key.first]; // copy
                const Vec3f &vs = vertices[key.second];
                Vec3f dir = vs - vf;
                for (size_t i = 1; i <= count_edge_vertices; ++i) { 
                    float ratio = i / count_edge_segments;
                    vertices.push_back(vf + dir * ratio);
                }
                bool     success;
                std::tie(it,success) = edge_divides.insert({key, new_vs});
                assert(success);
            }            
            const VerticesSequence &vs = it->second;

            int index_offset = count_edge_vertices/2;
            size_t i2 = (divide_index + 2) % 3;
            if (count_edge_vertices % 2 == 0 && key_swap == l[i1] < l[i2]) {
                --index_offset;
            }            
            int sign = (vs.positive_order) ? 1 : -1;
            size_t new_index = vs.start_index + sign*index_offset;
            
            size_t vi2   = indices[i2];
            const Vec3f &v2 = vertices[vi2];
            Vec3f        new_edge = v2 - vertices[new_index];
            float        new_len  = new_edge.norm();

            float ratio = (1 + index_offset) / count_edge_segments;
            float len1 = l[i0] * ratio;
            float len2 = l[i0] - len1;
            if (key_swap) std::swap(len1, len2);

            Vec3crd indices1(vi0, new_index, vi2);
            Vec3f lengths1(len1, new_len, l[i2]);
            
            Vec3crd indices2(new_index, vi1, vi2);
            Vec3f lengths2(len2, l[i1], new_len);

            // append key for divided edge when neccesary
            if (index_offset > 0) {
                std::pair<size_t, size_t> new_key(key.first, new_index);
                bool new_key_swap = false;
                if (new_key.first > new_key.second) {
                    std::swap(new_key.first, new_key.second);
                    new_key_swap = true;
                }
                if (edge_divides.find(new_key) == edge_divides.end()) {
                    // insert new
                    edge_divides.insert({new_key, (new_key_swap) ?
                        VerticesSequence(new_index - sign, !vs.positive_order)
                        : vs});
                }
            }

            if (index_offset < count_edge_vertices-1) {
                std::pair<size_t, size_t> new_key(new_index, key.second);
                bool new_key_swap = false;
                if (new_key.first > new_key.second) {
                    std::swap(new_key.first, new_key.second);
                    new_key_swap = true;
                }
                // bad order
                if (edge_divides.find(new_key) == edge_divides.end()) {
                    edge_divides.insert({new_key, (new_key_swap) ? 
                        VerticesSequence(vs.start_index + sign*(count_edge_vertices-1), !vs.positive_order)
                        : VerticesSequence(new_index + sign, vs.positive_order)});
                }
            }

            return {TriangleLengths(indices1, lengths1),
                    TriangleLengths(indices2, lengths2)};
        }
    };
    indexed_triangle_set result;
    result.indices.reserve(its.indices.size());
    const std::vector<Vec3f> &vertices = its.vertices;
    result.vertices = vertices; // copy
    std::queue<TriangleLengths> tls;
    
    EdgeDivides edge_divides;
    for (const Vec3crd &indices : its.indices) {
        Edges edges(indices, vertices);
        // speed up only sum not sqrt is apply
        if (!edges.is_dividable(max_length)) { 
             // small triangle
            result.indices.push_back(indices);
            continue; 
        }
        TriangleLengths tl(indices, edges.lengths);
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
                                            result.vertices, edge_divides);
                tl = tl1;
                tls.push(tl2);                
            }
        } while (true);
    }
    return result;
}

void ShapeDiameterFunction::connect_small_triangles(indexed_triangle_set &its, float min_length, float max_error)
{
    return remove_small_edges(its, min_length, max_error);
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