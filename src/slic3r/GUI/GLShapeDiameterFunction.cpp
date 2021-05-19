#include "ShapeDiameterFunction.hpp"

#include "GUI_App.hpp"
#include "3DScene.hpp"

#include <GL/glew.h>

using namespace Slic3r;
using namespace Slic3r::GUI;

void ShapeDiameterFunction::draw() const
{
    if (!enabled) return;
    if (!initialized) return;

    // create vertex buffer with width
    GLShaderProgram *shader = wxGetApp().get_shader("sdf");
    assert(shader != nullptr);
    shader->start_using();
    shader->set_uniform("model_matrix", tr_mat);
    shader->set_uniform("min_width", min_value);
    shader->set_uniform("width_range", fabs(max_value - min_value));
    shader->set_uniform("draw_normals", draw_normals);

    unsigned int stride = Vertex::size();
    GLint position_id = shader->get_attrib_location("v_position");
    GLint normal_id   = shader->get_attrib_location("v_normal");
    GLint width_id    = shader->get_attrib_location("v_width");

    glsafe(::glBindBuffer(GL_ARRAY_BUFFER, m_vbo_id));
    if (position_id != -1) {
        glsafe(::glEnableVertexAttribArray(position_id));
        glsafe(::glVertexAttribPointer(position_id, 3, GL_FLOAT, GL_FALSE,
            stride, (GLvoid *) (intptr_t) Vertex::position_offset()));
    }
    if (normal_id != -1) {
        glsafe(::glEnableVertexAttribArray(normal_id));
        glsafe(::glVertexAttribPointer(normal_id, 3, GL_FLOAT,  GL_FALSE,
            stride, (GLvoid *) (intptr_t) Vertex::normal_offset()));
    }
    if (width_id != -1) {
        glsafe(::glEnableVertexAttribArray(width_id));
        glsafe(::glVertexAttribPointer(width_id, 3, GL_FLOAT, GL_FALSE,
            stride, (GLvoid *) (intptr_t) Vertex::width_offset()));
    }

    glsafe(::glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,m_vbo_indices_id));
    glsafe(::glDrawElements(GL_TRIANGLES, (GLsizei)indices_count, GL_UNSIGNED_INT, nullptr));
    glsafe(glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));

    // clear 
    if (position_id != -1) glsafe(::glDisableVertexAttribArray(position_id));
    if (normal_id != -1) glsafe(::glDisableVertexAttribArray(normal_id));
    if (width_id != -1) glsafe(::glDisableVertexAttribArray(width_id));
    glsafe(::glBindBuffer(GL_ARRAY_BUFFER, 0));

    shader->stop_using();    
}

// create points on unit sphere surface
ShapeDiameterFunction::Directions
ShapeDiameterFunction::create_fibonacci_sphere_samples(
    double angle, size_t count_samples)
{
    if (count_samples <= 1) {
        Direction d;
        d.dir = Vec3f(0.f, 0.f, 1.f);
        d.weight = 1.f;
        return {d};
    }
    assert(angle < 180);
    assert(angle > 1);
    double min_z = cos(angle / 2. * M_PI/180.);
    assert(min_z > 0.);

    Directions points; 
    points.reserve(count_samples);
    const double phi = M_PI * (3. - sqrt(5.)); // golden angle in radians
    for (size_t i = 0; i < count_samples; ++i) {
        double z = 1. - (i / double(count_samples - 1));
        if (z < min_z) break;
        double radius = sqrt(1. - z * z);  // radius at z
        double theta = phi * i; // golden angle increment
        double x = cos(theta) * radius;
        double y = sin(theta) * radius;
        Direction d;
        d.dir = Vec3f(
            static_cast<float>(x), 
            static_cast<float>(y),
            static_cast<float>(z));
        d.weight = d.dir.z();
        points.push_back(d);
    }
    //store(points);
    return points;
}

std::vector<Vec3f> create_triangle_normals(const indexed_triangle_set &its) {
    std::vector<Vec3f> normals;
    normals.reserve(its.indices.size());
    for (const Vec3crd &index : its.indices) { 
        const stl_vertex &v0 = its.vertices[index[0]];
        const stl_vertex &v1 = its.vertices[index[1]];
        const stl_vertex &v2 = its.vertices[index[2]];
        Vec3f direction = (v1 - v0).cross(v2 - v0);
        direction.normalize();
        normals.push_back(direction);
    }
    return normals;
}

// calculate normals by averaging normals of neghbor triangles
std::vector<Vec3f> ShapeDiameterFunction::create_normals(
    const indexed_triangle_set& its)
{
    size_t         count_vertices = its.vertices.size();    
    std::vector<Vec3f> triangle_normals = create_triangle_normals(its);
    assert(its.indices.size() == triangle_normals.size());
    std::vector<Vec3f> normals(count_vertices, Vec3f(.0, .0, .0));
    std::vector<unsigned char> count(count_vertices, 0);
    for (const Vec3crd &indice : its.indices) {
        size_t       index = &indice - &its.indices.front();
        const Vec3f &normal = triangle_normals[index];
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

bool ShapeDiameterFunction::initialize_model(const ModelObject *mo)
{
    initialized = false;
    if (mo == nullptr) return false;
    its = mo->raw_indexed_triangle_set(); // create new
    tr_mat = mo->instances.front()->get_matrix();
    normals = create_normals(its);
    tree = AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(
        its.vertices, its.indices);
    return initialize_width();
}

bool ShapeDiameterFunction::initialize_width() {
    const std::vector<Vec3f> &vertices = its.vertices;
    const Points3 &           indices  = its.indices;
    unit_z_rays = create_fibonacci_sphere_samples(angle, count_samples);
    std::vector<Vertex> buffer   = {};
    buffer.reserve(vertices.size());
    for (const Vec3f &vertex : vertices) {
        size_t       index  = &vertex - &vertices.front();
        const Vec3f &normal = normals[index];
        float        width  = calc_width(vertex, normal);
        buffer.emplace_back(vertex, normal, width);
    }

    // initialize triangle vertex data
    if (m_vbo_id != 0) glsafe(::glDeleteBuffers(1, &m_vbo_id));
    glsafe(::glGenBuffers(1, &m_vbo_id));
    glsafe(::glBindBuffer(GL_ARRAY_BUFFER, m_vbo_id));
    GLsizeiptr    size = buffer.size() * Vertex::size();
    const GLvoid *data = buffer.data();
    glsafe(::glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW));
    glsafe(::glBindBuffer(GL_ARRAY_BUFFER, 0));

    if (m_vbo_indices_id != 0)
        glsafe(::glDeleteBuffers(1, &m_vbo_indices_id));
    glsafe(::glGenBuffers(1, &m_vbo_indices_id));
    glsafe(::glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_indices_id));
    GLsizeiptr    indices_size = indices.size() * sizeof(Vec3crd);
    const GLvoid *indices_data = indices.data();
    glsafe(::glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_size, indices_data,
                          GL_STATIC_DRAW));
    glsafe(::glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));

    indices_count = indices.size() * 3;
    initialized   = true;
    return true;
}

float ShapeDiameterFunction::calc_width(const Vec3f &point,
                                        const Vec3f &normal)
{
    // safe against ray intersection with origin trinagle made by source vertex
    const double safe_move = 1e-5;
    // value for width when no intersection
    const float  no_width  = -1.;

    Vec3f ray_dir   = -normal;
    Vec3d ray_point = (point + ray_dir * safe_move).cast<double>();

    const Vec3f     z_axe(0.f, 0.f, 1.f);
    Vec3f       axis  = z_axe.cross(ray_dir);
    float       angle = std::acos(z_axe.dot(ray_dir));

    auto tr_mat = Eigen::AngleAxis<float>(angle, axis).matrix();
    float sum_width  = 0.f;
    float sum_weight = 0.f;
    for (const Direction &dir : unit_z_rays) { 
        const Vec3f &ray     = dir.dir;
        Vec3f ray_tr = tr_mat*ray;
        Vec3d    ray_trd = ray_tr.cast<double>();
        igl::Hit hit;
        if (AABBTreeIndirect::intersect_ray_first_hit(
            its.vertices, its.indices, tree, ray_point, ray_trd, hit)) {
            sum_width += hit.t * dir.weight;
            sum_weight += dir.weight;
        }
    }
    if (sum_weight <= 0.) return no_width;
    return sum_width / sum_weight;
}

#include <admesh/stl.h>
bool ShapeDiameterFunction::store(const Directions &unit_z_rays) {
    TriangleMesh tm;
    stl_file& stl=tm.stl;
    stl.facet_start.reserve(2*unit_z_rays.size());
    float triangle_size = 1e-1f;
    float triangle_length = 1.f + 2.f;
    for (const auto &dir : unit_z_rays) { 
        Vec3f     ray = dir.dir;
        stl_facet facet;
        facet.normal = Vec3f(0.f,1.f,0.f);
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
