#include "ShapeDiameterFunction.hpp"
#include <libslic3r/AABBTreeIndirect.hpp>

#include "GUI_App.hpp"
#include "3DScene.hpp"

#include <GL/glew.h>

using namespace Slic3r;
using namespace Slic3r::GUI;

void ShapeDiameterFunction::set_enabled(bool enable){
	enabled = enable;
}

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

std::vector<Slic3r::Vec3d> create_fibonacci_sphere_samples(double angle, size_t count_samples)
{
    assert(angle < 180);
    assert(angle > 1);
    double last_z = sin(angle / 2. * M_PI/180.);
    assert(last_z > 0);

    std::vector<Slic3r::Vec3d> points; 
    points.reserve(count_samples);
    double phi = M_PI * (3. - sqrt(5.)); // golden angle in radians
    for (size_t i = 0; i < count_samples; ++i) {
        double z = 1. - (i / double(count_samples - 1));
        if (z < last_z) break;
        double radius = sqrt(1. - z * z);  // radius at z
        double theta = phi * i; // golden angle increment
        double x = cos(theta) * radius;
        double y = sin(theta) * radius;
        points.emplace_back(x, y, z);
    }
    return points;
}
// calculate normals by averaging normals of neghbor triangles
std::vector<Vec3d> create_normals(const Points3 &               indices,
                                  const std::vector<stl_facet> &facets,
                                  size_t count_vertices)
{
    assert(indices.size() == facets.size());
    std::vector<Vec3d> normals(count_vertices, Vec3d(.0, .0, .0));
    for (const Vec3crd &indice : indices) {
        size_t           index = &indice - &indices.front();
        const stl_facet &facet = facets[index];
        Vec3d n = facet.normal.cast<double>(); // should be normalized
        normals[indice.x()] += n;
        normals[indice.y()] += n;
        normals[indice.z()] += n;
    }
    // Improve: remember count and for normalization divide by count
    for (auto &normal : normals) normal.normalize();
    return normals;
}

bool ShapeDiameterFunction::initialize(const ModelObject *mo)
{
    initialized = false;
    if (mo == nullptr) return false;

    indexed_triangle_set its = mo->raw_indexed_triangle_set(); // create new
    const std::vector<Vec3f> &vertices = its.vertices;
    const Points3             indices  = its.indices;
    // dig normals
    TriangleMesh                  tmesh  = mo->mesh(); // create new
    const stl_file &              stl    = tmesh.stl;
    normals = create_normals(indices, stl.facet_start, vertices.size());
    //unit_z_rays = create_fibonacci_sphere_samples(angle, count_samples);

    auto tree =
        AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(vertices,
                                                                    indices);

    float no_width = -1.;
    std::vector<Vertex> buffer = {};
    buffer.reserve(vertices.size());
    for (const Vec3f &vertex : vertices) {
        size_t       index  = &vertex - &vertices.front();
        const Vec3d &normal = normals[index];

        // SDF calculation of width
        const Vec3d &ray_dir   = -normal;
        Vec3d        ray_point = vertex.cast<double>() + ray_dir * safe_move;
        igl::Hit     hit;
        bool intersected = AABBTreeIndirect::intersect_ray_first_hit(vertices,
                                                                     indices,
                                                                     tree,
                                                                     ray_point,
                                                                     ray_dir,
                                                                     hit);
        float width      = (intersected) ? hit.t : no_width;
        buffer.emplace_back(vertex, normal.cast<float>(), width);
    }
    
    // initialize triangle vertex data
    if (m_vbo_id != 0) glsafe(::glDeleteBuffers(1, &m_vbo_id));
    glsafe(::glGenBuffers(1, &m_vbo_id));
    glsafe(::glBindBuffer(GL_ARRAY_BUFFER, m_vbo_id));
    GLsizeiptr    size = buffer.size() * Vertex::size();
    const GLvoid *data = buffer.data();
    glsafe(::glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW));
    glsafe(::glBindBuffer(GL_ARRAY_BUFFER, 0));

    if (m_vbo_indices_id != 0) glsafe(::glDeleteBuffers(1, &m_vbo_indices_id));
    glsafe(::glGenBuffers(1, &m_vbo_indices_id));
    glsafe(::glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_indices_id));
    GLsizeiptr    indices_size = indices.size() * sizeof(Vec3crd);
    const GLvoid *indices_data = indices.data();
    glsafe(::glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_size, indices_data, GL_STATIC_DRAW));
    glsafe(::glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));

    tr_mat = mo->instances.front()->get_matrix();
    indices_count = indices.size() * 3;
    initialized = true;
    return true;
}
