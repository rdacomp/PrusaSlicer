#include "GLShapeDiameterFunction.hpp"

#include "GUI_App.hpp" // for wxGetApp() -> hold shaders
#include "3DScene.hpp" // glsafe
#include <GL/glew.h>   // ::glFunctions

using namespace Slic3r;
using namespace Slic3r::GUI;

void GLShapeDiameterFunction::draw() const
{
    if (!enabled) return;
    if (!initialized) return;

    // create vertex buffer with width
    GLShaderProgram *shader = wxGetApp().get_shader("sdf");
    assert(shader != nullptr);
    shader->start_using();
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

bool GLShapeDiameterFunction::initialize_model(const ModelObject *mo)
{
    initialized = false;
    if (mo == nullptr) return false;
    its = mo->raw_indexed_triangle_set(); // create new
    if(!initialize_indices()) return false;
    
    // Transform mesh - scale and scew could change width on model
    Transform3d tr_mat = mo->instances.front()->get_matrix();
    for (auto &vertex : its.vertices) {
        vertex = (tr_mat*vertex.cast<double>()).cast<float>();
    }

    tree = AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(
        its.vertices, its.indices);
    return initialize_normals();
}

bool GLShapeDiameterFunction::initialize_normals()
{ 
    normals = NormalUtils::create_normals(its, normal_type);
    return initialize_width();
}

bool GLShapeDiameterFunction::initialize_width() {
    const std::vector<Vec3f> &vertices = its.vertices;
    unit_z_rays = ShapeDiameterFunction::create_fibonacci_sphere_samples(angle, count_samples);
    std::vector<float> widths = ShapeDiameterFunction::calc_widths(
        unit_z_rays, its, normals, tree);

    // merge vertices normal and width together for GPU
    std::vector<Vertex> buffer = {};
    buffer.reserve(vertices.size());
    for (const Vec3f &vertex : vertices) {
        size_t       index  = &vertex - &vertices.front();
        const Vec3f &normal = normals[index];
        float        width  = widths[index];
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
    
    initialized = true;
    return true;
}

bool GLShapeDiameterFunction::initialize_indices()
{
    if (m_vbo_indices_id != 0) glsafe(::glDeleteBuffers(1, &m_vbo_indices_id));
    glsafe(::glGenBuffers(1, &m_vbo_indices_id));
    glsafe(::glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_indices_id));    
    const Points3 &indices     = its.indices; 
    GLsizeiptr    indices_size = indices.size() * sizeof(Vec3crd);
    const GLvoid *indices_data = indices.data();
    glsafe(::glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_size, indices_data, GL_STATIC_DRAW));
    glsafe(::glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
    indices_count = indices.size() * 3;
    return true;
}

void GLShapeDiameterFunction::render_normals() {

}
