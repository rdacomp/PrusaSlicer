#include "ShapeDiameterFunction.hpp"
#include <libslic3r/AABBTreeIndirect.hpp>

#include "GUI_App.hpp"
#include "3DScene.hpp"

#include <GL/glew.h>

using namespace Slic3r::GUI;

void ShapeDiameterFunction::set_enabled(bool enable){
	enabled = enable;
}

void ShapeDiameterFunction::draw() const {
    if (!enabled) return;
    if (!initialized) return;

    // create vertex buffer with width
    GLShaderProgram *shader = wxGetApp().get_shader("sdf");
    assert(shader != nullptr);

    unsigned int stride = Vertex::size();
    if (m_vbo_id == 0) {
        // first initialization        
        // TODO: remove const cast
        unsigned int *vbo_id = const_cast<unsigned int *>(&m_vbo_id);
        glsafe(::glGenBuffers(1, vbo_id));
        glsafe(::glBindBuffer(GL_ARRAY_BUFFER, m_vbo_id));
        GLsizeiptr    size = m_buffer.size() * stride;
        const GLvoid *data = m_buffer.data();
        glsafe(::glBufferData(GL_ARRAY_BUFFER, size, data, GL_STATIC_DRAW));
        glsafe(::glBindBuffer(GL_ARRAY_BUFFER, 0));
    }

    shader->start_using();
    //shader->set_uniform("width_range", range);

    glsafe(::glEnable(GL_DEPTH_TEST));
    glsafe(::glDepthMask(GL_FALSE));

    glsafe(::glEnable(GL_BLEND));
    glsafe(::glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));

    // glsafe(::glFrontFace(GL_CW));

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

    glsafe(::glDrawArrays(GL_TRIANGLES, 0, (GLsizei) m_buffer.size()));

    // clear 
    if (position_id != -1) glsafe(::glDisableVertexAttribArray(position_id));
    if (normal_id != -1) glsafe(::glDisableVertexAttribArray(normal_id));
    if (width_id != -1) glsafe(::glDisableVertexAttribArray(width_id));
    glsafe(::glBindBuffer(GL_ARRAY_BUFFER, 0));
    glsafe(::glBindTexture(GL_TEXTURE_2D, 0));

    // glsafe(::glFrontFace(GL_CCW));

    glsafe(::glDisable(GL_BLEND));
    glsafe(::glDepthMask(GL_TRUE));

    shader->stop_using();    
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
    const std::vector<stl_facet> &facets = stl.facet_start;

    // calculate average normals for vertex
    std::vector<Vec3d> normals(vertices.size(), Vec3d(.0, .0, .0));
    assert(indices.size() == facets.size());
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

    auto tree =
        AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(vertices,
                                                                    indices);

    float no_width = -1.;
    m_buffer.reserve(vertices.size());
    for (const Vec3f &vertex : vertices) {
        size_t       index  = &vertex - &vertices.front();
        const Vec3d &normal = normals[index];

        // SDF calculation of width
        Vec3d        ray_point = vertex.cast<double>();
        const Vec3d &ray_dir   = -normal;
        igl::Hit     hit;
        bool intersected = AABBTreeIndirect::intersect_ray_first_hit(vertices,
                                                                     indices,
                                                                     tree,
                                                                     ray_point,
                                                                     ray_dir,
                                                                     hit);
        float width      = (intersected) ? hit.t : no_width;
        m_buffer.emplace_back(vertex, normal.cast<float>(), width);
    }

    initialized = true;
    return true;
}
