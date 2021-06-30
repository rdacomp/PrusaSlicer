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

    GLShaderProgram *shader = wxGetApp().get_shader("sdf");
    assert(shader != nullptr);
    shader->start_using();
    shader->set_uniform("min_width", sample_config.min_width);
    shader->set_uniform("width_range", fabs(sample_config.max_width - sample_config.min_width));
    shader->set_uniform("draw_normals", allow_render_normals);
    shader->set_uniform("normal_z_max", sample_config.normal_z_max);

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
    if (allow_render_normals) render_normals();
    else if (allow_render_points) render_points();
    render_rays();
}

bool GLShapeDiameterFunction::initialize_model(const ModelObject *mo)
{
    initialized = false;
    if (mo == nullptr) return false;
    indexed_triangle_set its = mo->raw_indexed_triangle_set(); // create new
    
    Transform3d tr_mat = mo->instances.front()->get_matrix();
    // Transform mesh - scale and scew could change width on model
    for (auto &vertex : its.vertices) {
        vertex = (tr_mat*vertex.cast<double>()).cast<float>();
    }

    if (allow_remesh)
        ShapeDiameterFunction::connect_small_triangles(its, min_triangle_size, max_thr);

    // create tree
    tree.vertices_indices = its; // copy
    tree.triangle_normals = NormalUtils::create_triangle_normals(its);
    tree.tree = AABBTreeIndirect::build_aabb_tree_over_indexed_triangle_set(
        its.vertices, its.indices);
    return divide();
}

bool GLShapeDiameterFunction::divide() {
    const indexed_triangle_set& its = tree.vertices_indices;
    if (!allow_divide_triangle) {
        triangles.indices  = its.indices;  // copy
        triangles.vertices = its.vertices; // copy
    } else {
        indexed_triangle_set divided =
            ShapeDiameterFunction::subdivide(its, max_triangle_size);
        triangles.indices  = divided.indices;  // copy
        triangles.vertices = divided.vertices; // copy
    }
    if(!initialize_indices()) return false;
    return initialize_normals();
}

bool GLShapeDiameterFunction::initialize_normals()
{ 
    triangles.vertex_normals  = NormalUtils::create_normals(triangles, normal_type);
    return initialize_width();
}

bool GLShapeDiameterFunction::initialize_width() {
    sdf_config.dirs = ShapeDiameterFunction::create_fibonacci_sphere_samples(angle, count_samples);
    widths = ShapeDiameterFunction::calc_widths(triangles.vertices, triangles.vertex_normals, tree, sdf_config);

    // merge vertices normal and width together for GPU
    std::vector<Vertex> buffer = {};
    const std::vector<Vec3f> &vertices = triangles.vertices;
    buffer.reserve(vertices.size());
    for (const Vec3f &vertex : vertices) {
        size_t       index  = &vertex - &vertices.front();
        const Vec3f &normal = triangles.vertex_normals[index];
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
    if (allow_render_points) sample_surface();
    return true;
}


#include <libslic3r/PointGrid3D.hpp>
void poisson_sphere_from_samples(
    GLShapeDiameterFunction::PointRadiuses & samples,
    const PointGrid3D &grid)
{
    // first fill place with bigger needs to support than rest
    std::sort(samples.begin(), samples.end(),
              [](const GLShapeDiameterFunction::PointRadius &lhs,
                 const GLShapeDiameterFunction::PointRadius &rhs){
                  return lhs.radius < rhs.radius;
    });
    GLShapeDiameterFunction::PointRadiuses result;
    result.reserve(samples.size());
    float max_r = samples.back().radius;
    Vec3f cell_size(max_r, max_r, max_r);
    PointGrid3D actGrid(cell_size);
    for (const auto &sample : samples) { 
        float r = sample.radius;
        if (actGrid.collides_with(sample.point, r)) continue;
        if (grid.collides_with(sample.point, r)) continue;
        actGrid.insert(sample.point);
        result.emplace_back(sample);
    }
    samples = result;
}

void GLShapeDiameterFunction::sample_surface() {
    std::mt19937       random_generator;
    std::random_device rd;
    random_generator.seed(rd());
    points.clear();
    points.reserve(triangles.vertices.size());
    
    // SupportPointGenerator::process
    float c = sample_config.max_radius; // cell size
    PointGrid3D grid(Vec3f(c, c, c)); // other support points

    std::function<bool(Vec3f, float)> add_point = [&](Vec3f point, float alone_radius){
        points.emplace_back(point, alone_radius);
        return true;
    };
    ShapeDiameterFunction::generate_support_points(
        triangles, widths, add_point, sample_config, random_generator);

    count_generated_points = points.size();
    poisson_sphere_from_samples(points, grid);
}

bool GLShapeDiameterFunction::initialize_indices()
{
    if (m_vbo_indices_id != 0) glsafe(::glDeleteBuffers(1, &m_vbo_indices_id));
    glsafe(::glGenBuffers(1, &m_vbo_indices_id));
    glsafe(::glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_vbo_indices_id));    
    const Points3 &indices      = triangles.indices; 
    GLsizeiptr    indices_size = indices.size() * sizeof(Vec3crd);
    const GLvoid *indices_data = indices.data();
    glsafe(::glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_size, indices_data, GL_STATIC_DRAW));
    glsafe(::glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0));
    indices_count = indices.size() * 3;
    return true;
}

#include "GLModel.hpp"
GLModel create_arrow(float width, float length) {
    indexed_triangle_set its;
    float w_2 = width / 2.f;
    its.vertices = {
        Vec3f(0.f, 0.f, length),
        Vec3f(w_2, 0.f, 0.f),
        Vec3f(-w_2, 0.f, 0.f),
        Vec3f(0.f, w_2, 0.f),
        Vec3f(0.f, -w_2, 0.f)
    };
    its.indices = {
        Vec3crd(0, 1, 3), 
        Vec3crd(0, 3, 2),
        Vec3crd(0, 2, 4),
        Vec3crd(0, 4, 1)
    };
    TriangleMesh tm(its);
    GLModel model;
    model.init_from(tm);
    return model;
}

void GLShapeDiameterFunction::render_normals() const {
    GLModel arrow_to_z = create_arrow(normal_width, normal_length);
    auto render_normal = [&](const Transform3f &transform) {
        glsafe(::glPushMatrix());
        glsafe(::glMultMatrixf(transform.data()));
        arrow_to_z.render();
        glsafe(::glPopMatrix());
    };

    GLShaderProgram *shader = wxGetApp().get_shader("gouraud_light");
    if (shader == nullptr) return;
    // normal color
    std::array<float, 4> color = {.2f, .8f, .2f, 1.f}; 
    shader->start_using();
    shader->set_uniform("uniform_color", color);
    Vec3f z_one(0.f, 0.f, 1.f);
    for (size_t index = 0; index < triangles.vertices.size(); ++index) {
        const Vec3f &vertex = triangles.vertices[index];
        const Vec3f &normal = triangles.vertex_normals[index];

        Vec3f        axis  = z_one.cross(normal);
        float        angle = acos(z_one.dot(normal));
        Transform3f  tr(
            Eigen::Translation<float,3>(vertex) *
            Eigen::AngleAxis<float>(angle, axis)
        );
        render_normal(tr);
    }
    shader->stop_using();
}

GLModel create_tetrahedron(float size) {
    indexed_triangle_set its;
    float s_2 = size / 2.f;
    its.vertices = {
        Vec3f(0.f, 0.f, size), 
        Vec3f(-s_2, -s_2, 0.f), 
        Vec3f(s_2, -s_2, 0.f),
        Vec3f(0.f, s_2, 0.f)
    };
    its.indices = {
        Vec3crd(0, 1, 2), 
        Vec3crd(0, 2, 3), 
        Vec3crd(0, 3, 1),
        Vec3crd(3, 2, 1)
    };
    TriangleMesh tm(its);
    GLModel      model;
    model.init_from(tm);
    return model;
}

void GLShapeDiameterFunction::render_points() const
{
    double radius = 1.0;
    indexed_triangle_set its = its_make_sphere(radius, 0.2);
    TriangleMesh tm(its);
    GLModel m; m.init_from(tm);
    //GLModel m = create_tetrahedron(1.f);
    GLShaderProgram *shader = wxGetApp().get_shader("gouraud_light");
    if (shader == nullptr) return;
    std::array<float, 4> color = {.2f, .2f, .8f, 1.f};
    shader->start_using();
    shader->set_uniform("uniform_color", color);
    for (const PointRadius &pr : points) {
        const Vec3f &point = pr.point;
        float        scale = pr.radius / 2.;
        glsafe(::glPushMatrix());
        glsafe(::glTranslatef(point.x(), point.y(), point.z()));
        glsafe(::glScalef(scale, scale, scale));
        m.render();
        glsafe(::glPopMatrix());
    }
    shader->stop_using();
}

void GLShapeDiameterFunction::render_rays() const {
    float   radius        = 5.f;
    GLShaderProgram *shader = wxGetApp().get_shader("gouraud_light");
    if (shader == nullptr) return;
    // normal color
    std::array<float, 4> color = {.8f, .8f, .8f, 1.f};
    shader->start_using();
    shader->set_uniform("uniform_color", color);
    Vec3f z_one(0.f, 0.f, 1.f);
    for (const auto& ray: sdf_config.dirs) {
        GLModel arrow_to_z = create_arrow(1.f, 10.f * ray.weight);
        const Vec3f &normal = ray.dir;
        Vec3f       axis  = z_one.cross(normal);
        float       angle = acos(z_one.dot(normal));
        Transform3f tr(Eigen::Translation<float, 3>(normal*radius) *
                       Eigen::AngleAxis<float>(angle, axis));
        glsafe(::glPushMatrix());
        glsafe(::glMultMatrixf(tr.data()));
        arrow_to_z.render();
        glsafe(::glPopMatrix());
    }
    shader->stop_using();
}