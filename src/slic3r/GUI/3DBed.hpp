#ifndef slic3r_3DBed_hpp_
#define slic3r_3DBed_hpp_

#include "GLTexture.hpp"
#include "3DScene.hpp"
#include "GLShader.hpp"

class GLUquadric;
typedef class GLUquadric GLUquadricObj;

namespace Slic3r {
namespace GUI {

class GeometryBuffer
{
#if ENABLE_PRINTBED_SHADER
    struct Vertex
    {
        float position[3];
        float tex_coords[2];

        Vertex()
        {
            position[0] = 0.0f; position[1] = 0.0f; position[2] = 0.0f;
            tex_coords[0] = 0.0f; tex_coords[1] = 0.0f;
        }
    };

    std::vector<Vertex> m_vertices;
#else
    std::vector<float> m_vertices;
    std::vector<float> m_tex_coords;
#endif // ENABLE_PRINTBED_SHADER

public:
    bool set_from_triangles(const Polygons& triangles, float z, bool generate_tex_coords);
    bool set_from_lines(const Lines& lines, float z);

#if ENABLE_PRINTBED_SHADER
    const float* get_vertices_data() const;
#else
    const float* get_vertices() const { return m_vertices.data(); }
    const float* get_tex_coords() const { return m_tex_coords.data(); }
#endif // ENABLE_PRINTBED_SHADER

#if ENABLE_PRINTBED_SHADER
    unsigned int get_vertices_data_size() const { return (unsigned int)m_vertices.size() * get_vertex_data_size(); }
    unsigned int get_vertex_data_size() const { return (unsigned int)(5 * sizeof(float)); }
    unsigned int get_position_offset() const { return 0; }
    unsigned int get_tex_coords_offset() const { return (unsigned int)(3 * sizeof(float)); }
#endif // ENABLE_PRINTBED_SHADER
    unsigned int get_vertices_count() const;
};

class Bed3D
{
    struct Axes
    {
        static const double Radius;
        static const double ArrowBaseRadius;
        static const double ArrowLength;
        Vec3d origin;
        Vec3d length;
        GLUquadricObj* m_quadric;

        Axes();
        ~Axes();

        void render() const;

    private:
        void render_axis(double length) const;
    };

public:
    enum EType : unsigned char
    {
        MK2,
        MK3,
        SL1,
        Custom,
        Num_Types
    };

private:
    EType m_type;
    Pointfs m_shape;
    BoundingBoxf3 m_bounding_box;
    Polygon m_polygon;
    GeometryBuffer m_triangles;
    GeometryBuffer m_gridlines;
#if ENABLE_PRINTBED_SHADER
    mutable GLTexture m_texture;
    mutable Shader m_shader;
    mutable unsigned int m_vbo_id;
#elif !ENABLE_BED_MODEL_TEXTURE
    mutable GLTexture m_top_texture;
    mutable GLTexture m_bottom_texture;
#endif // ENABLE_PRINTBED_SHADER
    mutable GLStlModel m_model;
#if ENABLE_BED_MODEL_TEXTURE
    mutable GLStlModel m_texture_model;
#endif // ENABLE_BED_MODEL_TEXTURE
    Axes m_axes;

    mutable float m_scale_factor;

public:
    Bed3D();
#if ENABLE_PRINTBED_SHADER
    ~Bed3D();
#endif // ENABLE_PRINTBED_SHADER

    EType get_type() const { return m_type; }

    bool is_prusa() const { return (m_type == MK2) || (m_type == MK3) || (m_type == SL1); }
    bool is_custom() const { return m_type == Custom; }

    const Pointfs& get_shape() const { return m_shape; }
    // Return true if the bed shape changed, so the calee will update the UI.
    bool set_shape(const Pointfs& shape);

    const BoundingBoxf3& get_bounding_box() const { return m_bounding_box; }
    bool contains(const Point& point) const;
    Point point_projection(const Point& point) const;

    void render(float theta, bool useVBOs, float scale_factor) const;
    void render_axes() const;

private:
    void calc_bounding_box();
    void calc_triangles(const ExPolygon& poly);
    void calc_gridlines(const ExPolygon& poly, const BoundingBox& bed_bbox);
    EType detect_type(const Pointfs& shape) const;
#if ENABLE_PRINTBED_SHADER
    void render_prusa(const std::string& key, bool bottom) const;
    void render_prusa_shader(unsigned int vertices_count, bool transparent) const;
#else
    void render_prusa(const std::string& key, float theta, bool useVBOs) const;
#endif // ENABLE_PRINTBED_SHADER
    void render_custom() const;

#if ENABLE_PRINTBED_SHADER
    void reset();
#endif // ENABLE_PRINTBED_SHADER
};

} // GUI
} // Slic3r

#endif // slic3r_3DBed_hpp_
