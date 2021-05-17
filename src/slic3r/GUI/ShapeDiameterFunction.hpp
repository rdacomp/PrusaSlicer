#ifndef slic3r_ShapeDiameterFunction_hpp_
#define slic3r_ShapeDiameterFunction_hpp_

#include <libslic3r/Model.hpp>
#include <libslic3r/Point.hpp>

namespace Slic3r::GUI {

class ShapeDiameterFunction
{
    bool         is_calculated = false;
    bool         enabled       = false;
    bool         initialized   = false;
    unsigned int m_vbo_id      = 0;
    unsigned int m_vbo_indices_id = 0;

public:
    float min_value = 0.1f;
    float max_value = 10.f;
    bool draw_normals = false;
    double safe_move = 1e-5; // safe against ray intersection with origin trinagle made by source vertex
    double angle = 120; // [in deg] in range from 1 to 179
    size_t count_samples = 30; // count samples on half sphere

    ShapeDiameterFunction() = default;

    void set_enabled(bool enable);
    bool is_enabled() const { return enabled; };
    
    void draw() const;

    // create_buffer
    bool initialize(const ModelObject *mo);
private:
    struct Vertex
    {
        Vec3f position;
        Vec3f normal;
        float width;
        Vertex(Vec3f position, Vec3f normal, float width)
            : position(position), normal(normal), width(width)
        {}

        static unsigned int size(){ // sizeof(Vertex)
            return  3 * sizeof(float) +     // vertex coordinate
                    3 * sizeof(float) + // normal coordinate
                    sizeof(float);      // object width
        }

        static size_t position_offset() {return size_t{0}; }
        static size_t normal_offset() { return (size_t)(sizeof(Vec3f)); }
        static size_t width_offset() { return (size_t)(2 * sizeof(Vec3f)); }
    };

    // draw information
    size_t indices_count;
    Transform3d tr_mat;

    // rays into z direction
    // for calculation SDF function
    std::vector<Slic3r::Vec3d> unit_z_rays;
    // normals for each vertex of mesh
    std::vector<Vec3d> normals;
};

} // namespace Slic3r::GUI
#endif // slic3r_ShapeDiameterFunction_hpp_