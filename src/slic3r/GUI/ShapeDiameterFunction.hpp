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

        static size_t position_offset() { return 0; }
        static size_t normal_offset() { return (size_t)(3 * sizeof(float)); }
        static size_t width_offset() { return (size_t)(4 * sizeof(float)); }
    };
    using Buffer = std::vector<Vertex>;
    Buffer m_buffer = {};
    size_t triangle_count;
};

} // namespace Slic3r::GUI
#endif // slic3r_ShapeDiameterFunction_hpp_