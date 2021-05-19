#ifndef slic3r_GLShapeDiameterFunction_hpp_
#define slic3r_GLShapeDiameterFunction_hpp_

#include <libslic3r/Model.hpp>
#include <libslic3r/Point.hpp>
#include <libslic3r/AABBTreeIndirect.hpp>
#include <libslic3r/ShapeDiameterFunction.hpp>

namespace Slic3r::GUI {
// Shapira, Lior, Ariel Shamir, and Daniel Cohen-Or. 
// "Consistent mesh partitioning and skeletonisation using the shape diameter function." 
// The Visual Computer 24.4 (2008): 249-259.
class GLShapeDiameterFunction
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
    float angle = 120; // [in deg] in range from 1 to 179
    size_t count_samples = 1; // count samples on half sphere

    GLShapeDiameterFunction() = default; // set default values
    ~GLShapeDiameterFunction() = default; // needed by PIMPL in GLCanvas3D.hpp

    void set_enabled(bool enable) { enabled = enable; };
    bool is_enabled() const { return enabled; };
    size_t get_ray_count() const { return unit_z_rays.size(); }
    
    void draw() const;

    // create vertex shader data for draw
    bool initialize_model(const ModelObject *mo);
    // calculate width(tree and normals are already calculated)
    bool initialize_width();
private:
    bool initialize_indices();

    // structure uploaded to GPU
    struct Vertex
    {
        Vec3f position;
        Vec3f normal;
        float width;
        Vertex(Vec3f position, Vec3f normal, float width)
            : position(position), normal(normal), width(width)
        {}

        static unsigned int size(){ return sizeof(Vertex);}
        static size_t position_offset() {return size_t{0}; }
        static size_t normal_offset() { return (size_t)(sizeof(Vec3f)); }
        static size_t width_offset() { return (size_t)(2 * sizeof(Vec3f)); }
    };

    // draw information
    size_t indices_count = 0;
    Transform3d tr_mat;

    ShapeDiameterFunction::Directions unit_z_rays;
    // normals for each vertex of mesh
    indexed_triangle_set its;
    // same count as its::vertices - should be inside of index triangle set
    std::vector<Vec3f> normals; 

    AABBTreeIndirect::Tree3f tree;    
};

} // namespace Slic3r::GUI
#endif // slic3r_GLShapeDiameterFunction_hpp_