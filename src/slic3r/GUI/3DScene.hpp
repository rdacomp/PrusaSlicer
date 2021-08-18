#ifndef slic3r_3DScene_hpp_
#define slic3r_3DScene_hpp_

#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/Geometry.hpp"

#if ENABLE_TEXTURED_VOLUMES
#include "GLTexture.hpp"
#endif // ENABLE_TEXTURED_VOLUMES
#if ENABLE_SINKING_CONTOURS
#include "GLModel.hpp"
#endif // ENABLE_SINKING_CONTOURS

#include <functional>

#define HAS_GLSAFE
#ifdef HAS_GLSAFE
    extern void glAssertRecentCallImpl(const char *file_name, unsigned int line, const char *function_name);
    inline void glAssertRecentCall() { glAssertRecentCallImpl(__FILE__, __LINE__, __FUNCTION__); }
    #define glsafe(cmd) do { cmd; glAssertRecentCallImpl(__FILE__, __LINE__, __FUNCTION__); } while (false)
    #define glcheck() do { glAssertRecentCallImpl(__FILE__, __LINE__, __FUNCTION__); } while (false)
#else // HAS_GLSAFE
    inline void glAssertRecentCall() { }
    #define glsafe(cmd) cmd
    #define glcheck()
#endif // HAS_GLSAFE

#if ENABLE_TEXTURED_VOLUMES
struct stl_facet;
#endif // ENABLE_TEXTURED_VOLUMES

namespace Slic3r {
class SLAPrintObject;
enum  SLAPrintObjectStep : unsigned int;
class DynamicPrintConfig;
class ExtrusionPath;
class ExtrusionMultiPath;
class ExtrusionLoop;
class ExtrusionEntity;
class ExtrusionEntityCollection;
class ModelObject;
class ModelVolume;
enum ModelInstanceEPrintVolumeState : unsigned char;

#if ENABLE_TEXTURED_VOLUMES
// A container for interleaved arrays of 3D vertices,
// Vertex data may contain: position (mandatory), normal (optional), uv coords (optional)
// possibly indexed by triangles and / or quads.
struct GLIndexedVertexArray {
    // Only Eigen types of Nx16 size are vectorized. This bounding box will not be vectorized.
    static_assert(sizeof(Eigen::AlignedBox<float, 3>) == 24, "Eigen::AlignedBox<float, 3> is not being vectorized, thus it does not need to be aligned");
    using BoundingBox = Eigen::AlignedBox<float, 3>;

    enum class EFormat {
        Position,
        PositionNormal,
        PositionUV,
        PositionNormalUV
    };

    // Format of the interleaved data
    // Need to be set before any call to push_geometry()
    EFormat format{ EFormat::PositionNormal };

    // vector of floats containing:
    // 3 floats for position, mandatory
    // 3 floats for normal, optional
    // 2 floats for uv, optional
    // the vector is cleared after data are loaded into the graphics card
    std::vector<float> interleaved_data;
    // vector of indices for triangle primitives
    // the vector is cleared after data are loaded into the graphics card
    std::vector<unsigned int> triangle_indices;
    // vector of indices for quad primitives
    // the vector is cleared after data are loaded into the graphics card
    std::vector<unsigned int> quad_indices;

    // When the geometry data is loaded into the graphics card as Vertex Buffer Objects,
    // the above mentioned std::vectors are cleared and the following variables are set,
    // to record their size.
    size_t interleaved_data_size{ 0 };
    size_t triangle_indices_size{ 0 };
    size_t quad_indices_size{ 0 };

    // IDs of the Vertex Buffer Objects, into which the geometry has been loaded.
    // Zero if the VBOs are not sent to GPU yet.
    unsigned int interleaved_data_VBO_id{ 0 };
    unsigned int triangle_indices_VBO_id{ 0 };
    unsigned int quad_indices_VBO_id{ 0 };

    GLIndexedVertexArray() { m_bounding_box.setEmpty(); }
    GLIndexedVertexArray(const GLIndexedVertexArray& rhs) :
        interleaved_data(rhs.interleaved_data),
        triangle_indices(rhs.triangle_indices),
        quad_indices(rhs.quad_indices),
        m_bounding_box(rhs.m_bounding_box),
        interleaved_data_VBO_id(0),
        triangle_indices_VBO_id(0),
        quad_indices_VBO_id(0)
    {
        assert(!rhs.has_VBOs());
    }
    GLIndexedVertexArray(GLIndexedVertexArray&& rhs) :
        interleaved_data(std::move(rhs.interleaved_data)),
        triangle_indices(std::move(rhs.triangle_indices)),
        quad_indices(std::move(rhs.quad_indices)),
        m_bounding_box(std::move(rhs.m_bounding_box)),
        interleaved_data_VBO_id(0),
        triangle_indices_VBO_id(0),
        quad_indices_VBO_id(0)
    {
        assert(!rhs.has_VBOs());
    }

    ~GLIndexedVertexArray() { release_VBOs(); }

    GLIndexedVertexArray& operator = (const GLIndexedVertexArray& rhs)
    {
        assert(!has_VBOs());
        assert(!rhs.has_VBOs());

        interleaved_data      = rhs.interleaved_data;
        triangle_indices      = rhs.triangle_indices;
        quad_indices          = rhs.quad_indices;
        m_bounding_box        = rhs.m_bounding_box;
        return *this;
    }

    GLIndexedVertexArray& operator = (GLIndexedVertexArray&& rhs)
    {
        assert(!has_VBOs());
        assert(!rhs.has_VBOs());

        interleaved_data      = std::move(rhs.interleaved_data);
        triangle_indices      = std::move(rhs.triangle_indices);
        quad_indices          = std::move(rhs.quad_indices);
        m_bounding_box        = rhs.m_bounding_box;
        return *this;
    }

    inline bool has_VBOs() const { return interleaved_data_VBO_id > 0 && (triangle_indices_VBO_id > 0 || quad_indices_VBO_id > 0); }
    inline const BoundingBox& bounding_box() const { return m_bounding_box; }

    // Is there any geometry data stored?
    inline bool empty() const { return interleaved_data.size() == 0 && interleaved_data_size == 0; }

#if ENABLE_SMOOTH_NORMALS
    void load_mesh_full_shading(const TriangleMesh& mesh, bool smooth_normals = false);
    void load_mesh(const TriangleMesh& mesh, bool smooth_normals = false) { load_mesh_full_shading(mesh, smooth_normals); }
#else
    void load_mesh_full_shading(const TriangleMesh& mesh);
    void load_mesh(const TriangleMesh& mesh) { load_mesh_full_shading(mesh); }
#endif // ENABLE_SMOOTH_NORMALS

    // push_geometry() parameters:
    // position is mandatory
    // normal is optional (required by PositionNormal and PositionNormalUV formats)
    // uv is optional (required by PositionUV and PositionNormalUV formats)
    // if optional values are required by the format, but are missing, dummy default values are added to the buffer
    // a more appropriate name would be push_vertex()
    void push_geometry(const Vec3f& position, const Vec3f* const normal = nullptr, const Vec2f* const uv = nullptr);
    void push_geometry(const stl_facet& facet);

    void push_triangle(unsigned int idx1, unsigned int idx2, unsigned int idx3);
    void push_quad(unsigned int idx1, unsigned int idx2, unsigned int idx3, unsigned int idx4);

    // Finalize the initialization of the geometry & indices,
    // upload the geometry and indices to OpenGL VBO objects
    // and shrink the allocated data, possibly relasing it if it has been loaded into the VBOs.
    void finalize_geometry(bool opengl_initialized);

    inline void reserve(size_t sz) {
        assert(!has_VBOs());

        interleaved_data.reserve(sz * vertex_size_floats());
        triangle_indices.reserve(sz * 3);
        quad_indices.reserve(sz * 4);
    }

    // Shrink the internal storage to tighly fit the data stored.
    inline void shrink_to_fit() {
        assert(!has_VBOs());

        interleaved_data.shrink_to_fit();
        triangle_indices.shrink_to_fit();
        quad_indices.shrink_to_fit();
    }

    // Clear all data
    void release_geometry() {
        release_VBOs();
        release_cpu_geometry();
    }

    // Clear vectors' data
    void release_cpu_geometry();

    // Return an estimate of the memory consumed by this class.
    size_t cpu_memory_used() const {
        return sizeof(*this) + interleaved_data.capacity() * sizeof(float) +
            (triangle_indices.capacity() + quad_indices.capacity()) * sizeof(unsigned int);
    }
    // Return an estimate of the memory held by GPU vertex buffers.
    size_t gpu_memory_used() const {
        size_t memsize = 0;
        if (interleaved_data_VBO_id > 0)
            memsize += interleaved_data_size * sizeof(float);
        if (triangle_indices_VBO_id > 0)
            memsize += triangle_indices_size * sizeof(unsigned int);
        if (quad_indices_VBO_id > 0)
            memsize += quad_indices_size * sizeof(unsigned int);
        return memsize;
    }
    size_t total_memory_used() const { return cpu_memory_used() + gpu_memory_used(); }

    void render() const;
    void render(const std::pair<size_t, size_t>& tverts_range, const std::pair<size_t, size_t>& qverts_range) const;

    bool has_normal() const { return format == EFormat::PositionNormal || format == EFormat::PositionNormalUV; }
    bool has_uv() const { return format == EFormat::PositionUV || format == EFormat::PositionNormalUV; }

    size_t vertex_size_floats() const;
    size_t vertex_size_bytes() const { return vertex_size_floats() * sizeof(float); }

    size_t normal_offset_floats() const;
    size_t normal_offset_bytes() const { return normal_offset_floats() * sizeof(float); }

    size_t uv_offset_floats() const;
    size_t uv_offset_bytes() const { return uv_offset_floats() * sizeof(float); }

private:
    BoundingBox m_bounding_box;

    // Release OpenGL VBOs.
    void release_VBOs();
};
#else
// A container for interleaved arrays of 3D vertices and normals,
// possibly indexed by triangles and / or quads.
class GLIndexedVertexArray {
public:
    // Only Eigen types of Nx16 size are vectorized. This bounding box will not be vectorized.
    static_assert(sizeof(Eigen::AlignedBox<float, 3>) == 24, "Eigen::AlignedBox<float, 3> is not being vectorized, thus it does not need to be aligned");
    using BoundingBox = Eigen::AlignedBox<float, 3>;

    GLIndexedVertexArray() { m_bounding_box.setEmpty(); }
    GLIndexedVertexArray(const GLIndexedVertexArray &rhs) :
        vertices_and_normals_interleaved(rhs.vertices_and_normals_interleaved),
        triangle_indices(rhs.triangle_indices),
        quad_indices(rhs.quad_indices),
        m_bounding_box(rhs.m_bounding_box)
        { assert(! rhs.has_VBOs()); m_bounding_box.setEmpty(); }
    GLIndexedVertexArray(GLIndexedVertexArray &&rhs) :
        vertices_and_normals_interleaved(std::move(rhs.vertices_and_normals_interleaved)),
        triangle_indices(std::move(rhs.triangle_indices)),
        quad_indices(std::move(rhs.quad_indices)),
        m_bounding_box(rhs.m_bounding_box)
        { assert(! rhs.has_VBOs()); }

    ~GLIndexedVertexArray() { release_geometry(); }

    GLIndexedVertexArray& operator=(const GLIndexedVertexArray &rhs)
    {
        assert(vertices_and_normals_interleaved_VBO_id == 0);
        assert(triangle_indices_VBO_id == 0);
        assert(quad_indices_VBO_id == 0);
        assert(rhs.vertices_and_normals_interleaved_VBO_id == 0);
        assert(rhs.triangle_indices_VBO_id == 0);
        assert(rhs.quad_indices_VBO_id == 0);
        vertices_and_normals_interleaved 	  = rhs.vertices_and_normals_interleaved;
        triangle_indices                 	  = rhs.triangle_indices;
        quad_indices                          = rhs.quad_indices;
        m_bounding_box                   	  = rhs.m_bounding_box;
        vertices_and_normals_interleaved_size = rhs.vertices_and_normals_interleaved_size;
        triangle_indices_size                 = rhs.triangle_indices_size;
        quad_indices_size                     = rhs.quad_indices_size;
        return *this;
    }

    GLIndexedVertexArray& operator=(GLIndexedVertexArray &&rhs) 
    {
        assert(vertices_and_normals_interleaved_VBO_id == 0);
        assert(triangle_indices_VBO_id == 0);
        assert(quad_indices_VBO_id == 0);
        assert(rhs.vertices_and_normals_interleaved_VBO_id == 0);
        assert(rhs.triangle_indices_VBO_id == 0);
        assert(rhs.quad_indices_VBO_id == 0);
        this->vertices_and_normals_interleaved 		 = std::move(rhs.vertices_and_normals_interleaved);
        this->triangle_indices                 		 = std::move(rhs.triangle_indices);
        this->quad_indices                     		 = std::move(rhs.quad_indices);
        this->m_bounding_box                   		 = rhs.m_bounding_box;
        this->vertices_and_normals_interleaved_size  = rhs.vertices_and_normals_interleaved_size;
        this->triangle_indices_size                  = rhs.triangle_indices_size;
        this->quad_indices_size                      = rhs.quad_indices_size;
        return *this;
    }

    // Vertices and their normals, interleaved
    std::vector<float> vertices_and_normals_interleaved;
    std::vector<int>   triangle_indices;
    std::vector<int>   quad_indices;

    // When the geometry data is loaded into the graphics card as Vertex Buffer Objects,
    // the above mentioned std::vectors are cleared and the following variables keep their original length.
    size_t vertices_and_normals_interleaved_size{ 0 };
    size_t triangle_indices_size{ 0 };
    size_t quad_indices_size{ 0 };

    // IDs of the Vertex Buffer Objects, into which the geometry has been loaded.
    // Zero if the VBOs are not sent to GPU yet.
    unsigned int vertices_and_normals_interleaved_VBO_id{ 0 };
    unsigned int triangle_indices_VBO_id{ 0 };
    unsigned int quad_indices_VBO_id{ 0 };

#if ENABLE_SMOOTH_NORMALS
    void load_mesh_full_shading(const TriangleMesh& mesh, bool smooth_normals = false);
    void load_mesh(const TriangleMesh& mesh, bool smooth_normals = false) { load_mesh_full_shading(mesh, smooth_normals); }
#else
    void load_mesh_full_shading(const TriangleMesh& mesh);
    void load_mesh(const TriangleMesh& mesh) { load_mesh_full_shading(mesh); }
#endif // ENABLE_SMOOTH_NORMALS

    inline bool has_VBOs() const { return vertices_and_normals_interleaved_VBO_id != 0; }

    inline void reserve(size_t sz) {
        vertices_and_normals_interleaved.reserve(sz * 6);
        triangle_indices.reserve(sz * 3);
        quad_indices.reserve(sz * 4);
    }

    inline void push_geometry(float x, float y, float z, float nx, float ny, float nz) {
        assert(vertices_and_normals_interleaved_VBO_id == 0);
        if (vertices_and_normals_interleaved_VBO_id != 0)
            return;

        if (vertices_and_normals_interleaved.size() + 6 > vertices_and_normals_interleaved.capacity())
            vertices_and_normals_interleaved.reserve(next_highest_power_of_2(vertices_and_normals_interleaved.size() + 6));
        vertices_and_normals_interleaved.emplace_back(nx);
        vertices_and_normals_interleaved.emplace_back(ny);
        vertices_and_normals_interleaved.emplace_back(nz);
        vertices_and_normals_interleaved.emplace_back(x);
        vertices_and_normals_interleaved.emplace_back(y);
        vertices_and_normals_interleaved.emplace_back(z);

        this->vertices_and_normals_interleaved_size = this->vertices_and_normals_interleaved.size();
        m_bounding_box.extend(Vec3f(x, y, z));
    };

    inline void push_geometry(double x, double y, double z, double nx, double ny, double nz) {
        push_geometry(float(x), float(y), float(z), float(nx), float(ny), float(nz));
    }

    template<typename Derived, typename Derived2>
    inline void push_geometry(const Eigen::MatrixBase<Derived>& p, const Eigen::MatrixBase<Derived2>& n) {
        push_geometry(float(p(0)), float(p(1)), float(p(2)), float(n(0)), float(n(1)), float(n(2)));
    }

    inline void push_triangle(int idx1, int idx2, int idx3) {
        assert(vertices_and_normals_interleaved_VBO_id == 0);
        if (vertices_and_normals_interleaved_VBO_id != 0)
            return;

        if (triangle_indices.size() + 3 > vertices_and_normals_interleaved.capacity())
            triangle_indices.reserve(next_highest_power_of_2(triangle_indices.size() + 3));
        triangle_indices.emplace_back(idx1);
        triangle_indices.emplace_back(idx2);
        triangle_indices.emplace_back(idx3);
        triangle_indices_size = triangle_indices.size();
    };

    inline void push_quad(int idx1, int idx2, int idx3, int idx4) {
        assert(vertices_and_normals_interleaved_VBO_id == 0);
        if (vertices_and_normals_interleaved_VBO_id != 0)
            return;

        if (quad_indices.size() + 4 > vertices_and_normals_interleaved.capacity())
            quad_indices.reserve(next_highest_power_of_2(quad_indices.size() + 4));
        quad_indices.emplace_back(idx1);
        quad_indices.emplace_back(idx2);
        quad_indices.emplace_back(idx3);
        quad_indices.emplace_back(idx4);
        quad_indices_size = quad_indices.size();
    };

    // Finalize the initialization of the geometry & indices,
    // upload the geometry and indices to OpenGL VBO objects
    // and shrink the allocated data, possibly relasing it if it has been loaded into the VBOs.
    void finalize_geometry(bool opengl_initialized);
    // Release the geometry data, release OpenGL VBOs.
    void release_geometry();

    void render() const;
    void render(const std::pair<size_t, size_t>& tverts_range, const std::pair<size_t, size_t>& qverts_range) const;

    // Is there any geometry data stored?
    bool empty() const { return vertices_and_normals_interleaved_size == 0; }

    void clear() {
        this->vertices_and_normals_interleaved.clear();
        this->triangle_indices.clear();
        this->quad_indices.clear();
        vertices_and_normals_interleaved_size = 0;
        triangle_indices_size = 0;
        quad_indices_size = 0;
        m_bounding_box.setEmpty();
    }

    // Shrink the internal storage to tighly fit the data stored.
    void shrink_to_fit() {
        vertices_and_normals_interleaved.shrink_to_fit();
        triangle_indices.shrink_to_fit();
        quad_indices.shrink_to_fit();
    }

    const BoundingBox& bounding_box() const { return m_bounding_box; }

    // Return an estimate of the memory consumed by this class.
    size_t cpu_memory_used() const { return sizeof(*this) + vertices_and_normals_interleaved.capacity() * sizeof(float) + triangle_indices.capacity() * sizeof(int) + quad_indices.capacity() * sizeof(int); }
    // Return an estimate of the memory held by GPU vertex buffers.
    size_t gpu_memory_used() const {
    	size_t memsize = 0;
    	if (vertices_and_normals_interleaved_VBO_id != 0)
    		memsize += vertices_and_normals_interleaved_size * 4;
    	if (triangle_indices_VBO_id != 0)
    		memsize += triangle_indices_size * 4;
    	if (quad_indices_VBO_id != 0)
    		memsize += quad_indices_size * 4;
    	return memsize;
    }
    size_t total_memory_used() const { return cpu_memory_used() + gpu_memory_used(); }

private:
    BoundingBox m_bounding_box;
};
#endif // ENABLE_TEXTURED_VOLUMES

class GLVolume {
public:
    static const std::array<float, 4> SELECTED_COLOR;
    static const std::array<float, 4> HOVER_SELECT_COLOR;
    static const std::array<float, 4> HOVER_DESELECT_COLOR;
    static const std::array<float, 4> OUTSIDE_COLOR;
    static const std::array<float, 4> SELECTED_OUTSIDE_COLOR;
    static const std::array<float, 4> DISABLED_COLOR;
    static const std::array<float, 4> SLA_SUPPORT_COLOR;
    static const std::array<float, 4> SLA_PAD_COLOR;
    static const std::array<float, 4> NEUTRAL_COLOR;
    static const std::array<std::array<float, 4>, 4> MODEL_COLOR;

    enum EHoverState : unsigned char
    {
        HS_None,
#if ENABLE_SINKING_CONTOURS
        HS_Hover,
#endif // ENABLE_SINKING_CONTOURS
        HS_Select,
        HS_Deselect
    };

    GLVolume(float r = 1.f, float g = 1.f, float b = 1.f, float a = 1.f);
    GLVolume(const std::array<float, 4>& rgba) : GLVolume(rgba[0], rgba[1], rgba[2], rgba[3]) {}

private:
    Geometry::Transformation m_instance_transformation;
    Geometry::Transformation m_volume_transformation;

    // Shift in z required by sla supports+pad
    double        m_sla_shift_z;
    // Bounding box of this volume, in unscaled coordinates.
    BoundingBoxf3 m_transformed_bounding_box;
    // Whether or not is needed to recalculate the transformed bounding box.
    bool          m_transformed_bounding_box_dirty;
    // Convex hull of the volume, if any.
    std::shared_ptr<const TriangleMesh> m_convex_hull;
    // Bounding box of this volume, in unscaled coordinates.
    BoundingBoxf3 m_transformed_convex_hull_bounding_box;
    // Whether or not is needed to recalculate the transformed convex hull bounding box.
    bool          m_transformed_convex_hull_bounding_box_dirty;

#if ENABLE_SINKING_CONTOURS
    class SinkingContours
    {
        static const float HalfWidth;
        GLVolume& m_parent;
        GUI::GLModel m_model;
        BoundingBoxf3 m_old_box;
        Vec3d m_shift{ Vec3d::Zero() };

    public:
        SinkingContours(GLVolume& volume) : m_parent(volume) {}
        void render();

    private:
        void update();
    };

    SinkingContours m_sinking_contours;
#endif // ENABLE_SINKING_CONTOURS

public:
    // Color of the triangles / quads held by this volume.
    std::array<float, 4> color;
    // Color used to render this volume.
    std::array<float, 4> render_color;

    struct CompositeID {
        CompositeID(int object_id, int volume_id, int instance_id) : object_id(object_id), volume_id(volume_id), instance_id(instance_id) {}
        CompositeID() : object_id(-1), volume_id(-1), instance_id(-1) {}
        // Object ID, which is equal to the index of the respective ModelObject in Model.objects array.
        int             object_id;
        // Volume ID, which is equal to the index of the respective ModelVolume in ModelObject.volumes array.
        // If negative, it is an index of a geometry produced by the PrintObject for the respective ModelObject,
        // and which has no associated ModelVolume in ModelObject.volumes. For example, SLA supports.
        // Volume with a negative volume_id cannot be picked independently, it will pick the associated instance.
        int             volume_id;
        // Instance ID, which is equal to the index of the respective ModelInstance in ModelObject.instances array.
        int             instance_id;
		bool operator==(const CompositeID &rhs) const { return object_id == rhs.object_id && volume_id == rhs.volume_id && instance_id == rhs.instance_id; }
		bool operator!=(const CompositeID &rhs) const { return ! (*this == rhs); }
		bool operator< (const CompositeID &rhs) const 
			{ return object_id < rhs.object_id || (object_id == rhs.object_id && (volume_id < rhs.volume_id || (volume_id == rhs.volume_id && instance_id < rhs.instance_id))); }
    };
    CompositeID         composite_id;
    // Fingerprint of the source geometry. For ModelVolumes, it is the ModelVolume::ID and ModelInstanceID, 
    // for generated volumes it is the timestamp generated by PrintState::invalidate() or PrintState::set_done(),
    // and the associated ModelInstanceID.
    // Valid geometry_id should always be positive.
    std::pair<size_t, size_t> geometry_id;
    // An ID containing the extruder ID (used to select color).
    int                 	extruder_id;

    // Various boolean flags.
    struct {
	    // Is this object selected?
	    bool                selected : 1;
	    // Is this object disabled from selection?
	    bool                disabled : 1;
	    // Is this object printable?
	    bool                printable : 1;
	    // Whether or not this volume is active for rendering
	    bool                is_active : 1;
	    // Whether or not to use this volume when applying zoom_to_volumes()
	    bool                zoom_to_volumes : 1;
	    // Wheter or not this volume is enabled for outside print volume detection in shader.
	    bool                shader_outside_printer_detection_enabled : 1;
	    // Wheter or not this volume is outside print volume.
	    bool                is_outside : 1;
	    // Wheter or not this volume has been generated from a modifier
	    bool                is_modifier : 1;
	    // Wheter or not this volume has been generated from the wipe tower
	    bool                is_wipe_tower : 1;
	    // Wheter or not this volume has been generated from an extrusion path
	    bool                is_extrusion_path : 1;
	    // Wheter or not to always render this volume using its own alpha 
	    bool                force_transparent : 1;
	    // Whether or not always use the volume's own color (not using SELECTED/HOVER/DISABLED/OUTSIDE)
	    bool                force_native_color : 1;
        // Whether or not render this volume in neutral
        bool                force_neutral_color : 1;
#if ENABLE_SINKING_CONTOURS
        // Whether or not to force rendering of sinking contours
        bool                force_sinking_contours : 1;
#endif // ENABLE_SINKING_CONTOURS
    };

    // Is mouse or rectangle selection over this object to select/deselect it ?
    EHoverState         	hover;

    // Interleaved triangles & normals with indexed triangles & quads.
    GLIndexedVertexArray        indexed_vertex_array;
    // Ranges of triangle and quad indices to be rendered.
    std::pair<size_t, size_t>   tverts_range;
    std::pair<size_t, size_t>   qverts_range;

    // If the qverts or tverts contain thick extrusions, then offsets keeps pointers of the starts
    // of the extrusions per layer.
    std::vector<coordf_t>       print_zs;
    // Offset into qverts & tverts, or offsets into indices stored into an OpenGL name_index_buffer.
    std::vector<size_t>         offsets;

#if ENABLE_TEXTURED_VOLUMES
    // texture name
    std::string                 texture;
#endif // ENABLE_TEXTURED_VOLUMES

    // Bounding box of this volume, in unscaled coordinates.
    BoundingBoxf3 bounding_box() const {
        BoundingBoxf3 out;
        if (!indexed_vertex_array.bounding_box().isEmpty()) {
            out.min = indexed_vertex_array.bounding_box().min().cast<double>();
            out.max = indexed_vertex_array.bounding_box().max().cast<double>();
            out.defined = true;
        };
        return out;
    }

    void set_render_color(float r, float g, float b, float a);
    void set_render_color(const std::array<float, 4>& rgba);
    // Sets render color in dependence of current state
    void set_render_color();
    // set color according to model volume
    void set_color_from_model_volume(const ModelVolume& model_volume);

    const Geometry::Transformation& get_instance_transformation() const { return m_instance_transformation; }
    void set_instance_transformation(const Geometry::Transformation& transformation) { m_instance_transformation = transformation; set_bounding_boxes_as_dirty(); }

    const Vec3d& get_instance_offset() const { return m_instance_transformation.get_offset(); }
    double get_instance_offset(Axis axis) const { return m_instance_transformation.get_offset(axis); }

    void set_instance_offset(const Vec3d& offset) { m_instance_transformation.set_offset(offset); set_bounding_boxes_as_dirty(); }
    void set_instance_offset(Axis axis, double offset) { m_instance_transformation.set_offset(axis, offset); set_bounding_boxes_as_dirty(); }

    const Vec3d& get_instance_rotation() const { return m_instance_transformation.get_rotation(); }
    double get_instance_rotation(Axis axis) const { return m_instance_transformation.get_rotation(axis); }

    void set_instance_rotation(const Vec3d& rotation) { m_instance_transformation.set_rotation(rotation); set_bounding_boxes_as_dirty(); }
    void set_instance_rotation(Axis axis, double rotation) { m_instance_transformation.set_rotation(axis, rotation); set_bounding_boxes_as_dirty(); }

    Vec3d get_instance_scaling_factor() const { return m_instance_transformation.get_scaling_factor(); }
    double get_instance_scaling_factor(Axis axis) const { return m_instance_transformation.get_scaling_factor(axis); }

    void set_instance_scaling_factor(const Vec3d& scaling_factor) { m_instance_transformation.set_scaling_factor(scaling_factor); set_bounding_boxes_as_dirty(); }
    void set_instance_scaling_factor(Axis axis, double scaling_factor) { m_instance_transformation.set_scaling_factor(axis, scaling_factor); set_bounding_boxes_as_dirty(); }

    const Vec3d& get_instance_mirror() const { return m_instance_transformation.get_mirror(); }
    double get_instance_mirror(Axis axis) const { return m_instance_transformation.get_mirror(axis); }

    void set_instance_mirror(const Vec3d& mirror) { m_instance_transformation.set_mirror(mirror); set_bounding_boxes_as_dirty(); }
    void set_instance_mirror(Axis axis, double mirror) { m_instance_transformation.set_mirror(axis, mirror); set_bounding_boxes_as_dirty(); }

    const Geometry::Transformation& get_volume_transformation() const { return m_volume_transformation; }
    void set_volume_transformation(const Geometry::Transformation& transformation) { m_volume_transformation = transformation; set_bounding_boxes_as_dirty(); }

    const Vec3d& get_volume_offset() const { return m_volume_transformation.get_offset(); }
    double get_volume_offset(Axis axis) const { return m_volume_transformation.get_offset(axis); }

    void set_volume_offset(const Vec3d& offset) { m_volume_transformation.set_offset(offset); set_bounding_boxes_as_dirty(); }
    void set_volume_offset(Axis axis, double offset) { m_volume_transformation.set_offset(axis, offset); set_bounding_boxes_as_dirty(); }

    const Vec3d& get_volume_rotation() const { return m_volume_transformation.get_rotation(); }
    double get_volume_rotation(Axis axis) const { return m_volume_transformation.get_rotation(axis); }

    void set_volume_rotation(const Vec3d& rotation) { m_volume_transformation.set_rotation(rotation); set_bounding_boxes_as_dirty(); }
    void set_volume_rotation(Axis axis, double rotation) { m_volume_transformation.set_rotation(axis, rotation); set_bounding_boxes_as_dirty(); }

    const Vec3d& get_volume_scaling_factor() const { return m_volume_transformation.get_scaling_factor(); }
    double get_volume_scaling_factor(Axis axis) const { return m_volume_transformation.get_scaling_factor(axis); }

    void set_volume_scaling_factor(const Vec3d& scaling_factor) { m_volume_transformation.set_scaling_factor(scaling_factor); set_bounding_boxes_as_dirty(); }
    void set_volume_scaling_factor(Axis axis, double scaling_factor) { m_volume_transformation.set_scaling_factor(axis, scaling_factor); set_bounding_boxes_as_dirty(); }

    const Vec3d& get_volume_mirror() const { return m_volume_transformation.get_mirror(); }
    double get_volume_mirror(Axis axis) const { return m_volume_transformation.get_mirror(axis); }

    void set_volume_mirror(const Vec3d& mirror) { m_volume_transformation.set_mirror(mirror); set_bounding_boxes_as_dirty(); }
    void set_volume_mirror(Axis axis, double mirror) { m_volume_transformation.set_mirror(axis, mirror); set_bounding_boxes_as_dirty(); }
     
    double get_sla_shift_z() const { return m_sla_shift_z; }
    void set_sla_shift_z(double z) { m_sla_shift_z = z; }

    void set_convex_hull(std::shared_ptr<const TriangleMesh> convex_hull) { m_convex_hull = std::move(convex_hull); }
    void set_convex_hull(const TriangleMesh &convex_hull) { m_convex_hull = std::make_shared<const TriangleMesh>(convex_hull); }
    void set_convex_hull(TriangleMesh &&convex_hull) { m_convex_hull = std::make_shared<const TriangleMesh>(std::move(convex_hull)); }

    int                 object_idx() const { return composite_id.object_id; }
    int                 volume_idx() const { return composite_id.volume_id; }
    int                 instance_idx() const { return composite_id.instance_id; }

    Transform3d         world_matrix() const;
    bool                is_left_handed() const;

    const BoundingBoxf3& transformed_bounding_box() const;
    // non-caching variant
    BoundingBoxf3        transformed_convex_hull_bounding_box(const Transform3d &trafo) const;
    // caching variant
    const BoundingBoxf3& transformed_convex_hull_bounding_box() const;
    // convex hull
    const TriangleMesh*  convex_hull() const { return m_convex_hull.get(); }

    bool                empty() const { return indexed_vertex_array.empty(); }

    void                set_range(double low, double high);

    void                render() const;

    void                finalize_geometry(bool opengl_initialized) { indexed_vertex_array.finalize_geometry(opengl_initialized); }
    void                release_geometry() { indexed_vertex_array.release_geometry(); }

    void                set_bounding_boxes_as_dirty() { m_transformed_bounding_box_dirty = true; m_transformed_convex_hull_bounding_box_dirty = true; }

    bool                is_sla_support() const;
    bool                is_sla_pad() const;

    bool                is_sinking() const;
    bool                is_below_printbed() const;
#if ENABLE_SINKING_CONTOURS
    void                render_sinking_contours();
#endif // ENABLE_SINKING_CONTOURS

    // Return an estimate of the memory consumed by this class.
    size_t 				cpu_memory_used() const { 
    	//FIXME what to do wih m_convex_hull?
    	return sizeof(*this) - sizeof(indexed_vertex_array) + indexed_vertex_array.cpu_memory_used() + print_zs.capacity() * sizeof(coordf_t) + offsets.capacity() * sizeof(size_t);
    }
    // Return an estimate of the memory held by GPU vertex buffers.
    size_t 				gpu_memory_used() const { return indexed_vertex_array.gpu_memory_used(); }
    size_t 				total_memory_used() const { return cpu_memory_used() + gpu_memory_used(); }
};

typedef std::vector<GLVolume*> GLVolumePtrs;
typedef std::pair<GLVolume*, std::pair<unsigned int, double>> GLVolumeWithIdAndZ;
typedef std::vector<GLVolumeWithIdAndZ> GLVolumeWithIdAndZList;

#if ENABLE_TEXTURED_VOLUMES
#define ENABLE_GLTEXTURES_MANAGER_DEBUG 0
class GLTexturesManager
{
    struct TexItem
    {
        std::string name;
        std::shared_ptr<GUI::GLTexture> texture;
    };

    std::vector<TexItem> m_textures;

public:
    void update_from_model(const Model& model);

    // return the gpu id of the texture
    unsigned int get_texture_id(const std::string& name) const;

#if ENABLE_GLTEXTURES_MANAGER_DEBUG
    void output_content() const;
#endif // ENABLE_GLTEXTURES_MANAGER_DEBUG
};
#endif // ENABLE_TEXTURED_VOLUMES

class GLVolumeCollection
{
public:
    enum class ERenderType : unsigned char
    {
        Opaque,
        Transparent,
        All
    };

private:
    // min and max vertex of the print box volume
    float m_print_box_min[3];
    float m_print_box_max[3];

    // z range for clipping in shaders
    float m_z_range[2];

    // plane coeffs for clipping in shaders
    float m_clipping_plane[4];

    struct Slope
    {
        // toggle for slope rendering 
        bool active{ false };
        float normal_z;
    };

    Slope m_slope;
    bool m_show_sinking_contours = false;

#if ENABLE_TEXTURED_VOLUMES
    mutable GLTexturesManager m_textures_manager;
#endif // ENABLE_TEXTURED_VOLUMES

public:
    GLVolumePtrs volumes;

    GLVolumeCollection() { set_default_slope_normal_z(); }
    ~GLVolumeCollection() { clear(); }

    std::vector<int> load_object(
        const ModelObject 		*model_object,
        int                      obj_idx,
        const std::vector<int>	&instance_idxs,
        const std::string 		&color_by,
        bool 					 opengl_initialized);

    int load_object_volume(
        const ModelObject *model_object,
        int                obj_idx,
        int                volume_idx,
        int                instance_idx,
        const std::string &color_by,
        bool 			   opengl_initialized);

    // Load SLA auxiliary GLVolumes (for support trees or pad).
    void load_object_auxiliary(
        const SLAPrintObject           *print_object,
        int                             obj_idx,
        // pairs of <instance_idx, print_instance_idx>
        const std::vector<std::pair<size_t, size_t>>& instances,
        SLAPrintObjectStep              milestone,
        // Timestamp of the last change of the milestone
        size_t                          timestamp,
        bool 			   				opengl_initialized);

    int load_wipe_tower_preview(
        int obj_idx, float pos_x, float pos_y, float width, float depth, float height, float rotation_angle, bool size_unknown, float brim_width, bool opengl_initialized);

    GLVolume* new_toolpath_volume(const std::array<float, 4>& rgba, size_t reserve_vbo_floats = 0);
    GLVolume* new_nontoolpath_volume(const std::array<float, 4>& rgba, size_t reserve_vbo_floats = 0);

#if ENABLE_TEXTURED_VOLUMES
    void update_textures_from_model(const Model& model) { m_textures_manager.update_from_model(model); }
    unsigned int get_texture_id(const std::string& name) const { return m_textures_manager.get_texture_id(name); }
#endif // ENABLE_TEXTURED_VOLUMES

    // Render the volumes by OpenGL.
    void render(ERenderType type, bool disable_cullface, const Transform3d& view_matrix, std::function<bool(const GLVolume&)> filter_func = std::function<bool(const GLVolume&)>()) const;

    // Finalize the initialization of the geometry & indices,
    // upload the geometry and indices to OpenGL VBO objects
    // and shrink the allocated data, possibly relasing it if it has been loaded into the VBOs.
    void finalize_geometry(bool opengl_initialized) { for (auto* v : volumes) v->finalize_geometry(opengl_initialized); }
    // Release the geometry data assigned to the volumes.
    // If OpenGL VBOs were allocated, an OpenGL context has to be active to release them.
    void release_geometry() { for (auto *v : volumes) v->release_geometry(); }
    // Clear the geometry
    void clear() { for (auto *v : volumes) delete v; volumes.clear(); }

    bool empty() const { return volumes.empty(); }
    void set_range(double low, double high) { for (GLVolume *vol : this->volumes) vol->set_range(low, high); }

    void set_print_box(float min_x, float min_y, float min_z, float max_x, float max_y, float max_z) {
        m_print_box_min[0] = min_x; m_print_box_min[1] = min_y; m_print_box_min[2] = min_z;
        m_print_box_max[0] = max_x; m_print_box_max[1] = max_y; m_print_box_max[2] = max_z;
    }

    void set_z_range(float min_z, float max_z) { m_z_range[0] = min_z; m_z_range[1] = max_z; }
    void set_clipping_plane(const double* coeffs) { m_clipping_plane[0] = coeffs[0]; m_clipping_plane[1] = coeffs[1]; m_clipping_plane[2] = coeffs[2]; m_clipping_plane[3] = coeffs[3]; }

    bool is_slope_active() const { return m_slope.active; }
    void set_slope_active(bool active) { m_slope.active = active; }

    float get_slope_normal_z() const { return m_slope.normal_z; }
    void set_slope_normal_z(float normal_z) { m_slope.normal_z = normal_z; }
    void set_default_slope_normal_z() { m_slope.normal_z = -::cos(Geometry::deg2rad(90.0f - 45.0f)); }
    void set_show_sinking_contours(bool show) { m_show_sinking_contours = show; }

    // returns true if all the volumes are completely contained in the print volume
    // returns the containment state in the given out_state, if non-null
    bool check_outside_state(const DynamicPrintConfig* config, ModelInstanceEPrintVolumeState* out_state) const;
    bool check_outside_state(const DynamicPrintConfig* config, bool& partlyOut, bool& fullyOut) const;
    void reset_outside_state();

    void update_colors_by_extruder(const DynamicPrintConfig* config);

    // Returns a vector containing the sorted list of all the print_zs of the volumes contained in this collection
    std::vector<double> get_current_print_zs(bool active_only) const;

    // Return an estimate of the memory consumed by this class.
    size_t 				cpu_memory_used() const;
    // Return an estimate of the memory held by GPU vertex buffers.
    size_t 				gpu_memory_used() const;
    size_t 				total_memory_used() const { return this->cpu_memory_used() + this->gpu_memory_used(); }
    // Return CPU, GPU and total memory log line.
    std::string         log_memory_info() const;

private:
    GLVolumeCollection(const GLVolumeCollection &other);
    GLVolumeCollection& operator=(const GLVolumeCollection &);
};

GLVolumeWithIdAndZList volumes_to_render(const GLVolumePtrs& volumes, GLVolumeCollection::ERenderType type, const Transform3d& view_matrix, std::function<bool(const GLVolume&)> filter_func = nullptr);

struct _3DScene
{
    static void thick_lines_to_verts(const Lines& lines, const std::vector<double>& widths, const std::vector<double>& heights, bool closed, double top_z, GLVolume& volume);
    static void thick_lines_to_verts(const Lines3& lines, const std::vector<double>& widths, const std::vector<double>& heights, bool closed, GLVolume& volume);
	static void extrusionentity_to_verts(const Polyline &polyline, float width, float height, float print_z, GLVolume& volume);
    static void extrusionentity_to_verts(const ExtrusionPath& extrusion_path, float print_z, GLVolume& volume);
    static void extrusionentity_to_verts(const ExtrusionPath& extrusion_path, float print_z, const Point& copy, GLVolume& volume);
    static void extrusionentity_to_verts(const ExtrusionLoop& extrusion_loop, float print_z, const Point& copy, GLVolume& volume);
    static void extrusionentity_to_verts(const ExtrusionMultiPath& extrusion_multi_path, float print_z, const Point& copy, GLVolume& volume);
    static void extrusionentity_to_verts(const ExtrusionEntityCollection& extrusion_entity_collection, float print_z, const Point& copy, GLVolume& volume);
    static void extrusionentity_to_verts(const ExtrusionEntity* extrusion_entity, float print_z, const Point& copy, GLVolume& volume);
    static void polyline3_to_verts(const Polyline3& polyline, double width, double height, GLVolume& volume);
    static void point3_to_verts(const Vec3crd& point, double width, double height, GLVolume& volume);
};

static constexpr float BedEpsilon = 3.f * float(EPSILON);

}

#endif
