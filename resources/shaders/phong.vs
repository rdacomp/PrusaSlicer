#version 110

const vec3 ZERO = vec3(0.0, 0.0, 0.0);

struct PrintBoxDetection
{
    bool active;
    vec3 min;
    vec3 max;
    mat4 volume_world_matrix;
};

struct SlopeDetection
{
    bool active;
	float normal_z;
    mat3 volume_world_normal_matrix;
};

struct BoundingBox
{
    vec3 center;
    vec3 sizes;
};

struct ClippingPlane
{
    bool active;
    // Clipping plane, x = min z, y = max z. Used by the FFF and SLA previews to clip with a top / bottom plane.
    vec2 z_range;
    // Clipping plane - general orientation. Used by the SLA gizmo.
    vec4 plane;
};

uniform PrintBoxDetection print_box;
uniform SlopeDetection slope;
uniform ClippingPlane clipping_plane;

varying vec3 delta_box_min;
varying vec3 delta_box_max;

varying vec3 clipping_planes_dots;

varying vec3 model_pos;
varying vec3 model_normal;
varying float world_pos_z;
varying float world_normal_z;

void main()
{
    model_pos = gl_Vertex.xyz;
    model_normal = gl_Normal;
    
    // Position in world coordinates.
    vec4 world_pos = print_box.volume_world_matrix * gl_Vertex;
    world_pos_z = world_pos.z;

    // compute deltas for out of print volume detection (world coordinates)
    if (print_box.active) {
        delta_box_min = world_pos.xyz - print_box.min;
        delta_box_max = world_pos.xyz - print_box.max;
    }
    else {
        delta_box_min = ZERO;
        delta_box_max = ZERO;
    }

    // z component of normal vector in world coordinate used for slope shading
	world_normal_z = slope.active ? (normalize(slope.volume_world_normal_matrix * gl_Normal)).z : 0.0;
    
    // Fill in the scalars for fragment shader clipping. Fragments with any of these components lower than zero are discarded.
    if (clipping_plane.active)
        clipping_planes_dots = vec3(dot(world_pos, clipping_plane.plane), world_pos.z - clipping_plane.z_range.x, clipping_plane.z_range.y - world_pos.z);
        
    gl_Position = ftransform();
}
