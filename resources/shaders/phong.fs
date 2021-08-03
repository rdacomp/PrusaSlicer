#version 110

#define INTENSITY_CORRECTION 0.6

// normalized values for (-0.6/1.31, 0.6/1.31, 1./1.31)
const vec3 LIGHT_TOP_DIR = vec3(-0.4574957, 0.4574957, 0.7624929);
#define LIGHT_TOP_DIFFUSE    (0.8 * INTENSITY_CORRECTION)
#define LIGHT_TOP_SPECULAR   (0.125 * INTENSITY_CORRECTION)
#define LIGHT_TOP_SHININESS  20.0

// normalized values for (1./1.43, 0.2/1.43, 1./1.43)
const vec3 LIGHT_FRONT_DIR = vec3(0.6985074, 0.1397015, 0.6985074);
#define LIGHT_FRONT_DIFFUSE  (0.3 * INTENSITY_CORRECTION)
//#define LIGHT_FRONT_SPECULAR (0.0 * INTENSITY_CORRECTION)
//#define LIGHT_FRONT_SHININESS 5.0

#define INTENSITY_AMBIENT    0.3

const vec3 ZERO = vec3(0.0, 0.0, 0.0);
const vec3 GREEN = vec3(0.0, 0.7, 0.0);
const vec3 YELLOW = vec3(0.5, 0.7, 0.0);
const vec3 RED = vec3(0.7, 0.0, 0.0);
const vec3 WHITE = vec3(1.0, 1.0, 1.0);
const float EPSILON = 0.0001;
const float BANDS_WIDTH = 10.0;

#define PI 3.1415926538
#define TWO_PI (2.0 * PI)

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

struct ProjectedTexture
{
    bool active;
    // 0 = cubic, 1 = cylindrical, 2 = spherical
    int projection;
    BoundingBox box;
};

struct ClippingPlane
{
    bool active;
    // Clipping plane, x = min z, y = max z. Used by the FFF and SLA previews to clip with a top / bottom plane.
    vec2 z_range;
    // Clipping plane - general orientation. Used by the SLA gizmo.
    vec4 plane;
};

uniform vec4 uniform_color;
uniform SlopeDetection slope;
uniform ProjectedTexture proj_texture;
uniform sampler2D projection_tex;
uniform ClippingPlane clipping_plane;

uniform bool compute_triangle_normals_in_fs;

#ifdef ENABLE_ENVIRONMENT_MAP
    uniform bool use_environment_tex;
    uniform sampler2D environment_tex;
#endif // ENABLE_ENVIRONMENT_MAP

varying vec3 clipping_planes_dots;

varying vec3 delta_box_min;
varying vec3 delta_box_max;

varying vec3 model_pos;
varying vec3 model_normal;

varying float world_pos_z;
varying float world_normal_z;

// x = diffuse, y = specular;
varying vec2 intensity;

varying vec3 eye_normal;

float azimuth(vec2 dir)
{
    float ret = atan(dir.y, dir.x); // [-PI..PI]
    if (ret < 0.0)
        ret += TWO_PI; // [0..2*PI]
    ret /= TWO_PI; // [0..1]    
    return ret;
}

vec2 cubic_uv(vec3 position)
{
    vec2 ret = vec2(0.0, 0.0);
    return ret;
}

vec2 cylindrical_uv(vec3 position, vec3 normal)
{
    vec2 ret = vec2(0.0, 0.0);
    vec3 dir = position - proj_texture.box.center;
	float max_box_xy_size = max(proj_texture.box.sizes.x, proj_texture.box.sizes.y);
	max_box_xy_size *= PI / floor(PI);
    if (length(normal.xy) == 0.0) {
        // caps
        ret = dir.xy / max_box_xy_size + 0.5;
        if (dir.z < 0.0)
            ret.y = 1.0 - ret.y;
    }
    else {
        ret.x = azimuth(dir.xy) * floor(PI);
        float min_z = proj_texture.box.center.z - 0.5 * proj_texture.box.sizes.z;
        ret.y = (position.z - min_z) / proj_texture.box.sizes.z; // [0..1]
    }
    return ret;
}

vec2 spherical_uv(vec3 position)
{
    vec2 ret = vec2(0.0, 0.0);
    vec3 dir = position - proj_texture.box.center;
    ret.x = azimuth(dir.xy) * 2.0;
    ret.y = atan(length(dir.xy), -dir.z) / PI; // [0..1]
    return ret;
}

vec2 calc_intensity(vec3 eye_position, vec3 eye_normal)
{
    vec2 ret = vec2(0.0, 0.0);
    
    // Compute the cos of the angle between the normal and lights direction. The light is directional so the direction is constant for every vertex.
    // Since these two are normalized the cosine is the dot product. We also need to clamp the result to the [0,1] range.
    float NdotL = max(dot(eye_normal, LIGHT_TOP_DIR), 0.0);

    ret.x = INTENSITY_AMBIENT + NdotL * LIGHT_TOP_DIFFUSE;
    ret.y = LIGHT_TOP_SPECULAR * pow(max(dot(-normalize(eye_position), reflect(-LIGHT_TOP_DIR, eye_normal)), 0.0), LIGHT_TOP_SHININESS);

    // Perform the same lighting calculation for the 2nd light source (no specular applied).
    NdotL = max(dot(eye_normal, LIGHT_FRONT_DIR), 0.0);
    ret.x += NdotL * LIGHT_FRONT_DIFFUSE;
    
    return ret;
}

void main()
{
    if (clipping_plane.active && any(lessThan(clipping_planes_dots, ZERO)))
        discard;

    vec2 intensity_fs = intensity;
    vec3 eye_normal_fs = eye_normal;
    // z component of normal vector in world coordinate used for slope shading
    float world_normal_z_fs = world_normal_z;
    if (compute_triangle_normals_in_fs) {
		// Normal shared by the three vertices of the triangle, in model space
		vec3 triangle_normal = normalize(cross(dFdx(model_pos), dFdy(model_pos)));
#ifdef FLIP_TRIANGLE_NORMALS
        triangle_normal = -triangle_normal;
#endif
		// Transform position and normal in camera space
		vec3 eye_position = (gl_ModelViewMatrix * vec4(model_pos, 1.0)).xyz;
		eye_normal_fs = normalize(gl_NormalMatrix * triangle_normal);
		intensity_fs = calc_intensity(eye_position, eye_normal_fs);
		world_normal_z_fs = slope.active ? (normalize(slope.volume_world_normal_matrix * triangle_normal)).z : 0.0;
	}

    vec3 color = uniform_color.rgb;
    float alpha = uniform_color.a;		
    if (slope.active && world_normal_z_fs < slope.normal_z - EPSILON) {
        color = vec3(0.7, 0.7, 1.0);
        alpha = 1.0;
    }
    // if the fragment is outside the print volume -> use darker color
	color = (any(lessThan(delta_box_min, ZERO)) || any(greaterThan(delta_box_max, ZERO))) ? mix(color, ZERO, 0.3333) : color;
    if (proj_texture.active) {
        vec2 tex_coords;
        if (proj_texture.projection == 0)
            tex_coords = cubic_uv(model_pos);
        else if (proj_texture.projection == 1)
            tex_coords = cylindrical_uv(model_pos, model_normal);
        else if (proj_texture.projection == 2)
            tex_coords = spherical_uv(model_pos);    
        color = mix(color, texture2D(projection_tex, tex_coords).rgb, 0.5);
    }
             
#ifdef ENABLE_ENVIRONMENT_MAP
    if (use_environment_tex)
        gl_FragColor = vec4(0.45 * texture2D(environment_tex, normalize(eye_normal_fs).xy * 0.5 + 0.5).xyz + 0.8 * color * intensity_fs.x, alpha);
    else
#endif
        gl_FragColor = vec4(vec3(intensity_fs.y) + color * intensity_fs.x, alpha);
}
