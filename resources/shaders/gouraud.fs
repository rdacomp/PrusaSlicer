#version 110

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
uniform bool sinking;
uniform ClippingPlane clipping_plane;

#ifdef ENABLE_ENVIRONMENT_MAP
    uniform bool use_environment_tex;
    uniform sampler2D environment_tex;
#endif // ENABLE_ENVIRONMENT_MAP

varying vec3 clipping_planes_dots;

// x = diffuse, y = specular;
varying vec2 intensity;

varying vec3 delta_box_min;
varying vec3 delta_box_max;

varying vec3 model_pos;
varying vec3 model_normal;

varying float world_pos_z;
varying float world_normal_z;
varying vec3 eye_normal;

vec3 sinking_color(vec3 position, vec3 color)
{
    return (mod(position.x + position.y + position.z, BANDS_WIDTH) < (0.5 * BANDS_WIDTH)) ? mix(color, ZERO, 0.6666) : color;
}

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
    if (length(normal.xy) == 0.0) {
        // caps
        ret = dir.xy / proj_texture.box.sizes.xy + 0.5;
        if (dir.z < 0.0)
            ret.y = 1.0 - ret.y;
    }
    else {
        ret.x = azimuth(dir.xy);        
        float min_z = proj_texture.box.center.z - 0.5 * proj_texture.box.sizes.z;
        ret.y = (position.z - min_z) / proj_texture.box.sizes.z; // [0..1]
    }
    return ret;
}

vec2 spherical_uv(vec3 position)
{
    vec2 ret = vec2(0.0, 0.0);
    vec3 dir = position - proj_texture.box.center;
    ret.x = azimuth(dir.xy);
    ret.y = atan(length(dir.xy), -dir.z) / PI; // [0..1]
    return ret;
}

void main()
{
    if (clipping_plane.active && any(lessThan(clipping_planes_dots, ZERO)))
        discard;
    vec3 color = uniform_color.rgb;
    float alpha = uniform_color.a;
    if (slope.active && world_normal_z < slope.normal_z - EPSILON) {
        color = vec3(0.7, 0.7, 1.0);
        alpha = 1.0;
    }
    // if the fragment is outside the print volume -> use darker color
	color = (any(lessThan(delta_box_min, ZERO)) || any(greaterThan(delta_box_max, ZERO))) ? mix(color, ZERO, 0.3333) : color;
    // if the object is sinking, shade it with inclined bands or white around world z = 0
    if (sinking)
        color = (abs(world_pos_z) < 0.05) ? WHITE : sinking_color(model_pos, color);
    if (proj_texture.active) {
        vec2 tex_coords;    
        if (proj_texture.projection == 1)
            tex_coords = cylindrical_uv(model_pos, model_normal);
        else if (proj_texture.projection == 2)
            tex_coords = spherical_uv(model_pos);
        else
            tex_coords = cubic_uv(model_pos);
    
        color = mix(color, texture2D(projection_tex, tex_coords).rgb, 0.5);
    }
#ifdef ENABLE_ENVIRONMENT_MAP
    if (use_environment_tex)
        gl_FragColor = vec4(0.45 * texture2D(environment_tex, normalize(eye_normal).xy * 0.5 + 0.5).xyz + 0.8 * color * intensity.x, alpha);
    else
#endif
        gl_FragColor = vec4(vec3(intensity.y) + color * intensity.x, alpha);
}
