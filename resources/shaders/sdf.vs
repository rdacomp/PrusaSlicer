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


attribute vec4 v_position;
attribute vec3 v_normal;
attribute float v_width;

uniform mat4 model_matrix;

varying vec3 normal;
varying float width;

// gouraud
// x = diffuse, y = specular;
varying vec2 intensity;

void main()
{
    gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * model_matrix * v_position;   
    width = v_width;
    normal = v_normal;
        
    // First transform the normal into camera space and normalize the result.
    vec4 model_normal = model_matrix * vec4(v_normal.xyz, 0.);    
    vec3 eye_normal = normalize(gl_NormalMatrix * gl_Normal * model_normal.xyz);
    
    // Compute the cos of the angle between the normal and lights direction. The light is directional so the direction is constant for every vertex.
    // Since these two are normalized the cosine is the dot product. We also need to clamp the result to the [0,1] range.
    float NdotL = max(dot(eye_normal, LIGHT_TOP_DIR), 0.0);

    intensity.x = INTENSITY_AMBIENT + NdotL * LIGHT_TOP_DIFFUSE;
    vec3 position = (gl_ModelViewMatrix * model_matrix * v_position).xyz;
    intensity.y = LIGHT_TOP_SPECULAR * pow(max(dot(-normalize(position), reflect(-LIGHT_TOP_DIR, eye_normal)), 0.0), LIGHT_TOP_SHININESS);

    // Perform the same lighting calculation for the 2nd light source (no specular applied).
    NdotL = max(dot(eye_normal, LIGHT_FRONT_DIR), 0.0);
    intensity.x += NdotL * LIGHT_FRONT_DIFFUSE;
}
