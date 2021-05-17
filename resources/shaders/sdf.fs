#version 110

// cant create array
//uniform vec3 colors[9] = vec3[9]( 
float colors_size = 9.;
const vec3 c8 = vec3(0.0, 0.0, 0.5);
const vec3 c7 = vec3(0.0, 0.0, 1.0);
const vec3 c6 = vec3(0.0, 0.5, 1.0);
const vec3 c5 = vec3(0.0, 1.0, 1.0);
const vec3 c4 = vec3(0.5, 1.0, 0.5);
const vec3 c3 = vec3(1.0, 1.0, 0.0);
const vec3 c2 = vec3(1.0, 0.5, 0.0);
const vec3 c1 = vec3(1.0, 0.0, 0.0);
const vec3 c0 = vec3(0.5, 0.0, 0.0);

uniform float min_width;
uniform float width_range;
uniform bool draw_normals;

varying vec3 normal;
varying float width;
varying vec2 intensity;

float color_index(float value){
    return (value - min_width) * colors_size / width_range;
}

vec3 get_color(int index){
    if(index == 0) return c0;
    if(index == 1) return c1;
    if(index == 2) return c2;
    if(index == 3) return c3;
    if(index == 4) return c4;
    if(index == 5) return c5;
    if(index == 6) return c6;
    if(index == 7) return c7;
    return c8;
}

vec3 width_to_color3(float width){
    float index_val = color_index(width); 
    if(index_val <= 0.) return get_color(0);
    if(index_val >= 8.) return get_color(8);
    float index_f = floor(index_val);
    int index = int(index_f);
    float ratio = index_val - index_f;    
    return (1. - ratio) * get_color(index) +
                  ratio * get_color(index+1);
}

void main()
{
    vec3 color = width_to_color3(width);
    if(draw_normals){
        color = (normal + 1. ) / 2.;
    }
    gl_FragColor = vec4(vec3(intensity.y) + color * intensity.x, 1.);
}