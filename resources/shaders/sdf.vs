#version 110

attribute vec4 v_position;
attribute vec3 v_normal;
attribute float v_width;

varying vec3 normal;
varying float width;

void main()
{
    gl_Position = gl_ProjectionMatrix * gl_ModelViewMatrix * v_position;
    normal = v_normal;
    width = v_width;
}
