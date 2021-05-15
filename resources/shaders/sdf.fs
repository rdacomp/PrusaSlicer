#version 110

varying vec3 normal;
varying float width;

void main()
{
    gl_FragColor = vec4(normal.x, normal.y, normal.z, 1.);
}