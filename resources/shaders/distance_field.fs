#version 110

const vec3 back_color_dark = vec3(0.235, 0.235, 0.235);
const vec3 back_color_light = vec3(0.365, 0.365, 0.365);
const vec4 fore_color = vec4(1.0, 1.0, 1.0, 1.0);

uniform sampler2D texture;
uniform bool transparent_background;

varying vec2 tex_coords;

const float smoothing = 1.0 / 64.0;

void main()
{
    // calculates radial gradient
    vec4 back_color = transparent_background ? vec4(0.0) : vec4(mix(back_color_light, back_color_dark, smoothstep(0.0, 0.5, length(abs(tex_coords.xy) - vec2(0.5)))), 1.0);

    // resolves distances field
    float alpha = smoothstep(0.5 - smoothing, 0.5 + smoothing, texture2D(texture, tex_coords).r);

    // blends foreground with background
    gl_FragColor = vec4(mix(back_color, fore_color, alpha));
}