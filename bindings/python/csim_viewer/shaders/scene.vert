#version 330 core

layout(location = 0) in vec3 a_position;
layout(location = 1) in vec3 a_color;
layout(location = 2) in vec3 a_normal;

uniform mat4 u_mvp;

out vec3 v_color;

void main() {
    // Zero normals mark unlit lines and the screen overlay.
    float light = dot(a_normal, a_normal) < 0.01 ? 1.0 :
        0.42 + 0.58 * max(dot(normalize(a_normal), normalize(vec3(0.3,-0.4,1.0))), 0.0);
    v_color = a_color * light;
    gl_Position = u_mvp * vec4(a_position, 1.0);
}
