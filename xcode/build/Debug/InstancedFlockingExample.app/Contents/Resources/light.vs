#version 400 core

uniform mat4 mv;
uniform mat4 p;
uniform vec3 light;

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in mat4 transformMat;

out VS_OUT {
    vec3 N;
    vec3 L;
    vec3 V;
} vs_out;

void main(){
    
    vec4 P = transformMat*vec4(position,1.);
    vs_out.N = mat3(transformMat)*normal;
    vs_out.L = light - P.xyz;
    vs_out.V = -P.xyz;
    gl_Position = p*mv*P;
    
}