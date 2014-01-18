#version 400 core

vec4 myColor = vec4(1., 0.9, 0.9, 1.);

in VS_OUT {
    vec3 N;
    vec3 L;
    vec3 V;
} vs_in;

vec3 diffuse_albedo = vec3(1.,.9,.9);
vec3 specular_albedo = vec3(1.,.9,.9);
float specular_power = 1.0;
vec3 ambient = vec3(.09);

out vec4 color;

void main(){
    
    vec3 N = normalize(vs_in.N);
    vec3 L = normalize(vs_in.L);
    vec3 V = normalize(vs_in.V);
    
    vec3 H = normalize(L+V);
    
    float mag = length(vs_in.L);
    
    vec3 diffuse = (max(dot(N,L),0.)*diffuse_albedo) / (mag*.25);
    vec3 specular = (pow(max(dot(N,H),0.),specular_power)*specular_albedo) / (mag*.25);
    vec4 intensity = vec4(vec3(6.),1.);
    color = min(((vec4(ambient + diffuse + specular, 1.) / (mag*.25) )*intensity),vec4(1.)) * myColor;
    // color = vec4(1.0);
}