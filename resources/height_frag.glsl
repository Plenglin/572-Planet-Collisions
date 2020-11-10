#version 450 
#extension GL_ARB_shader_storage_buffer_object : require
out vec4 color;
in vec3 vertex_pos;
in vec2 vertex_tex;


uniform sampler2D tex;
uniform sampler2D tex2;
uniform vec3 camoff;
uniform vec3 campos;

void main()
{
color.rgb = texture(tex2, vertex_tex).rgb;
color.a=1;
}
