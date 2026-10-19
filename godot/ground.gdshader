shader_type spatial;
varying vec3 world_point;
void vertex() { world_point = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz; }
float hash(vec2 p) { return fract(sin(dot(p, vec2(127.1,311.7))) * 43758.5453); }
float noise(vec2 p) {
 vec2 i=floor(p); vec2 f=fract(p); f=f*f*(3.0-2.0*f);
 return mix(mix(hash(i),hash(i+vec2(1,0)),f.x),mix(hash(i+vec2(0,1)),hash(i+vec2(1,1)),f.x),f.y);
}
void fragment() {
 float n=noise(world_point.xz*0.15)*0.65+noise(world_point.xz*2.0)*0.35;
 ALBEDO=mix(vec3(0.13,0.22,0.075),vec3(0.42,0.48,0.22),n);
 ROUGHNESS=0.95;
}
