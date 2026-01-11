#version 450
layout(binding = 0) uniform sampler2DArray uMipTexture; // mipmapped color array
layout(location = 0) uniform vec2 uGaze;                 // normalized gaze [0,1]
layout(location = 1) uniform float uFoveaRadius;         // fovea radius in [0,1]
layout(location = 2) uniform float uMaxMip;             // max mip level
layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 outColor;

float radialDist(vec2 a, vec2 b) { return length(a - b); }

void main() {
    float r = radialDist(vUV, uGaze);                  // normalized radial distance
    // smoothstep transition to avoid popping; map to desired mip level
    float t = smoothstep(uFoveaRadius, 1.0, r);
    float mip = mix(0.0, uMaxMip, t);                  // 0 is finest mip, uMaxMip coarsest
    // fetch from array using implicit LOD control
    ivec3 texSize = textureSize(uMipTexture, 0);
    int layer = int(clamp(floor(mip + 0.5), 0, int(uMaxMip)));
    // sample appropriate LOD from the mipmapped array
    vec4 c = texture(uMipTexture, vec3(vUV, layer));
    outColor = c;
}