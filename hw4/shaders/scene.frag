#version 120
varying vec3 vPosition;
varying vec3 vNormal;
varying vec3 vGouraud;

uniform int uShadingMode;
uniform vec3 uAmbientLight;
uniform vec3 uMaterialAmbient;
uniform vec3 uMaterialDiffuse;
uniform vec3 uMaterialSpecular;
uniform float uMaterialShininess;
uniform int uLightCount;
uniform vec3 uLightPositions[8];
uniform vec3 uLightColors[8];
uniform float uLightAttenuations[8];

vec3 computeLighting(vec3 position, vec3 normal) {
    vec3 viewDir = normalize(-position);
    vec3 result = uMaterialAmbient * uAmbientLight;
    for (int i = 0; i < 8; ++i) {
        if (i >= uLightCount) { break; }
        vec3 lightVec = uLightPositions[i] - position;
        float distance = length(lightVec);
        vec3 L = normalize(lightVec);
        vec3 H = normalize(L + viewDir);
        float diff = max(dot(normal, L), 0.0);
        float spec = 0.0;
        if (diff > 0.0) {
            spec = pow(max(dot(normal, H), 0.0), uMaterialShininess);
        }
        float atten = 1.0 / (1.0 + uLightAttenuations[i] * distance * distance);
        vec3 lightColor = uLightColors[i] * atten;
        result += uMaterialDiffuse * diff * lightColor;
        result += uMaterialSpecular * spec * lightColor;
    }
    return result;
}

void main() {
    if (uShadingMode == 0) {
        gl_FragColor = vec4(vGouraud, 1.0);
    } else {
        vec3 normal = normalize(vNormal);
        vec3 color = computeLighting(vPosition, normal);
        gl_FragColor = vec4(color, 1.0);
    }
}
