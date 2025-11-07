#version 120
varying vec3 vPosition;
varying vec3 vNormal;
varying vec3 vTangent;
varying vec3 vBitangent;
varying vec2 vTexCoord;

uniform sampler2D uColorTexture;
uniform sampler2D uNormalTexture;
uniform vec3 uLightPosition;
uniform vec3 uLightColor;
uniform vec3 uAmbientLight;
uniform vec3 uSpecularColor;
uniform float uShininess;

void main() {
    vec3 baseColor = texture2D(uColorTexture, vTexCoord).rgb;
    vec3 normalSample = texture2D(uNormalTexture, vTexCoord).rgb;
    vec3 tangentNormal = normalize(normalSample * 2.0 - 1.0);

    mat3 TBN = mat3(normalize(vTangent), normalize(vBitangent), normalize(vNormal));
    vec3 normal = normalize(TBN * tangentNormal);

    vec3 viewDir = normalize(-vPosition);
    vec3 lightVec = uLightPosition - vPosition;
    float distance = length(lightVec);
    vec3 L = normalize(lightVec);
    vec3 H = normalize(L + viewDir);

    float diff = max(dot(normal, L), 0.0);
    float spec = 0.0;
    if (diff > 0.0) {
        spec = pow(max(dot(normal, H), 0.0), uShininess);
    }

    float attenuation = 1.0 / (1.0 + 0.02 * distance * distance);
    vec3 lightColor = uLightColor * attenuation;

    vec3 color = baseColor * uAmbientLight;
    color += baseColor * diff * lightColor;
    color += uSpecularColor * spec * lightColor;
    gl_FragColor = vec4(color, 1.0);
}
