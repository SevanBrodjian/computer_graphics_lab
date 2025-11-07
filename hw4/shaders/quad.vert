#version 120
attribute vec3 aPosition;
attribute vec3 aNormal;
attribute vec3 aTangent;
attribute vec3 aBitangent;
attribute vec2 aTexCoord;

uniform mat4 uModelView;
uniform mat4 uProjection;
uniform mat3 uNormalMatrix;

varying vec3 vPosition;
varying vec3 vNormal;
varying vec3 vTangent;
varying vec3 vBitangent;
varying vec2 vTexCoord;

void main() {
    vec4 viewPos = uModelView * vec4(aPosition, 1.0);
    vPosition = viewPos.xyz;
    vNormal = normalize(uNormalMatrix * aNormal);
    vTangent = normalize(uNormalMatrix * aTangent);
    vBitangent = normalize(uNormalMatrix * aBitangent);
    vTexCoord = aTexCoord;
    gl_Position = uProjection * viewPos;
}
