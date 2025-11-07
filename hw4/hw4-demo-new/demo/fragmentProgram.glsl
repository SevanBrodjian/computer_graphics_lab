/* Straightforward fragment shader that will calculate per-fragment
 * normals if toggled on and use the normal values for the texture
 * coordinates of the sky and leaf textures, else it will take the
 * texture coordinates from the vertex shader. The two textures are
 * then mixed together and displayed.
 */

uniform float t, toggle;
varying float u, v;

uniform sampler2D sky, leaf;

void main()
{    
    vec4 texcoord0 = gl_TexCoord[0];
    vec4 texcoord1 = gl_TexCoord[1];
    if(toggle == 0.0) // per-fragment normal calculation
    {
        float tx = t;
        
        // z is a cosine function of u, v, and t, so the normals
        // would be derivatives of cosine -> sine functions
        float nu = 2.0 * u * sin(u * u + v * v + tx * tx);
        float nv = 2.0 * v * sin(u * u + v * v + tx * tx);
        
        // normalize normal
        float mag = sqrt(nu * nu + nv * nv + 1.0);
        nu /= mag;
        nv /= mag;
        
        // set texture coordinates
        // 0 = sky, 1 = leaf
        texcoord0 = vec4(nu, nv, 0.0, 1.0);
        texcoord1 = vec4(u, v, 0.0, 1.0);
    }
    
    // set texture colors
    vec4 skyColor = texture2D(sky, texcoord0.st);
    vec4 leafColor = texture2D(leaf, texcoord1.st);
    
    // blend the textures together using alpha of leaf texture
    gl_FragColor = mix(skyColor, leafColor, leafColor.a);
}
