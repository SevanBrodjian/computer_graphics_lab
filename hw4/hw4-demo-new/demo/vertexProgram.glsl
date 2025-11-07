/* Straightforward vertex shader that will calculate per-vertex normals
 * if toggled on and use the normal values for the texture coordinates
 * of the sky and leaf textures, passing them to the fragment shader.
 * This also calculates the vertex positions, using a cosine function
 * of x, y, and t to calculate z.
 */

uniform float t, toggle;
varying float u, v;

const float waterScale = 5.0;

void main()
{
    u = gl_Vertex.x;
    v = gl_Vertex.y;
    
    float tx = t;
    float z = cos(u * u + v * v + tx * tx) / waterScale;
    
    // set vector with the vertex
    vec3 world = vec3(u, v, z);
    // transform points using the model view matrix
    gl_Position = gl_ModelViewProjectionMatrix * vec4(world, 1.0);
    
    if(toggle == 1.0) // per-vertex normal calculation
    {
        float nu = 2.0 * u * sin(u * u + v * v + tx * tx);
        float nv = 2.0 * v * sin(u * u + v * v + tx * tx);
   
        // normalize normal
        float mag = sqrt(nu * nu + nv * nv + 1.0);
        nu /= mag;
        nv /= mag;
        
        // set texture coordinates
        // 0 = sky, 1 = leaf
        gl_TexCoord[0] = vec4(nu, nv, 0.0, 1.0);
        gl_TexCoord[1] = vec4(u, v, 0.0, 1.0);
    }
}
