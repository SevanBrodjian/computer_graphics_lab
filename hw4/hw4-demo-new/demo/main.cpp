#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <cstdlib>
#include <vector>
#define GL_GLEXT_PROTOTYPES 1
#ifdef __APPLE__
#include <OpenGL/gl.h>
// #include <GL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#endif


// glu replacements:
static void perspectiveGL(double fovy_deg, double aspect, double zNear, double zFar) {
    const double fovy_rad = fovy_deg * 3.14159265358979323846 / 180.0;
    const double top = zNear * tan(0.5 * fovy_rad);
    const double right = top * aspect;
    glFrustum(-right, right, -top, top, zNear, zFar);
}

static const char* glErrString(GLenum e) {
    switch (e) {
        case GL_NO_ERROR: return "GL_NO_ERROR";
        case GL_INVALID_ENUM: return "GL_INVALID_ENUM";
        case GL_INVALID_VALUE: return "GL_INVALID_VALUE";
        case GL_INVALID_OPERATION: return "GL_INVALID_OPERATION";
#ifdef GL_STACK_OVERFLOW
        case GL_STACK_OVERFLOW: return "GL_STACK_OVERFLOW";
#endif
#ifdef GL_STACK_UNDERFLOW
        case GL_STACK_UNDERFLOW: return "GL_STACK_UNDERFLOW";
#endif
        case GL_OUT_OF_MEMORY: return "GL_OUT_OF_MEMORY";
#ifdef GL_INVALID_FRAMEBUFFER_OPERATION
        case GL_INVALID_FRAMEBUFFER_OPERATION: return "GL_INVALID_FRAMEBUFFER_OPERATION";
#endif
        default: return "Unknown GL error";
    }
}


// The number of points to divide the grid into
static const float gridSize = 100;

using namespace std;

extern GLenum readpng(const char *filename);

static double rho = 0, theta = 0;
static int lastX, lastY;
static GLenum shaderProgram;
static string vertProgFileName, fragProgFileName;
static GLenum skyTex, leafTex;
static GLint leafUniformPos, skyUniformPos;
static GLint tUniformPos, toggleUniformPos;
static GLuint displayList;

float toggle = 1.0; // toggle variable for normal modes; per-vertex vs per-fragment

static double clip(double x, double a, double b) {
   double y = (x>a)?x:a;
   return (y<b)?y:b;
}

static void initGL() {
   GLfloat pos[] = {7.0, 2.0, 3.0, 1.0};
   glEnable(GL_LIGHTING);
   glEnable(GL_LIGHT0);
   glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,1);
   glEnable(GL_COLOR_MATERIAL);
   glColorMaterial(GL_FRONT,GL_DIFFUSE);
   GLfloat black[4] = {0,0,0,0};
   glMaterialfv(GL_FRONT,GL_AMBIENT,black);
   glMaterialfv(GL_FRONT,GL_SPECULAR,black);
   glLightfv(GL_LIGHT0, GL_POSITION, pos);

   glEnable(GL_DEPTH_TEST);

   cerr << "Loading textures" << endl;
   if(!(skyTex = readpng("sky.png")))
      exit(1);
   if(!(leafTex = readpng("leaf.png")))
      exit(1);
}

static void readShaders() {
   string vertProgramSource, fragProgramSource;
   
   ifstream vertProgFile(vertProgFileName.c_str());
   if (! vertProgFile)
      cerr << "Error opening vertex shader program\n";
   ifstream fragProgFile(fragProgFileName.c_str());
   if (! fragProgFile)
      cerr << "Error opening fragment shader program\n";

   getline(vertProgFile, vertProgramSource, '\0');
   const char* vertShaderSource = vertProgramSource.c_str();

   getline(fragProgFile, fragProgramSource, '\0');
   const char* fragShaderSource = fragProgramSource.c_str();

   char buf[1024];
   GLsizei blah;

   // Initialize shaders
   GLenum vertShader, fragShader;

   shaderProgram = glCreateProgram();

   vertShader = glCreateShader(GL_VERTEX_SHADER);
   glShaderSource(vertShader, 1, &vertShaderSource, NULL);
   glCompileShader(vertShader);
    
   GLint isCompiled = 0;
   glGetShaderiv(vertShader, GL_COMPILE_STATUS, &isCompiled);
   if(isCompiled == GL_FALSE)
   {
      GLint maxLength = 0;
      glGetShaderiv(vertShader, GL_INFO_LOG_LENGTH, &maxLength);
    
      // The maxLength includes the NULL character
      std::vector<GLchar> errorLog(maxLength);
      glGetShaderInfoLog(vertShader, maxLength, &maxLength, &errorLog[0]);
    
      // Provide the infolog in whatever manor you deem best.
      // Exit with failure.
      for (int i = 0; i < errorLog.size(); i++)
         cout << errorLog[i];
      glDeleteShader(vertShader); // Don't leak the shader.
      return;
   }

   fragShader = glCreateShader(GL_FRAGMENT_SHADER);
   glShaderSource(fragShader, 1, &fragShaderSource, NULL);
   glCompileShader(fragShader);

   isCompiled = 0;
   glGetShaderiv(fragShader, GL_COMPILE_STATUS, &isCompiled);
   if(isCompiled == GL_FALSE)
   {
      GLint maxLength = 0;
      glGetShaderiv(fragShader, GL_INFO_LOG_LENGTH, &maxLength);
    
      // The maxLength includes the NULL character
      std::vector<GLchar> errorLog(maxLength);
      glGetShaderInfoLog(fragShader, maxLength, &maxLength, &errorLog[0]);
    
      // Provide the infolog in whatever manor you deem best.
      // Exit with failure.
      for (int i = 0; i < errorLog.size(); i++)
         cout << errorLog[i];
      glDeleteShader(fragShader); // Don't leak the shader.
      return;
   }

   glAttachShader(shaderProgram, vertShader);
   glAttachShader(shaderProgram, fragShader);
   glLinkProgram(shaderProgram);
   cerr << "Enabling fragment program: " << glErrString(glGetError()) << endl;
   glGetProgramInfoLog(shaderProgram, 1024, &blah, buf);
   cerr << buf;

   cerr << "Enabling program object" << endl;
   glUseProgram(shaderProgram);

   leafUniformPos = glGetUniformLocation(shaderProgram, "leaf");
   skyUniformPos = glGetUniformLocation(shaderProgram, "sky");
   tUniformPos = glGetUniformLocation(shaderProgram, "t");
   toggleUniformPos = glGetUniformLocation(shaderProgram, "toggle");

   glActiveTexture(GL_TEXTURE0);
   glBindTexture(GL_TEXTURE_2D, skyTex);
   glUniform1i(skyUniformPos, 0);

   glActiveTexture(GL_TEXTURE1);
   glBindTexture(GL_TEXTURE_2D, leafTex);
   glUniform1i(leafUniformPos, 1);
}


static void display() {
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

   const float size = 5.0;
   const float du = 2*size/gridSize, dv = 2*size/gridSize;

   glColor3f(0, 1, 0);
   for(float u = -size; u <= size; u += du) {
      glBegin(GL_TRIANGLE_STRIP);
      for(float v = -size; v <= size; v += dv) {
         glVertex2f(u, v);
         glVertex2f(u+du, v);
      }
      glEnd();
   }

   glUseProgram(0);
   glPopMatrix();

   glColor3f(0, 1, 0);
   glBegin(GL_LINES);
   glVertex3f(-5,0,0);
   glVertex3f(5,0,0);
   glVertex3f(0,-5,0);
   glVertex3f(0,5,0);
   glEnd();
}

static void doList() {
   const float t = glutGet(GLUT_ELAPSED_TIME) * -0.001;

   glMatrixMode(GL_MODELVIEW);

   glPushMatrix();
   glRotatef(rho,1,0,0);
   glRotatef(theta,0,1,0);

   glColor3f(1,0,0);
   //glDisable(GL_TEXTURE_2D);
   glUseProgram(shaderProgram);
   glUniform1f(tUniformPos, t);
   glUniform1f(toggleUniformPos, toggle);

   glCallList(displayList);

   glutSwapBuffers();
}

static void reshape(int x, int y) {
   const double aspect = x/(double)y;

   glViewport(0, 0, x, y);

   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   perspectiveGL(60, aspect, 1, 50);

   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   glTranslatef(0,0,-10);
}

static void mouseButton(int button, int state, int x, int y) {
   switch(button) {
      case GLUT_LEFT_BUTTON:
         if(state == GLUT_DOWN) {
            lastX = x;
            lastY = y;
         }
         break;
      case GLUT_MIDDLE_BUTTON:
         if(state == GLUT_DOWN) {
           readShaders();
         }
         glutPostRedisplay();
         break;
      case GLUT_RIGHT_BUTTON:
         if(state == GLUT_DOWN)
            toggle = (toggle == 1.0) ? 0.0 : 1.0;
         break;
   }
}

static void motion(int x, int y) {
   double dx = x-lastX, dy = y-lastY;
   lastX = x;
   lastY = y;

   theta += dx;
   rho += dy;

   rho = clip(rho, -90, 90);
   glutPostRedisplay();
}

int main(int argc, char *argv[]) {
   glutInit(&argc, argv);
   
   glutInitWindowSize(atoi(argv[1]), atoi(argv[2]));
   glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
   glutCreateWindow("GLSL test");

   initGL();

   vertProgFileName = "vertexProgram.glsl";
   fragProgFileName = "fragmentProgram.glsl";
   readShaders();

   displayList = glGenLists(1);
   glNewList(displayList, GL_COMPILE);
   display();
   glEndList();

   glutReshapeFunc(reshape);
   glutDisplayFunc(doList);
   glutIdleFunc(doList);
   glutMouseFunc(mouseButton);
   glutMotionFunc(motion);
   glutMainLoop();
   return 0;
}
