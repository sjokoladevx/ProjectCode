#include <GLUT/glut.h>
#include <stdio.h> 
#include <stdlib.h>

int w, h;
const int font=(int)GLUT_BITMAP_9_BY_15;
char b[30]; 
char p[30]; 
double batteryLevel;
double batteryPercent;
static void resize(int width, int height)

//CITE: http://www.programming-techniques.com/2012/05/font-rendering-in-glut-using-bitmap.html

{
    const float ar = (float) width / (float) height;
    w = width;
    h = height;
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glFrustum(-ar, ar, -1.0, 1.0, 2.0, 100.0);     glMatrixMode(GL_MODELVIEW);
    glLoadIdentity() ;
} 
void setOrthographicProjection() {
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, w, 0, h);
    glScalef(1, -1, 1);
    glTranslatef(0, -h, 0);
    glMatrixMode(GL_MODELVIEW);
} 
void resetPerspectiveProjection() {
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
} 
void renderBitmapString(float x, float y, void *font,const char *string){
    const char *c;
    glRasterPos2f(x, y);
    for (c=string; *c != '\0'; c++) {
        glutBitmapCharacter(font, *c);
    }
} 
static void display(void){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor3d(1.0, 0.0, 0.0);
    setOrthographicProjection();
    glPushMatrix();
    glLoadIdentity();
    renderBitmapString(20,20,(void *)font,"Copter Battery Life");
    renderBitmapString(300,100,(void *)font,"Acceleration X");
    renderBitmapString(300,200,(void *)font,"Acceleration Y");
    renderBitmapString(300,150,(void *)font,"Acceleration Z");
    renderBitmapString(300,250,(void *)font,"Temperature");
    renderBitmapString(300,300,(void *)font,"Altitude");
    renderBitmapString(300,350,(void *)font,"Pressure");

    int i = 60;
    renderBitmapString(20,i-20,(void *)font,"        _____");
    renderBitmapString(20,i-10,(void *)font,"        |   |");
    renderBitmapString(20,i,(void *)font,"________|   |________");
    renderBitmapString(20,i+40,(void *)font,"          +");
    while (i < 360){
    i+= 10;
    renderBitmapString(20,i,(void *)font,"|                  |");
    }
    renderBitmapString(20,i-40,(void *)font,"          -");
    renderBitmapString(20,i,(void *)font,"____________________");

    //CODE SNIPPET

    i = 0;
    while (i < batteryPercent){

    glColor3d(1.0 - batteryPercent/100.0, batteryPercent/100, 0.0);
    renderBitmapString(29,300 - ((i/100.0)*300) + 60,(void *)font,"@@@@@@@@@@@@@@@@@@");
    //printf("%f\n",300 - ((i/100.0)*300) + 60);
    i+= 5;
    }

    //print battery level
    renderBitmapString(20,400, (void*)font, b);
    //print battery percent
    renderBitmapString(20,420, (void*)font, p);

    //SNIPPET

    glPopMatrix();
    resetPerspectiveProjection();
    glutSwapBuffers();
} 
void update(int value){
    batteryLevel = (rand() / (double)RAND_MAX) * 4;
    sprintf(b, "batteryLevel : %f", batteryLevel );
    batteryPercent = 100.0 * (batteryLevel / 4 );
    sprintf(p, "batteryPercent : %d%%", (int)batteryPercent );
    glutTimerFunc(1000, update, 0);
    glutPostRedisplay();
} 
int main(int argc, char *argv[])
{
    glutInit(&argc, argv);
    glutInitWindowSize(640,480);
    glutInitWindowPosition(10,10);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);     glutCreateWindow("Font Rendering Using Bitmap Font - Programming Techniques0");     glutReshapeFunc(resize);
    glutDisplayFunc(display);
    glutTimerFunc(25, update, 0);     
    glutMainLoop();

    return 0;
}