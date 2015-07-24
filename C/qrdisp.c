/*--------------------------------------------------------------------------
 * qrdisp -- simple quad rotor 3D visualization
 *       taken from SourceForge Autopilot project and adapted
 *--------------------------------------------------------------------------
 */

#ifdef __WIN32__
#define GLUT_DISABLE_ATEXIT_HACK
#endif /*  */

#define ANTIALIAS_LINES

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>


/* quad rotor state
 */
#include "qrsim.h"
#include "qrmod.h"

extern qrstate_t qrstate;
extern camloc_t camloc;
extern void simulate();
extern void keyb(unsigned char key, int x, int y);
extern void skeyb(int key, int x, int y);


/* GLUT window info
 */
int         WIDTH = 640;
int         HEIGHT = 480;
int         LOC_X = 10;
int         LOC_Y = 10;

/* graphics stuff
 */
static double RAD2DEG = 180 / PI;

void DrawScene(camloc_t camloc, qrstate_t qrstate);
void LookAt(camloc_t camloc, double x, double y, double z,
            double phi, double theta, double psi);
void DrawQRModel(qrstate_t qrstate, int shadow);
void DrawRotor(double outer_radius, double inner_radius);

GLfloat blue[4] = { 0.0, 0.0, 1.0, 1.0 };
GLfloat red[4] = { 0.4, 0.0, 0.0, 1.0 };
GLfloat yellow[4] = { 0.9, 0.9, 0.0, 1.0 };
GLfloat dull_yellow[4] = { 0.4, 0.4, 0.0, 1.0 };
GLfloat grey[4] = { 0.2, 0.2, 0.2, 1.0 };
GLfloat darkgrey[4] = { 0.1, 0.1, 0.1, 1.0 };
GLfloat black[4] = { 0.0, 0.0, 0.0, 1.0 };
GLfloat shadowed[4] = { 0.0, 0.0, 0.0, 0.2 };
GLfloat dull[1] = { 10.0 };
GLfloat shiny[1] = { 100.0 };
GLfloat groundAmb[4] = { 0.2, 0.5, 0.1, 1.0 /* 0.5 -> lighter G */ };
GLfloat localAmb[4] = { 0.7, 0.7, 0.7, 1.0 };
GLfloat ambient0[4] = { 0.0, 0.0, 0.0, 1.0 };
GLfloat diffuse0[4] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat specular0[4] = { 1.0, 0.0, 0.0, 1.0 };
GLfloat ambient1[4] = { 0.0, 0.0, 0.0, 1.0 };
GLfloat diffuse1[4] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat specular1[4] = { 1.0, 0.0, 0.0, 1.0 };
GLfloat position0[4] = { 2.0, 100.5, 1.5, 1.0 };
GLfloat position1[4] = { -2.0, 100.5, 1.0, 0.0 };

double  groundHeight = -0.1;
int     shadow = 0;
double  main_rotor_angle = 0;
double  tail_rotor_angle = 0;
double* rotor_angle;

const double s_cg_wl = 10.91;
const double s_length = 8.0;
const double s_width = 5.6;
const double s_offset = 2.0;
const double s_height = 15.0;


/*--------------------------------------------------------------------------
 * draw everything (environment + quad rotor)
 *--------------------------------------------------------------------------
 */
void DrawScene(camloc_t camloc, qrstate_t qrstate)
{
    double phi, theta, psi;
    double glX, glY, glZ;
    int e;
    float mat[] = {
        1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };

    /* change from NASA airplane to OpenGL coordinates
     */
    glX = qrstate.x;
    glZ = qrstate.y;
    glY = -qrstate.z;

    phi = qrstate.phi;
    theta = qrstate.theta;
    psi = qrstate.psi;

    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, localAmb);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, 1);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 0);
    glEnable(GL_LIGHT0);
    glLightfv(GL_LIGHT0, GL_AMBIENT, ambient0);
    glLightfv(GL_LIGHT0, GL_POSITION, position0);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse0);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specular0);
    glEnable(GL_LIGHT1);
    glLightfv(GL_LIGHT1, GL_AMBIENT, ambient1);
    glLightfv(GL_LIGHT1, GL_POSITION, position1);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuse1);
    glLightfv(GL_LIGHT1, GL_SPECULAR, specular1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    /* determine camera location and orientation
     */
    LookAt(camloc, glX, glY, glZ, phi, theta, psi);

    /* mast
     */
    glEnable(GL_NORMALIZE);
    glDisable(GL_LIGHTING);
    glColor3f(0.0, 0.0, 1.0); /* blue */
    glBegin(GL_LINES);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 5.0, 0.0);
    glEnd();

    /* GROUND */
    glEnable(GL_LIGHTING);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, groundAmb);
    glBegin(GL_POLYGON);
    glNormal3f(0.0, 1.0, 0.0);
    glVertex3f(31200.0, groundHeight, -31200.0);
    glVertex3f(-31200.0, groundHeight, -31200.0);
    glVertex3f(-31200.0, groundHeight, 31200.0);
    glVertex3f(31200.0, groundHeight, 31200.0);
    glEnd();
    glDisable(GL_LIGHTING);

#ifdef ANTIALIAS_LINES
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
#endif

    /* grid
     */
    for (e = -1000; e <= 1000; e += 5) {
        glColor3f(0.0, 0.5, 0.0);
        glBegin(GL_LINES);
        glVertex3f(e, 0.0, -1000);
        glVertex3f(e, 0.0, 1000);
        glVertex3f(-1000, 0.0, e);
        glVertex3f(1000, 0.0, e);
        glEnd();
    }

#ifdef ANTIALIAS_LINES
    glDisable(GL_LINE_SMOOTH);
#endif


    /* draw the quad rotor
     */
    glEnable(GL_LIGHTING);
    glPushMatrix();
    glTranslatef(glX, glY, glZ);

    /* apply NASA aeroplane Euler angles standard
     * (in terms of OpenGL X,Y,Z frame where
     * Euler earth axes X,Y,Z are openGl axes X,Z,-Y)
     */
    glRotatef(RAD2DEG * psi, 0.0, -1.0, 0.0);
    glRotatef(RAD2DEG * theta, 0.0, 0.0, 1.0);
    glRotatef(RAD2DEG * phi, 1.0, 0.0, 0.0);

    DrawQRModel(qrstate, 0);

    glPopMatrix();
    glPushMatrix();
    glTranslatef(glX, 0.0, glZ);
    glMultMatrixf(mat);

    /* apply NASA aeroplane Euler angles standard
     * (same drill)
     */
    glRotatef(RAD2DEG * psi, 0.0, -1.0, 0.0);
    glRotatef(RAD2DEG * theta, 0.0, 0.0, 1.0);
    glRotatef(RAD2DEG * phi, 1.0, 0.0, 0.0);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    DrawQRModel(qrstate, 1);

    glDisable(GL_BLEND);
    glPopMatrix();
}

/*--------------------------------------------------------------------------
 * determine camera position and orientation depending on selected viewpoint
 *
 * needs to be improved for better viewing
 *--------------------------------------------------------------------------
 */
void LookAt(camloc_t camloc, double x, double y, double z, double phi,
            double theta, double psi)
{
    double camera[3];
    double dest[3];
    double up[3] = { 0.0, 1.0, 0.0 };

    /* defaults:
     */
    double dx = 10 * cos(psi); /* needs to be improved */
    double dz = 10 * sin(psi);

    dest[0] = x;
    dest[1] = y;
    dest[2] = z;
    camera[0] = camloc.tripod[0];
    camera[1] = camloc.tripod[1];
    camera[2] = camloc.tripod[2];

    /* determine settings based on selected viewpoint
     */
    switch (camloc.view) {
    case VIEW_STATIONARY:
        break;
    case VIEW_FLY_BEHIND:
        camera[0] = x - dx;
        camera[1] = y + 1;
        camera[2] = z - dz;
        break;
    case VIEW_WALK_BEHIND:
        camera[0] = x - dx;
        camera[1] = y + 4;
        camera[2] = z - dz;
        break;
    case VIEW_COCKPIT:
        camera[0] = x + 0.1 * cos(psi) * cos(theta);
        camera[1] = y + 0.1 * sin(theta);
        camera[2] = z + 0.1 * sin(psi);
        up[0] = -sin(phi) * sin(psi);
        up[1] = cos(phi) * cos(theta);
        up[2] = sin(phi) * cos(psi);
        dest[0] = x + 6.0 * cos(psi) * cos(theta);
        dest[1] = y + 6.0 * sin(theta);
        dest[2] = z + 6.0 * sin(psi);
        break;
    case VIEW_NORTH_UP:
        camera[0] = x;
        camera[1] = 30;
        camera[2] = z;
        up[2] = -1;
        break;
    case VIEW_TRACK_UP:
        camera[0] = x;
        camera[1] = 30;
        camera[2] = z;
        up[0] = dx;
        up[2] = dz;
        break;
    default:
        break;
    }
    gluLookAt(camera[0], camera[1], camera[2],
              dest[0], dest[1], dest[2], up[0], up[1], up[2]);
}

/*--------------------------------------------------------------------------
 * draw the quad rotor
 *--------------------------------------------------------------------------
 */
void DrawQRModel(qrstate_t qrstate, int shadow_arg)
{
    // double   phi, roll_moment, pitch_moment;

    // phi = qrstate.phi;
    // roll_moment = qrstate.mx;
    // pitch_moment = qrstate.my;

    shadow = shadow_arg;

    glEnable(GL_LIGHTING);
    glEnable(GL_NORMALIZE);

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE,
                 shadow ? shadowed : grey);

    /* outer protective cover
     */
    glBegin(GL_LINES);

    glVertex3f(1.0, 0.0, 0.5);
    glVertex3f(1.0, 0.0, -0.5);

    glVertex3f(1.0, 0.0, -0.5);
    glVertex3f(0.5, 0.0, -1.0);

    glVertex3f(0.5, 0.0, -1.0);
    glVertex3f(-0.5, 0.0, -1.0);

    glVertex3f(-0.5, 0.0, -1.0);
    glVertex3f(-1.0, 0.0, -0.5);

    glVertex3f(-1.0, 0.0, -0.5);
    glVertex3f(-1.0, 0.0, 0.5);

    glVertex3f(-1.0, 0.0, 0.5);
    glVertex3f(-0.5, 0.0, 1.0);

    glVertex3f(-0.5, 0.0, 1.0);
    glVertex3f(0.5, 0.0, 1.0);

    glVertex3f(0.5, 0.0, 1.0);
    glVertex3f(1.0, 0.0, 0.5);
    glEnd();

    /* motor struts
     */
    glBegin(GL_LINES);
    glVertex3f(-0.55, 0.0, 0.0);
    glVertex3f(0.55, 0.0, 0.0);

    glVertex3f(0.0, 0.0, -0.55);
    glVertex3f(0.0, 0.0, 0.55);
    glEnd();

    /* 4 motors, 1st one ("led") blinks to indicate front of vehicle
     */
    if (qrstate.leds && 0x0001) {
        glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE,
                     shadow ? shadowed : red);
    }
    glPushMatrix();
    glTranslatef(0.55, 0.0, 0.0);
    DrawRotor(0.08, 0.0);
    glPopMatrix();

    /* solid color on the bottom
     */
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE,
                 shadow ? shadowed : yellow);
    glPushMatrix();
    glTranslatef(0.55, -0.01, 0.0);
    DrawRotor(0.08, 0.0);
    glPopMatrix();

    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE,
                 shadow ? shadowed : grey);

    /* other 3 motors
     */
    glPushMatrix();
    glTranslatef(0.0, 0.0, 0.55);
    DrawRotor(0.08, 0.0);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-0.55, 0.0, 0.0);
    DrawRotor(0.08, 0.0);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0.0, 0.0, -0.55);
    DrawRotor(0.08, 0.0);
    glPopMatrix();


    /* 4 rotor covers
     */
    glPushMatrix();
    glTranslatef(0.55, 0.0, 0.0);
    DrawRotor(0.35, 0.33);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0.0, 0.0, 0.55);
    DrawRotor(0.35, 0.33);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-0.55, 0.0, 0.0);
    DrawRotor(0.35, 0.33);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(0.0, 0.0, -0.55);
    DrawRotor(0.35, 0.33);
    glPopMatrix();

    glEnable(GL_LIGHTING);
}

/*--------------------------------------------------------------------------
 * draw the quad rotor rotor (main == 1: main rotor else tail rotor)
 *--------------------------------------------------------------------------
 */
void DrawRotor(double outer_radius, double inner_radius)
{
    double x1 = 0;
    double x2 = 0;
    double z1 = inner_radius;
    double z2 = outer_radius;
    double k;
    double newx1, newz1, newx2, newz2;

    for (k = 0; k <= 2 * PI; k += PI / 12) {
        newx1 = outer_radius * sin(k);
        newz1 = outer_radius * cos(k);
        newx2 = inner_radius * sin(k);
        newz2 = inner_radius * cos(k);
        glBegin(GL_POLYGON);
        glVertex3f(x2, 0.0, z2);
        glVertex3f(x1, 0.0, z1);
        glVertex3f(newx1, 0.0, newz1);
        glVertex3f(newx2, 0.0, newz2);
        glEnd();
        x1 = newx1;
        x2 = newx2;
        z1 = newz1;
        z2 = newz2;
    }
}

/*--------------------------------------------------------------------------
 * resize callback
 *--------------------------------------------------------------------------
 */
void resize(GLsizei w, GLsizei h)
{
    h = (h ? h : 1);
    w = (w ? w : 1);
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(30.0, (GLfloat) w / (GLfloat) h, 1.0, 1000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

/*--------------------------------------------------------------------------
 * redraw callback
 *--------------------------------------------------------------------------
 */
void redraw()
{
    DrawScene(camloc, qrstate);
    glutSwapBuffers();
}

/*--------------------------------------------------------------------------
 * initialize GL stuff
 *--------------------------------------------------------------------------
 */
void    init_disp(int argc, char **argv)
{
    /* initialize GLUT stuff
     */
    glutInit(&argc, argv);
    glutInitWindowPosition(LOC_X, LOC_Y);
    glutInitWindowSize(WIDTH, HEIGHT);
    // double buffering, als used depth buffer, BTW, RGBA is default
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH);
    glutCreateWindow("QR Simulator -- Embedded Systems Lab, TU Delft");

    /* register call back functions
     */
    glutDisplayFunc(redraw);
    glutReshapeFunc(resize);
    glutIdleFunc(simulate);
    glutKeyboardFunc(keyb);
    glutSpecialFunc(skeyb);

    /* initialize GL stuff
     */
    glShadeModel(GL_FLAT);
    glClearColor(0.49, 0.62, 0.75, 0.0);
    glClearDepth(1.0f);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glDisable(GL_BLEND);
    glDisable(GL_ALPHA_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(30.0, (GLfloat) WIDTH / (GLfloat) HEIGHT, 1.0, 1000.0);
    glMatrixMode(GL_MODELVIEW);

    /* init camera location
     */
    camloc.view = VIEW_STATIONARY;
    camloc.tripod[0] = 1.0;
    camloc.tripod[1] = 4.0;
    camloc.tripod[2] = 10.0;
}

/*--------------------------------------------------------------------------
 * start GL loop
 *--------------------------------------------------------------------------
 */
void    start_disp()
{
    glutMainLoop();
}
