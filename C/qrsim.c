/*--------------------------------------------------------------------------
 *
 * qrsim -- simple quad rotor simulator
 *
 *          Arjan J.C. van Gemund
 *      Embedded Software Lab, TU Delft
 *
 *     NOTE: this code is a hack by guys that didn't have the time
 *           to adhere to all good software engineering principles
 *       i.e., you should do better!
 *--------------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <GL/glut.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include "js.h"

#include "qrsim.h"

#include "qrcont.h"
#include "qrmod.h"
#include "qrdisp.h"

/* define what is ground level z value for QR
 */
#define Z_AT_GND    (-0.3)
#define DT  0.0004  /* simultion time step (0.002 is ub) */

userstate_t userstate;
camloc_t    camloc;
qrstate_t   qrstate;

#ifndef PI
#define PI 3.14159265
#endif

static float    RAD2DEG = 180 / PI;

/* simulation control (EULER mode, PAUSE, PRINT statistics)
 */
int sim_mode_adjust_body_angles = 0;
int sim_mode_print = 0;

/* simulation display counter to speedup things
 */
int simulations;


#define JS_DEV      "/dev/input/js0"
#define JS_SENSITIVITY  10000.0
int             fd,errno;
struct js_event     js;
int         axis[6];
int         button[12];
double          js_roll, js_pitch, js_yaw, js_lift;

double          kb_roll, kb_pitch, kb_yaw, kb_lift;


/*--------------------------------------------------------------------------
 * signon message
 *--------------------------------------------------------------------------
 */
void menu(void) {
    printf("\n");
    printf("Pilot control keys:\n");
    printf("a: increase lift\n");
    printf("z: decrease lift\n");
    printf("w: increase yaw\n");
    printf("q: decrease yaw\n");
    printf("<up>: increase pitch\n");
    printf("<dn>: decrease pitch\n");
    printf("<lt>: decrease roll\n");
    printf("<rt>: increase roll\n");
    printf("\n");
    printf("Simulation control keys:\n");
    printf("^C, <esc>: exit\n");
    printf("^G: gravity on - off [on]\n");
    printf("^M, <ret>: next control mode 0, 1, .., 6, 0 .. [0]\n");
    printf("^P: print control stats mode on - off [off]\n");
    printf("^R: reset simulation\n");
    printf("^S: freeze simulation on - off [off]\n");
    printf("^T: Euler angle control on - off [on]\n");
    printf("^V: print model stats on - off [off]\n");
    printf("0: force QR x,y,z to zero\n");
    printf("\n");
    printf("Display control keys:\n");
    printf("1, <F1>: select viewpoint stationary\n");
    printf("2, <F2>: select viewpoint fly behind\n");
    printf("3, <F3>: select viewpoint walk behind\n");
    printf("4, <F4>: select viewpoint cockpit\n");
    printf("5, <F5>: select viewpoint north up\n");
    printf("6, <F6>: select viewpoint track up\n");
    printf("8: zoom in\n");
    printf("9: zoom out\n");
    printf("\n");
    printf("Simulation model override keys:\n");
    printf("o: force decrease phi\n");
    printf("p: force increase phi\n");
    printf("k: force decrease theta\n");
    printf("l: force increase theta\n");
    printf("n: force decrease psi\n");
    printf("m: force increase psi\n");
    printf("x: force decrease x\n");
    printf("c: force increase x\n");
    printf("y: force decrease y\n");
    printf("u: force increase y\n");
    printf("\n");
    printf("Any other key generates this menu\n");
    printf("\n");
}

/*--------------------------------------------------------------------------
 * initialize joystick
 *--------------------------------------------------------------------------
 */
void js_init(void) {
    int i;

    button[0] = 0;
    for (i = 0; i < 3; i++)
        axis[i]=0;
    axis[3] = 32767;

    no_js = 1;
    if ((fd = open(JS_DEV, O_RDONLY)) < 0) {
        fprintf(stderr,"warning: error opening JS\n");
        perror("jstest");
        no_js = 1;
        return;
    }
    fcntl(fd, F_SETFL, O_NONBLOCK);
    read(fd, &js, sizeof(struct js_event));
    read(fd, &js, sizeof(struct js_event));
    read(fd, &js, sizeof(struct js_event));
}

/*--------------------------------------------------------------------------
 * initialize userstate signals (input to controller)
 *--------------------------------------------------------------------------
 */
void user_init(userstate_t *userstate, controlmode_t control_mode) {
    kb_lift = kb_yaw = kb_pitch = kb_roll = 0;
    js_lift = js_yaw = js_pitch = js_roll = 0;
    userstate->lift = 0;
    userstate->yaw = 0;
    userstate->pitch = 0;
    userstate->roll = 0;
    userstate->control_mode = control_mode;
}

/*--------------------------------------------------------------------------
 * normal keys callback function
 *--------------------------------------------------------------------------
 */
void keyb(unsigned char key, int x, int y) {
    x = x;
    y = y;
    int use_body_angles = sim_mode_adjust_body_angles;
    switch (key) {

        /* simulator control
         */
    case 03: /* ^C: exit */
        exit(0);
        break;
    case 07: /* ^G: toggle gravity */
        qrstate.sim_control.enable_gravity = ! qrstate.sim_control.enable_gravity;
        if (qrstate.sim_control.enable_gravity)
            printf("gravity on\n");
        else
            printf("gravity off\n");
        break;
    case 13: /* ^M: change control mode + QR reset */
        qr_init(&qrstate, Z_AT_GND);
        user_init(&userstate, (userstate.control_mode + 1) % 10);
        printf("qrsim controller mode %d\n",userstate.control_mode);
        break;
    case 16: /* ^P: print statistics toggle */
        sim_mode_print = ! sim_mode_print;
        break;
    case 18: /* ^R: reset */
        qr_init(&qrstate, Z_AT_GND);
        user_init(&userstate, userstate.control_mode);
        break;
    case 19: /* ^S: freeze simulation toggle */
        qrstate.sim_control.pause = ! qrstate.sim_control.pause;
        break;
    case 20: /* ^T: toggle Euler angle update mode */
        sim_mode_adjust_body_angles = ! sim_mode_adjust_body_angles;
        if (sim_mode_adjust_body_angles)
            printf("Body angles\n");
        else
            printf("Euler angles\n");
        break;
    case 22: /* ^V: toggle trace generator */
        qrstate.sim_control.verbose = ! qrstate.sim_control.verbose;
        break;
    case 27: /* ESC: exit */
        exit(0);
        break;

    case '0': /* override - reset x/y/x */
        qrstate.x = 0;
        qrstate.y = 0;
        if (qrstate.z > -0.3) qrstate.z = -0.3;
        if (qrstate.z < -3.3) qrstate.z = -3.3;
        break;

        /* view control
         */
    case '1':
        camloc.view = VIEW_STATIONARY;
        break;
    case '2':
        camloc.view = VIEW_FLY_BEHIND;
        break;
    case '3':
        camloc.view = VIEW_WALK_BEHIND;
        break;
    case '4':
        camloc.view = VIEW_COCKPIT;
        break;
    case '5':
        camloc.view = VIEW_NORTH_UP;
        break;
    case '6':
        camloc.view = VIEW_TRACK_UP;
        break;
    case '8': /* zoom in */
        camloc.tripod[0] *= 0.99;
        camloc.tripod[1] *= 0.99;
        camloc.tripod[2] *= 0.99;
        break;
    case '9': /* zoom out */
        camloc.tripod[0] *= 1.01;
        camloc.tripod[1] *= 1.01;
        camloc.tripod[2] *= 1.01;
        break;

        /* quad rotor control
         */
    case 'a': /* increase lift */
        kb_lift += 0.5;
        break;
    case 'z': /* decrease lift */
        kb_lift -= 0.5;
        break;
    case 'w': /* increase yaw */
        kb_yaw += 0.2;
        break;
    case 'q': /* decrease yaw */
        kb_yaw -= 0.2;
        break;
    case 'o': /* override - phi */
        update_euler(&qrstate,-2/RAD2DEG,0,0,use_body_angles);
        break;
    case 'p': /* override - phi */
        update_euler(&qrstate,+2/RAD2DEG,0,0,use_body_angles);
        break;
    case 'k': /* override - theta */
        update_euler(&qrstate,0,-2/RAD2DEG,0,use_body_angles);
        break;
    case 'l': /* override - theta */
        update_euler(&qrstate,0,+2/RAD2DEG,0,use_body_angles);
        break;
    case 'n': /* override - psi */
        update_euler(&qrstate,0,0,-2/RAD2DEG,use_body_angles);
        break;
    case 'm': /* override - psi */
        update_euler(&qrstate,0,0,+2/RAD2DEG,use_body_angles);
        break;
    case 'x': /* override - x axis */
        qrstate.x -= 0.1;
        break;
    case 'c': /* override - x axis */
        qrstate.x += 0.1;
        break;
    case 'y': /* override - y axis */
        qrstate.y -= 0.1;
        break;
    case 'u': /* override - y axis */
        qrstate.y += 0.1;
        break;
    default: menu();
    }
}

/*--------------------------------------------------------------------------
 * special function keys callback function
 *--------------------------------------------------------------------------
 */
void skeyb(int key, int x, int y) {
    x = x;
    y = y;
    switch (key) {

        /* view control
         */
    case GLUT_KEY_F1:
        camloc.view = VIEW_STATIONARY;
        break;
    case GLUT_KEY_F2:
        camloc.view = VIEW_FLY_BEHIND;
        break;
    case GLUT_KEY_F3:
        camloc.view = VIEW_WALK_BEHIND;
        break;
    case GLUT_KEY_F4:
        camloc.view = VIEW_COCKPIT;
        break;
    case GLUT_KEY_F5:
        camloc.view = VIEW_NORTH_UP;
        break;
    case GLUT_KEY_F6:
        camloc.view = VIEW_TRACK_UP;
        break;

        /* quad rotor control
         */
    case GLUT_KEY_UP: /* neg pitch */
        kb_pitch -= 0.1;    // nose down
        break;
    case GLUT_KEY_DOWN: /* pos pitch */
        kb_pitch += 0.1;    // nose up
        break;
    case GLUT_KEY_LEFT: /* neg roll */
        kb_roll -= 0.1;         // bank port
        break;
    case GLUT_KEY_RIGHT: /* pos roll */
        kb_roll += 0.1;         // bank starboard
        break;
    default: menu();
    }
}

/*--------------------------------------------------------------------------
 * simulation callback -- is called when glut is idle (no keys, etc.)
 *--------------------------------------------------------------------------
 */
void simulate() {
    if (!qrstate.sim_control.pause) {
        cont_nextstate(&userstate,&qrstate,sim_mode_print);
        qr_nextstate(&qrstate,DT);
        simulations++;
        if (simulations % 10 == 0) {
            redraw();
            simulations = 0;
        }

        /* check for JS events
         */
        while (! no_js && read(fd,&js,sizeof(struct js_event)) ==
               sizeof(struct js_event))  {
            switch(js.type & ~JS_EVENT_INIT) {
            case JS_EVENT_BUTTON:
                button[js.number] = js.value;
                break;
            case JS_EVENT_AXIS:
                axis[js.number] = js.value;
                break;
            }
            if (errno != EAGAIN) {
                perror("\njs: error reading (EAGAIN)");
                exit (1);
            }
        }
        js_roll = axis[0] / JS_SENSITIVITY;
        js_pitch = axis[1] / JS_SENSITIVITY;
        js_yaw += (axis[2] / JS_SENSITIVITY) * DT;
        js_lift = -(axis[3] - 32767) / JS_SENSITIVITY;
        // printf("%3f %3f %3f %3f\n",js_roll,js_pitch,js_yaw,js_lift);

        /* integrate keyboard and joystick
         */

        userstate.roll = kb_roll + js_roll;
        userstate.pitch = kb_pitch + js_pitch;
        userstate.yaw = kb_yaw + js_yaw;
        userstate.lift = kb_lift + js_lift;

    }
}

/*--------------------------------------------------------------------------
 * initialize and enter main event loop
 *--------------------------------------------------------------------------
 */
int main(int argc, char **argv) {
    /* initialize openGL stuff
     */
    printf("QRSIM (c) Embedded Systems Lab, TU Delft\n");
    menu();
    printf("qrsim control mode %d\n",userstate.control_mode);

    init_disp(argc, argv);

    /* initialize quad rotor stuff
     */
    qr_init(&qrstate, Z_AT_GND);
    user_init(&userstate, CONTROL_NONE);
    js_init();

    /* start simulation
     */
    simulations = 0;
    start_disp();
    return 0;
}
