/*--------------------------------------------------------------------------
 *
 * qrcont -- Simple quad rotor autopilot based on P controllers
 *           Arjan van Gemund
 *           Embedded Software Lab, TU Delft
 *
 *--------------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>

#include "qrcont.h"

int printcount = 0;


void    rotate_3d(double x1, double y1, double z1, double phi, double theta,
                  double psi, double *x2, double *y2, double *z2);


/*--------------------------------------------------------------------------
 * quad rotor controller
 *--------------------------------------------------------------------------
 */
void    cont_nextstate(userstate_t *userstate,
                       qrstate_t *qrstate,
                       int print_state)
{
    double  sp_z, sp_phi, sp_theta, sp_u, sp_v, sp_w, sp_p, sp_q, sp_r;
    double  a_lift, a_roll, a_pitch, a_yaw;
    double  a1, a2, a3, a4;
    double  oo1, oo2, oo3, oo4;

    /* select control scenario
     */
    a_lift = a_roll = a_pitch = a_yaw = 0; // safe default
    switch (userstate->control_mode) {

    case 0: /* no control
             */
        a_lift = userstate->lift;
        a_roll = userstate->roll;
        a_pitch = userstate->pitch;
        a_yaw = userstate->yaw;
        break;

    case 1: /* lift control using z
             * 1 P ctl only -> oscillation (z = int(int(lift))
             * so need cascaded P control (or PD control)
             */
        sp_z = qrstate->z_at_gnd - userstate->lift; // setpoint altitude
        a_lift = 10 * (sp_z - qrstate->z);
        a_lift = - a_lift; // pos lift -> neg z
        a_roll = userstate->roll;
        a_pitch = userstate->pitch;
        a_yaw = userstate->yaw;
        break;

    case 2: /* lift control using z
             * now with cascaded P control
             */
        sp_z = qrstate->z_at_gnd - userstate->lift; // setpoint altitude
        sp_w = 10 * (sp_z - qrstate->z);
        a_lift = 10 * (sp_w - qrstate->w);
        a_lift = - a_lift; // pos lift -> neg z
        a_roll = userstate->roll;
        a_pitch = userstate->pitch;
        a_yaw = userstate->yaw;
        break;

    case 3: /* add yaw control
             * yaw uses rate setpoint so 1 P controller
             */
        sp_z = qrstate->z_at_gnd - userstate->lift;
        sp_w = 10 * (sp_z - qrstate->z);
        a_lift = 20 * (sp_w - qrstate->w);
        a_lift = - a_lift; // pos lift -> neg z
        a_roll = userstate->roll;
        a_pitch = userstate->pitch;
        sp_r = 5 * userstate->yaw; // setpoint is angular rate
        a_yaw = 5 * (sp_r - qrstate->r);
        break;

    case 4: /* add roll & pitch control
             * 1 P ctl so oscillation
             */
        sp_z = qrstate->z_at_gnd - userstate->lift;
        sp_w = 10 * (sp_z - qrstate->z);
        a_lift = 20 * (sp_w - qrstate->w);
        a_lift = - a_lift; // pos lift -> neg z
        sp_phi = userstate->roll; // setpoint is angle
        a_roll = 10 * (sp_phi - qrstate->phi);
        sp_theta = userstate->pitch;
        a_pitch = 10 * (sp_theta - qrstate->theta);
        sp_r = 5 * userstate->yaw;
        a_yaw = 5 * (sp_r - qrstate->r);
        break;

    case 5: /* add roll & pitch control
             * now with cascaded P ctl
             */
        sp_z = qrstate->z_at_gnd - userstate->lift;
        sp_w = 10 * (sp_z - qrstate->z);
        a_lift = 20 * (sp_w - qrstate->w);
        a_lift = - a_lift; // pos lift -> neg z
        sp_phi = userstate->roll;
        sp_p = 10 * (sp_phi - qrstate->phi);
        a_roll = 10 * (sp_p - qrstate->p);
        sp_theta = userstate->pitch;
        sp_q = 10 * (sp_theta - qrstate->theta);
        a_pitch = 10 * (sp_q - qrstate->q);
        sp_r = 2 * userstate->yaw;
        a_yaw = 5 * (sp_r - qrstate->r);
        break;

    case 6: /* add u/v speed control on top of roll/pitch control
             * Note: QR should have speed sensors for this ..
             */
        sp_z = qrstate->z_at_gnd - userstate->lift;
        sp_w = 10 * (sp_z - qrstate->z);
        a_lift = 20 * (sp_w - qrstate->w);
        a_lift = - a_lift; // pos lift -> neg z
        sp_v = 10 * userstate->roll;
        sp_phi = 0.1 * (sp_v - qrstate->v);
        sp_p = 10 * (sp_phi - qrstate->phi);
        a_roll = 10 * (sp_p - qrstate->p);
        sp_u = - 10 * userstate->pitch; // nose down is pos u
        sp_theta = - 0.1 * (sp_u - qrstate->u); // idem
        sp_q = 10 * (sp_theta - qrstate->theta);
        a_pitch = 10 * (sp_q - qrstate->q);
        sp_r = 5 * userstate->yaw;
        a_yaw = 5 * (sp_r - qrstate->r);
        break;

    case 7:
        break;
    case 8:
        break;
    case 9:
        break;
    default:
        assert(0);
        break;
    }

    /* we only want positive lift so clip lift
     */
    if (a_lift < 0) a_lift = 0;

    /* map lift, roll, pitch, yaw to rotor actuator vars ai
     * so we need to solve for ai:
     *
     * b * (o1*o1 + o2*o2 + o3*o3 + o4*o4) = lift;
     * b * (- o2*o2 + o4*o4) = roll;
     * b * (o1*o1 - o3*o3) = pitch;
     * d * (- o1*o1 + o2*o2 - o3*o3 + o4*o4) = yaw;
     *
     * let ooi be oi*oi. then we must solve
     *
     * [  1  1  1  1  ] [ oo1 ]   [lift/b]
     * [  0 -1  0  1  ] [ oo2 ]   [roll/b]
     *                          =
     * [ -1  0  1  0  ] [ oo3 ]   [pitch/b]
     * [ -1  1 -1  1  ] [ oo4 ]   [yaw/d]
     *
     * the inverse matrix is
     *
     * [  1  0  2 -1  ]
     * [  1 -2  0  1  ]
     *                  * 1/4
     * [  1  0 -2 -1  ]
     * [  1  2  0  1  ]
     *
     * so with b = d = 1 we have
     */
    oo1 = (a_lift + 2 * a_pitch - a_yaw) / 4;
    oo2 = (a_lift - 2 * a_roll + a_yaw) / 4;
    oo3 = (a_lift - 2 * a_pitch - a_yaw) / 4;
    oo4 = (a_lift + 2 * a_roll + a_yaw) / 4;

    /* clip ooi as rotors only provide prositive thrust
     */
    if (oo1 < 0) oo1 = 0;
    if (oo2 < 0) oo2 = 0;
    if (oo3 < 0) oo3 = 0;
    if (oo4 < 0) oo4 = 0;

    /* with ai = oi it follows
     */
    a1 = sqrt(oo1);
    a2 = sqrt(oo2);
    a3 = sqrt(oo3);
    a4 = sqrt(oo4);

    /* print controller and quad rotor state
     */
    printcount++;
    if (print_state && (printcount % 10 == 0)) { // 10
        printf("%.1f ", qrstate->t);
        printf("%d ", userstate->control_mode);
        if (userstate->control_mode == CONTROL_NONE) {
            printf("a1 %.1f  a2 %.1f  a3 %.1f  a4 %.1f",
                   a1,a2,a3,a4);
        }
        else {
            printf("ul %.1f  l %.1f  r %.1f  p %.1f  y %.1f",
                   userstate->lift,a_lift,a_roll,a_pitch,a_yaw);
        }
        printf("  ");
        printf("z %.2f  psi %.2f  the %.2f  phi %.2f",
               qrstate->z - qrstate->z_at_gnd,qrstate->psi,
               qrstate->theta,qrstate->phi);
        printf("  ");
        printf("u %.2f  v %.2f  w %.2f",
               qrstate->u,qrstate->v,qrstate->w);
        printf("\n");
    }

    /* connect controller output to quad rotor actuators
     */
    qrstate->a1 = a1;
    qrstate->a2 = a2;
    qrstate->a3 = a3;
    qrstate->a4 = a4;
}

/*--------------------------------------------------------------------------
 * coordinates (x2,y2,z2) of (x1,y1,z1) when rotating frame (phi,theta,psi)
 *--------------------------------------------------------------------------
 */
void    rotate_3d(double x1, double y1, double z1, double phi, double theta,
                  double psi, double *x2, double *y2, double *z2)
{
    double  sinphi, sintheta, sinpsi, cosphi, costheta, cospsi;

    /* first optimize a bit
     */
    sinphi = sin(phi); sintheta = sin(theta); sinpsi = sin(psi);
    cosphi = cos(phi); costheta = cos(theta); cospsi = cos(psi);
    if (costheta == 0) {
        printf("rotate_3d: singularity in Euler angles: cos(theta) = 0\n");
        exit(0);
    }
    *x2 = costheta * cospsi * x1 + costheta * sinpsi * y1 - sintheta * z1;
    *y2 = (sinphi * sintheta * cosphi - cosphi * sinpsi) * x1 +
        (sinphi * sintheta * sinpsi + cosphi * cospsi) * y1 + sinphi * costheta * z1;
    *z2 = (cosphi * sintheta * cospsi + sinphi * sinpsi) * x1 +
        (cosphi * sintheta * sinpsi - sinphi * cospsi) * y1 + cosphi * costheta * z1;
}
