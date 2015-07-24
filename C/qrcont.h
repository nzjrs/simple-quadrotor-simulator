/*--------------------------------------------------------------------------
 *
 * qrcont -- simple quad rotor controller
 *           Arjan van Gemund
 *           Embedded Software Lab, TU Delft
 *
 *--------------------------------------------------------------------------
 */

#ifndef _QR_CONT_H_
#define _QR_CONT_H_

#include "qrmod.h"

typedef enum {
    CONTROL_NONE,
    CONTROL_LIFT,
    CONTROL_LIFT_CP,
    CONTROL_YAW_RATE,
    CONTROL_ROLL_PITCH,
    CONTROL_ROLL_PITCH_CP,
    CONTROL_SPEED,
} controlmode_t;

/* user setpoint state
 */
typedef struct {

    double  lift;
    double  yaw;
    double  pitch;
    double  roll;
    controlmode_t control_mode;

} userstate_t;

extern void cont_nextstate(userstate_t *userstate,
                           qrstate_t *qrstate,
                           int print_state);

#endif /* _QR_CONT_H_ */
