/*--------------------------------------------------------------------------
 *
 * qrsim -- simple quad rotor simulator
 *
 *          (c) Embedded Software Lab, TU Delft
 *
 *--------------------------------------------------------------------------
 */

#ifndef _QR_SIM_H_
#define _QR_SIM_H_

/* camloc_t: some camera viewpoint presets
 * and static tripod location
 */
typedef enum {
    VIEW_STATIONARY,
    VIEW_WALK_BEHIND,
    VIEW_FLY_BEHIND,
    VIEW_COCKPIT,
    VIEW_NORTH_UP,
    VIEW_TRACK_UP
} viewpoint_t;

typedef struct {
    viewpoint_t view;       /* camera attitude */
    float       tripod[3];  /* camera fixed location */
} camloc_t;

/* kalman state
 */
typedef struct {

    double  phi[10];
    double  p[10];
    double  bias[10];

} kalmanstate_t;

/* check if JS present
 */
int no_js;

#ifndef PI
#define PI 3.14159265
#endif /*  */

#endif /* _QR_SIM_H */
