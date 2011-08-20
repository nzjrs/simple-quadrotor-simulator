/*--------------------------------------------------------------------------
 *
 * qrmod -- Simple quad rotor model
 *          Arjan van Gemund 
 *          Embedded Software Lab, TU Delft
 *
 *  	    Euler angles: NASA standard aeroplane (psi-theta-phi)
 *
 *--------------------------------------------------------------------------
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "qrmod.h"

void qr_init(qrstate_t *qrstate, double z_at_gnd)
{
  	qrstate->z_at_gnd = z_at_gnd;	/* position coordinate (earth axis z) */

	qrstate->t = 0;		/* simulation time */

  	qrstate->u = 0;		/* airspeed (body axis x) */
  	qrstate->v = 0;		/* airspeed (body axis y) */
  	qrstate->w = 0;		/* airspeed (body axis z) */
  	qrstate->p = 0;		/* angular rotation speed phi (body axis x) */
  	qrstate->q = 0;		/* angular rotation speed theta (body axis y) */
  	qrstate->r = 0;		/* angular rotation speed psi (body axis z) */
  	qrstate->phi = 0;	/* roll angle (body axis x) */
  	qrstate->theta = 0;	/* pitch angle (body axis y) */
  	qrstate->psi = 0;	/* yaw angle (body axis z) */
  	qrstate->x = 0;		/* position coordinate (earth axis x) */
  	qrstate->y = 0;		/* position coordinate (earth axis y) */
  	qrstate->z = z_at_gnd;	/* position coordinate (earth axis z) */
  	qrstate->mx = 0;
  	qrstate->my = 0;

  	/* actuators 
  	 */
  	qrstate->a1 = 0;	/* rotor 1 */
  	qrstate->a2 = 0;	/* rotor 2 */
  	qrstate->a3 = 0;	/* rotor 3 */
  	qrstate->a4 = 0;	/* rotor 4 */
  	qrstate->leds = 0;	/* reset leds */
  	qrstate->tleds = 0;	/* reset led blink time */
	
  	/* sensors 
  	 */
  	qrstate->sp = 0;	/* gyro measrement of p */
  	qrstate->sq = 0;	/* gyro measrement of q */
  	qrstate->sr = 0;	/* gyro measrement of r */
  	qrstate->sx = 0;	/* acceleration (body axis x) */
  	qrstate->sy = 0;	/* acceleration (body axis y) */
  	qrstate->sz = 0;	/* acceleration (body axis z) */

    qrstate->sim_control.enable_gravity = 1;
    qrstate->sim_control.verbose = 0;
    qrstate->sim_control.pause = 0;
}

/*--------------------------------------------------------------------------
 * update earth orientation (Euler angles) from body angle increments
 * [Etkin page 104]
 *
 * Euler angle update mode
 * use_body_angles = 1: angle increments in terms of body frame (-> xformation)
 * else: angle increments in terms of earth frame (-> addition)
 *--------------------------------------------------------------------------
 */
void 	update_euler(qrstate_t *qrstate, 
		         double p, double q, double r, int use_body_angles)
{
	double phi, theta, psi;
	double d_phi, d_theta, d_psi;

	phi = qrstate->phi;
	theta = qrstate->theta;
	psi = qrstate->psi;

	if (use_body_angles) {
        /* Body angles */
		d_phi = p + (q * sin(phi) + r * cos(phi)) * tan(theta);
		d_theta = q * cos(phi) - r * sin(phi);
		d_psi = (q * sin(phi) + r * cos(phi)) / cos(theta);
	}
	else {
        /* Euler angles */
		d_phi = p;
		d_theta = q;
		d_psi = r;
	}

	qrstate->phi += d_phi;
	qrstate->theta += d_theta;
	qrstate->psi += d_psi;
}

/*--------------------------------------------------------------------------
 * quad rotor model with full dynamics and kinematics equations
 *--------------------------------------------------------------------------
 */
void qr_nextstate(qrstate_t *qrstate, double DT)
{
	double t, tleds;
	double a1, a2, a3, a4;
	int leds;
	double X, Y, Z;
	double L, M, N;
	double u, v, w, du, dv, dw;
	double p, q, r, dp, dq, dr;
	double phi, theta, psi, dphi, dtheta, dpsi;
	double x, y, z, dx, dy, dz;
	double sinphi, sintheta, sinpsi, cosphi, costheta, cospsi, tantheta;
	double a_, b_, c_, d_, e_, f_;

	double o1, o2, o3, o4; // rotor speed omega
	double m, b, d, g;
	double Ix, Iy, Iz, Izx;

	/* gravity force
	 */
	if (qrstate->sim_control.enable_gravity)
		g = 10; /* 9.81; */
	else
		g = 0;

	/* quad rotor constants (SI units)
	 */
	b = 1.0;
	d = 10.0; // 10*b: avoid lots of Z thrust when yawing
	m = 1.0;
	Ix = Iy = 1.0;
	Iz = 2.0;
	Izx = 0.0;

	/* copy from struct
	 */
	t = qrstate->t;
	a1 = qrstate->a1;
	a2 = qrstate->a2;
	a3 = qrstate->a3;
	a4 = qrstate->a4;
	leds = qrstate->leds;
	tleds = qrstate->tleds;

	u = qrstate->u;
	v = qrstate->v;
	w = qrstate->w;
	p = qrstate->p;
	q = qrstate->q;
	r = qrstate->r;
	phi = qrstate->phi;
	theta = qrstate->theta;
	psi = qrstate->psi;
	x = qrstate->x;
	y = qrstate->y;
	z = qrstate->z;

	/* optimize a bit
	 */
	sinphi = sin(phi); sintheta = sin(theta); sinpsi = sin(psi);
	cosphi = cos(phi); costheta = cos(theta); cospsi = cos(psi);
	if (costheta == 0) {
		printf("singularity in Euler angles: cos(theta) = 0\n");
		exit(0);
	}
	tantheta = sintheta / costheta;




	/* first part of the quad rotor model (specific):
	 * convert actuators signals to forces and moments
	 * (in terms of body axes)
	 *
	 * NOTE: OVERLY SIMPLE FOR NOW!
	 */


	/* clip rotor thrusts
	 */
	// if (a1 < 0) a1 = 0; 
	// if (a1 > 100) a1 = 100;
	// if (a2 < 0) a2 = 0; 
	// if (a2 > 100) a2 = 100;
	// if (a3 < 0) a3 = 0; 
	// if (a3 > 100) a2 = 100;
	// if (a4 < 0) a4 = 0; 
	// if (a4 > 100) a2 = 100;

	/* compute rotor speed
	 */
	o1 = a1; // front, turning clockwise
	o2 = a2; // starboard, turning counter-clockwise
	o3 = a3; // aft, turning clockwise
	o4 = a4; // port, turning counter-clockwise

	/* compute longitudal thrust (body axis)
	 */
	X = 0;

	/* compute lateral thrust (body axis)
	 */
	Y = 0;

	/* compute vertical thrust (body axis)
	 * function of 4 rotors
	 */
	Z = - b * (o1*o1 + o2*o2 + o3*o3 + o4*o4);

	/* compute roll moment (body axis)
	 */
	L = b * (o4*o4 - o2*o2); 

	/* compute pitch moment (body axis)
	 */
	M = b * (o1*o1 - o3*o3); 

	/* compute yaw moment (body axis)
	 */
	N = d * (o2*o2 + o4*o4 - o1*o1 - o3*o3); 
	
	/* trace control data
	 */ 
	if (qrstate->sim_control.verbose) {
		printf("t    a1    a2    a3    a4       X    Y    Z      L    M    N\n");
		printf("%.1f  %.1f  %.1f  %.1f  %.1f    %.1f  %.1f  %.1f     %.1f  %.1f  %.1f\n",
			t,a1,a2,a3,a4,X,Y,Z,L,M,N);
	}




	/* second part of the model (generic):
	 * simulate a free body in space [Etkin page 104]
	 * given the applied forces (X,Y,Z) and moments (L,M,N)
	 * (simple Euler integration)
	 */

	/* compute accelerations (body axes)
	 */
	du = (X / m) - g * sintheta - q * w + r * v;
	dv = (Y / m) + g * costheta * sinphi - r * u + p * w;
	dw = (Z / m) + g * costheta * cosphi - p * v + q * u;

	/* compute angular accelerations (body axes)
	 * we must also solve a system of 2 eqns for dp and dr
	 * we use the solution of
 	 * ax + by = e
	 * cx + dy = f
	 * which is (e.g., Cramer's rule)
	 * x = (de - bf) / (ad - bc)
	 * y = (af - ce) / (ad - bc)
	 * to compute dp and dr
	 */
	a_ = Ix; b_ = -Izx; e_ = L - q * r * (Iz - Iy) + Izx * p * q;
	c_ = -Izx; d_ = Iz; f_ = N - p * q * (Iy - Ix) - Izx * q * r;
	if (a_*d_ - b_*c_ == 0) {
		printf("singularity in (p,q) computation: zero determinant\n");
		exit(0);
	}
	dp = (d_ * e_ - b_ * f_) / (a_ * d_ - b_ * c_);
	dq = (M - r * p * (Ix - Iz) + Izx * (p * p - r * r)) / Iy;
	dr = (c_ * e_ - a_ * f_) / (b_ * c_ - a_ * d_);

	/* compute angular velocities (Euler angles)
	 */
	dphi = p + (q * sinphi + r * cosphi) * tantheta;
	dtheta = q * cosphi - r * sinphi;
	dpsi = (q * sinphi + r * cosphi) / costheta;

	/* compute velocities (earth axes)
	 */
	dx = u * costheta * cospsi + 
	     v * (sinphi * sintheta * cospsi - cosphi * sinpsi) +
	     w * (cosphi * sintheta * cospsi + sinphi * sinpsi);
	dy = u * costheta * sinpsi +
	     v * (sinphi * sintheta * sinpsi + cosphi * cospsi) +
	     w * (cosphi * sintheta * sinpsi - sinphi * cospsi);
	dz = -u * sintheta + 
	     v * sinphi * costheta +
	     w * cosphi * costheta;
	     
	/* integrate the system of equations
	 */
	u += du * DT; v += dv * DT; w += dw * DT;
	p += dp * DT; q += dq * DT; r += dr * DT;
	phi += dphi * DT; theta += dtheta * DT; psi += dpsi * DT;
	x += dx * DT; y += dy * DT; z += dz * DT;


	/* don't go through the ground
	 */
	if (z > qrstate->z_at_gnd) {
		z = qrstate->z_at_gnd;
		w = 0; // avoid integration windup
	}




	if (qrstate->sim_control.verbose) {
		printf("t   u    v    w       p    q    r       phi  thet psi     x    y    z\n");
		printf("%.1f  %.1f  %.1f  %.1f     %.1f  %.1f  %.1f     %.1f  %.1f  %.1f     %.1f  %.1f  %.1f\n",
			t,u,v,w,p,q,r,phi,theta,psi,x,y,z);
	}
	if (qrstate->sim_control.pause) {
		getchar(); 
		qrstate->sim_control.pause = ! qrstate->sim_control.pause;
	}

	/* update simulation time
	 */
	t += DT;

	/* update blinking led[0]
	 */
	if (t - tleds > 0.5) {
		leds = (leds ^ 0x0001);
		tleds = t;
	}
	
	/* write back to struct
	 */
	qrstate->t = t;
	qrstate->u = u;
	qrstate->v = v;
	qrstate->w = w;
	qrstate->p = p;
	qrstate->q = q;
	qrstate->r = r;
	qrstate->phi = phi;
	qrstate->theta = theta;
	qrstate->psi = psi;
	qrstate->x = x;
	qrstate->y = y;
	qrstate->z = z;
	qrstate->leds = leds;
	qrstate->tleds = tleds;
}

