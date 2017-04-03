/************************************************************
 *
 * 1-degree of freedom equation of motion (y-direction)
 * compiled UDF
 *
 ************************************************************/
#include "udf.h"

static real v_prev = 0.0;

DEFINE_CG_MOTION(piston, dt, vel, omega, time, dtime)
{
  Thread *t;
  Domain *d;
  face_t f;
  real NV_VEC (A);
  real NV_VEC (up);
  real force, dv;

  /* reset velocities */
  NV_S (vel, =, 0.0);
  NV_D (up, =, 0.0, 1.0, 0.0);
  NV_S (omega, =, 0.0);

  if (!Data_Valid_P ())
    return;

  t = DT_THREAD (dt);

  /* compute pressure force on body by looping through all faces */
  force = 0.0;
  begin_f_loop (f, t)
    {
      F_AREA (A, f, t);
      NV_V(A, *=, up);
      force += F_P (f, t) * NV_MAG (A);
    }
  end_f_loop (f, t)

  /* compute change in velocity, i.e., dv = F * dt / mass
     velocity update using explicit Euler formula */

  Message ("oForce%f dt%f\n", force, dtime);

  force = force - 9.81*0.110 - v_prev*0.6; //Add explicit forces, edit mass

  dv = dtime * force / 0.110;  //Mass change this
  v_prev += dv;
  Message ("time = %f, y_vel = %f, force = %f\n", time, v_prev, 
  force);

  /* set y-component of velocity */
  vel[1] = v_prev;
}