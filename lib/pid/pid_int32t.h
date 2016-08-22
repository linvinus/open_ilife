#ifndef _PID_INT32T_H_
#define _PID_INT32T_H_

#include "ch.h"

/**
 * @brief Instance structure for the floating-point PID Control.
 */
typedef struct
{
  int32_t A0;          /**< The derived gain, A0 = Kp + Ki + Kd . */
  int32_t A1;          /**< The derived gain, A1 = -Kp - 2Kd. */
  int32_t A2;          /**< The derived gain, A2 = Kd . */
  int32_t state[3];    /**< The state array of length 3. */
  int32_t Kp;               /**< The proportional gain. */
  int32_t Ki;               /**< The integral gain. */
  int32_t Kd;               /**< The derivative gain. */
  uint32_t multiply;
} arm_pid_instance_int32t;



#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif

static inline int16_t arm_pid_int32t(
arm_pid_instance_int32t * S,
int16_t in)
{
  int32_t out;
  in <<= S->multiply;

  /* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]  */
  out = (S->A0 * in) +
    (S->A1 * S->state[0]) + (S->A2 * S->state[1]) + (S->state[2]);

  /* Update state */
  S->state[1] = S->state[0];
  S->state[0] = in;
  S->state[2] = out;

  out >>= S->multiply;

  /* return to application */
  return ((int16_t)out);

}

void arm_pid_init_int32t(
  arm_pid_instance_int32t * S,
  const int16_t Kp,
  const int16_t Ki,
  const int16_t Kd,
  const int16_t multiply,
  const int32_t resetStateFlag);
  
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* _PID_INT32T_H_ */
