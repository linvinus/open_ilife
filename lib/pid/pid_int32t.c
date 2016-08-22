
#include "pid_int32t.h"

void arm_pid_init_int32t(
  arm_pid_instance_int32t * S,
  const int16_t Kp,
  const int16_t Ki,
  const int16_t Kd,
  const int16_t multiply,
  const int32_t resetStateFlag)
{
  S->multiply = multiply;
  S->Kp = (int32_t)Kp << multiply;
  S->Ki = (int32_t)Ki << multiply;
  S->Kd = (int32_t)Kd << multiply;
  

  /* Derived coefficient A0 */
  S->A0 = S->Kp + S->Ki + S->Kd;

  /* Derived coefficient A1 */
  S->A1 = (-S->Kp) - ((int32_t) 2 * S->Kd);

  /* Derived coefficient A2 */
  S->A2 = S->Kd;

  /* Check whether state needs reset or not */
  if(resetStateFlag)
  {
    /* Clear the state buffer.  The size will be always 3 samples */
    memset(S->state, 0, 3u * sizeof(int32_t));
  }

}
