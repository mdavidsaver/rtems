/*
 *  Rate Monotonic Manager
 *
 *
 *  COPYRIGHT (c) 1989, 1990, 1991, 1992, 1993, 1994.
 *  On-Line Applications Research Corporation (OAR).
 *  All rights assigned to U.S. Government, 1994.
 *
 *  This material may be reproduced by or for the U.S. Government pursuant
 *  to the copyright license under the clause at DFARS 252.227-7013.  This
 *  notice must appear in all copies of this file and its derivatives.
 *
 *  $Id$
 */

#include <rtems/system.h>
#include <rtems/isr.h>
#include <rtems/object.h>
#include <rtems/ratemon.h>
#include <rtems/thread.h>

void _Rate_monotonic_Manager_initialization(
  unsigned32 maximum_periods
)
{
}

rtems_status_code rtems_rate_monotonic_create(
  rtems_name    name,
  Objects_Id   *id
)
{
  return( RTEMS_NOT_CONFIGURED );
}

rtems_status_code rtems_rate_monotonic_ident(
  rtems_name    name,
  Objects_Id   *id
)
{
  return( RTEMS_NOT_CONFIGURED );
}

rtems_status_code rtems_rate_monotonic_cancel(
  Objects_Id id
)
{
  return( RTEMS_NOT_CONFIGURED );
}

rtems_status_code rtems_rate_monotonic_delete(
  Objects_Id id
)
{
  return( RTEMS_NOT_CONFIGURED );
}

rtems_status_code rtems_rate_monotonic_period(
  Objects_Id        id,
  rtems_interval length
)
{
  return( RTEMS_NOT_CONFIGURED );
}

void _Rate_monotonic_Timeout(
  Objects_Id  id,
  void       *ignored
)
{
}

boolean _Rate_monotonic_Set_state(
Rate_monotonic_Control *the_period
)
{
  return( FALSE );
}
