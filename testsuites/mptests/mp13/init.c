/*  Init
 *
 *  This routine is the initialization task for this test program.
 *  It is a user initialization task and has the responsibility for creating
 *  and starting the tasks that make up the test.  If the time of day
 *  clock is required for the test, it should also be set to a known
 *  value by this function.
 *
 *  Input parameters:
 *    argument - task argument
 *
 *  Output parameters:  NONE
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

#include "system.h"
#undef EXTERN
#define EXTERN
#include "conftbl.h"
#include "gvar.h"

rtems_task Init(
  rtems_task_argument argument
)
{
  rtems_status_code status;

  printf(
    "\n\n*** TEST 13 -- NODE %d ***\n",
    Multiprocessing_configuration.node
  );

  Task_name[ 1 ] = rtems_build_name( '1', '1', '1', ' ' );
  Task_name[ 2 ] = rtems_build_name( '2', '2', '2', ' ' );

  Queue_name[ 1 ] = rtems_build_name( 'M', 'S', 'G', ' ' );

  Semaphore_name[ 1 ] = rtems_build_name( 'S', 'E', 'M', ' ' );

  if ( Multiprocessing_configuration.node == 1 ) {
    puts( "Creating Message Queue (Global)" );
    status = rtems_message_queue_create(
      Queue_name[ 1 ],
      3,
      16,
      RTEMS_GLOBAL,
      &Queue_id[ 1 ]
    );
    directive_failed( status, "rtems_message_queue_create" );

    puts( "Creating Semaphore (Global)" );
    status = rtems_semaphore_create(
      Semaphore_name[ 1 ],
      1,
      RTEMS_GLOBAL | RTEMS_PRIORITY,
      &Semaphore_id[ 1 ]
    );
    directive_failed( status, "rtems_semaphore_create" );

    status = rtems_semaphore_obtain(
      Semaphore_id[ 1 ],
      RTEMS_DEFAULT_OPTIONS,
      RTEMS_NO_TIMEOUT
    );
    directive_failed( status, "rtems_semaphore_obtain" );
  }

  puts( "Creating Test_task 1 (local)" );
  status = rtems_task_create(
    Task_name[ 1 ],
    1,
    1024,
    RTEMS_TIMESLICE,
    RTEMS_DEFAULT_ATTRIBUTES,
    &Task_id[ 1 ]
  );
  directive_failed( status, "rtems_task_create" );

  puts( "Starting Test_task 1 (local)" );
  status = rtems_task_start( Task_id[ 1 ], Test_task1, 0 );
  directive_failed( status, "rtems_task_start" );

  puts( "Creating Test_task 2 (local)" );
  status = rtems_task_create(
    Task_name[ 2 ],
    1,
    1024,
    RTEMS_TIMESLICE,
    RTEMS_DEFAULT_ATTRIBUTES,
    &Task_id[ 2 ]
  );
  directive_failed( status, "rtems_task_create" );

  puts( "Starting Test_task 2 (local)" );
  status = rtems_task_start( Task_id[ 2 ], Test_task2, 0 );
  directive_failed( status, "rtems_task_start" );

  if ( Multiprocessing_configuration.node == 1 ) {
    status = rtems_task_wake_after( 5 * TICKS_PER_SECOND );
    directive_failed( status, "rtems_task_wake_after" );

    puts( "*** END OF TEST 13 ***" );
    exit( 0 );
  }
  puts( "Deleting initialization task" );
  status = rtems_task_delete( RTEMS_SELF );
  directive_failed( status, "rtems_task_delete of RTEMS_SELF" );
}
