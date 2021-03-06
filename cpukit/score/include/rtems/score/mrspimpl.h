/*
 * Copyright (c) 2014, 2016 embedded brains GmbH.  All rights reserved.
 *
 *  embedded brains GmbH
 *  Dornierstr. 4
 *  82178 Puchheim
 *  Germany
 *  <rtems@embedded-brains.de>
 *
 * The license and distribution terms for this file may be
 * found in the file LICENSE in this distribution or at
 * http://www.rtems.org/license/LICENSE.
 */

#ifndef _RTEMS_SCORE_MRSPIMPL_H
#define _RTEMS_SCORE_MRSPIMPL_H

#include <rtems/score/mrsp.h>

#if defined(RTEMS_SMP)

#include <rtems/score/assert.h>
#include <rtems/score/chainimpl.h>
#include <rtems/score/resourceimpl.h>
#include <rtems/score/schedulerimpl.h>
#include <rtems/score/status.h>
#include <rtems/score/threadqimpl.h>
#include <rtems/score/watchdogimpl.h>
#include <rtems/score/wkspace.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @addtogroup ScoreMRSP
 *
 * @{
 */

/**
 * @brief Internal state used for MRSP_Rival::status to indicate that this
 * rival waits for resource ownership.
 */
#define MRSP_WAIT_FOR_OWNERSHIP STATUS_MINUS_ONE

/*
 * FIXME: Operations with the resource dependency tree are protected by the
 * global scheduler lock.  Since the scheduler lock should be scheduler
 * instance specific in the future this will only work temporarily.  A more
 * sophisticated locking strategy is necessary.
 */

RTEMS_INLINE_ROUTINE void _MRSP_Giant_acquire( ISR_lock_Context *lock_context )
{
  _ISR_lock_Acquire( &_Scheduler_Lock, lock_context );
}

RTEMS_INLINE_ROUTINE void _MRSP_Giant_release( ISR_lock_Context *lock_context )
{
  _ISR_lock_Release( &_Scheduler_Lock, lock_context );
}

RTEMS_INLINE_ROUTINE void _MRSP_Acquire_critical(
  MRSP_Control         *mrsp,
  Thread_queue_Context *queue_context
)
{
  _Thread_queue_Acquire_critical( &mrsp->Wait_queue, queue_context );
}

RTEMS_INLINE_ROUTINE void _MRSP_Release(
  MRSP_Control         *mrsp,
  Thread_queue_Context *queue_context
)
{
  _Thread_queue_Release( &mrsp->Wait_queue, queue_context );
}

RTEMS_INLINE_ROUTINE Priority_Control _MRSP_Get_priority(
  const MRSP_Control      *mrsp,
  const Scheduler_Control *scheduler
)
{
  uint32_t scheduler_index;

  scheduler_index = _Scheduler_Get_index( scheduler );
  return mrsp->ceiling_priorities[ scheduler_index ];
}

RTEMS_INLINE_ROUTINE void _MRSP_Set_priority(
  MRSP_Control            *mrsp,
  const Scheduler_Control *scheduler,
  Priority_Control         new_priority
)
{
  uint32_t scheduler_index;

  scheduler_index = _Scheduler_Get_index( scheduler );
  mrsp->ceiling_priorities[ scheduler_index ] = new_priority;
}

RTEMS_INLINE_ROUTINE Status_Control _MRSP_Raise_priority(
  MRSP_Control         *mrsp,
  Thread_Control       *thread,
  Priority_Node        *priority_node,
  Thread_queue_Context *queue_context
)
{
  Status_Control           status;
  ISR_lock_Context         lock_context;
  const Scheduler_Control *scheduler;
  Priority_Control         ceiling_priority;
  Scheduler_Node          *own_node;

  _Thread_queue_Context_clear_priority_updates( queue_context );
  _Thread_Wait_acquire_default_critical( thread, &lock_context );

  scheduler = _Scheduler_Get_own( thread );
  own_node = _Thread_Scheduler_get_own_node( thread );
  ceiling_priority = _MRSP_Get_priority( mrsp, scheduler );

  if ( ceiling_priority <= own_node->Wait.Priority.Node.priority ) {
    _Priority_Node_initialize( priority_node, ceiling_priority );
    _Thread_Priority_add( thread, priority_node, queue_context );
    status = STATUS_SUCCESSFUL;
  } else {
    status = STATUS_MUTEX_CEILING_VIOLATED;
  }

  _Thread_Wait_release_default_critical( thread, &lock_context );
  return status;
}

RTEMS_INLINE_ROUTINE void _MRSP_Remove_priority(
  Thread_Control       *thread,
  Priority_Node        *priority_node,
  Thread_queue_Context *queue_context
)
{
  ISR_lock_Context  lock_context;

  _Thread_queue_Context_clear_priority_updates( queue_context );
  _Thread_Wait_acquire_default_critical( thread, &lock_context );
  _Thread_Priority_remove( thread, priority_node, queue_context );
  _Thread_Wait_release_default_critical( thread, &lock_context );
}

RTEMS_INLINE_ROUTINE void _MRSP_Replace_priority(
  MRSP_Control   *mrsp,
  Thread_Control *thread,
  MRSP_Rival     *rival
)
{
  ISR_lock_Context lock_context;

  _Thread_Wait_acquire_default_critical( thread, &lock_context );
  _Thread_Priority_replace( 
    thread,
    &rival->Ceiling_priority,
    &mrsp->Ceiling_priority
  );
  _Thread_Wait_release_default_critical( thread, &lock_context );
}

RTEMS_INLINE_ROUTINE Status_Control _MRSP_Claim_ownership(
  MRSP_Control         *mrsp,
  Thread_Control       *new_owner,
  Thread_queue_Context *queue_context
)
{
  Status_Control   status;
  Per_CPU_Control *cpu_self;

  status = _MRSP_Raise_priority(
    mrsp,
    new_owner,
    &mrsp->Ceiling_priority,
    queue_context
  );

  if ( status != STATUS_SUCCESSFUL ) {
    _MRSP_Release( mrsp, queue_context );
    return status;
  }

  _Resource_Node_add_resource( &new_owner->Resource_node, &mrsp->Resource );
  _Resource_Set_owner( &mrsp->Resource, &new_owner->Resource_node );
  _Scheduler_Thread_change_help_state( new_owner, SCHEDULER_HELP_ACTIVE_OWNER );

  cpu_self = _Thread_Dispatch_disable_critical(
    &queue_context->Lock_context.Lock_context
  );
  _MRSP_Release( mrsp, queue_context );

  _Thread_Priority_update( queue_context );

  _Thread_Dispatch_enable( cpu_self );
  return STATUS_SUCCESSFUL;
}

RTEMS_INLINE_ROUTINE Status_Control _MRSP_Initialize(
  MRSP_Control            *mrsp,
  const Scheduler_Control *scheduler,
  Priority_Control         ceiling_priority,
  Thread_Control          *executing,
  bool                     initially_locked
)
{
  uint32_t scheduler_count = _Scheduler_Count;
  uint32_t i;

  if ( initially_locked ) {
    return STATUS_INVALID_NUMBER;
  }

  mrsp->ceiling_priorities = _Workspace_Allocate(
    sizeof( *mrsp->ceiling_priorities ) * scheduler_count
  );
  if ( mrsp->ceiling_priorities == NULL ) {
    return STATUS_NO_MEMORY;
  }

  for ( i = 0 ; i < scheduler_count ; ++i ) {
    const Scheduler_Control *scheduler_of_cpu;

    scheduler_of_cpu = _Scheduler_Get_by_CPU_index( i );

    if ( scheduler != scheduler_of_cpu ) {
      mrsp->ceiling_priorities[ i ] =
        _Scheduler_Map_priority( scheduler_of_cpu, 0 );
    } else {
      mrsp->ceiling_priorities[ i ] = ceiling_priority;
    }
  }

  _Resource_Initialize( &mrsp->Resource );
  _Chain_Initialize_empty( &mrsp->Rivals );
  _Thread_queue_Initialize( &mrsp->Wait_queue );

  return STATUS_SUCCESSFUL;
}

RTEMS_INLINE_ROUTINE void _MRSP_Timeout( Watchdog_Control *watchdog )
{
  MRSP_Rival           *rival;
  MRSP_Control         *mrsp;
  Thread_Control       *thread;
  Thread_queue_Context  queue_context;

  rival = RTEMS_CONTAINER_OF( watchdog, MRSP_Rival, Watchdog );
  mrsp = rival->resource;
  thread = rival->thread;

  _Thread_queue_Context_initialize( &queue_context );
  _ISR_lock_ISR_disable( &queue_context.Lock_context.Lock_context );
  _MRSP_Acquire_critical( mrsp, &queue_context );

  if ( rival->status == MRSP_WAIT_FOR_OWNERSHIP ) {
    ISR_lock_Context giant_lock_context;

    _MRSP_Remove_priority( thread, &rival->Ceiling_priority, &queue_context );

    _MRSP_Giant_acquire( &giant_lock_context );

    _Chain_Extract_unprotected( &rival->Node );
    _Resource_Node_extract( &thread->Resource_node );
    _Resource_Node_set_dependency( &thread->Resource_node, NULL );
    _Scheduler_Thread_change_help_state( thread, rival->initial_help_state );
    _Scheduler_Thread_change_resource_root( thread, thread );

    _MRSP_Giant_release( &giant_lock_context );

    rival->status = STATUS_TIMEOUT;

    _MRSP_Release( mrsp, &queue_context );

    _Thread_Priority_update( &queue_context );
  } else {
    _MRSP_Release( mrsp, &queue_context );
  }
}

RTEMS_INLINE_ROUTINE Status_Control _MRSP_Wait_for_ownership(
  MRSP_Control         *mrsp,
  Resource_Node        *owner,
  Thread_Control       *executing,
  Thread_queue_Context *queue_context
)
{
  Status_Control     status;
  MRSP_Rival         rival;
  Thread_Life_state  life_state;
  Per_CPU_Control   *cpu_self;
  ISR_lock_Context   giant_lock_context;
  ISR_Level          level;
  Watchdog_Interval  timeout;

  _Assert( queue_context->timeout_discipline == WATCHDOG_RELATIVE );

  status = _MRSP_Raise_priority(
    mrsp,
    executing,
    &rival.Ceiling_priority,
    queue_context
  );

  if ( status != STATUS_SUCCESSFUL ) {
    _MRSP_Release( mrsp, queue_context );
    return status;
  }

  rival.thread = executing;
  rival.resource = mrsp;
  _Chain_Initialize_node( &rival.Node );

  _MRSP_Giant_acquire( &giant_lock_context );

  rival.initial_help_state =
    _Scheduler_Thread_change_help_state( executing, SCHEDULER_HELP_ACTIVE_RIVAL );
  rival.status = MRSP_WAIT_FOR_OWNERSHIP;

  _Chain_Initialize_node( &rival.Node );
  _Chain_Append_unprotected( &mrsp->Rivals, &rival.Node );
  _Resource_Add_rival( &mrsp->Resource, &executing->Resource_node );
  _Resource_Node_set_dependency( &executing->Resource_node, &mrsp->Resource );
  _Scheduler_Thread_change_resource_root(
    executing,
    THREAD_RESOURCE_NODE_TO_THREAD( _Resource_Node_get_root( owner ) )
  );

  _MRSP_Giant_release( &giant_lock_context );

  cpu_self = _Thread_Dispatch_disable_critical(
    &queue_context->Lock_context.Lock_context
  );
  _MRSP_Release( mrsp, queue_context );

  _Thread_Priority_update( queue_context );

  timeout = (Watchdog_Interval) queue_context->timeout;

  if ( timeout > 0 ) {
    _Watchdog_Preinitialize( &rival.Watchdog, cpu_self );
    _Watchdog_Initialize( &rival.Watchdog, _MRSP_Timeout );
    _ISR_Local_disable( level );
    _Watchdog_Per_CPU_insert_relative( &rival.Watchdog, cpu_self, timeout );
    _ISR_Local_enable( level );
  }

  life_state = _Thread_Set_life_protection( THREAD_LIFE_PROTECTED );
  _Thread_Dispatch_enable( cpu_self );

  _Assert( _Debug_Is_thread_dispatching_allowed() );

  /* Wait for state change */
  do {
    status = rival.status;
  } while ( status == MRSP_WAIT_FOR_OWNERSHIP );

  _Thread_Set_life_protection( life_state );

  if ( timeout > 0 ) {
    _ISR_Local_disable( level );
    _Watchdog_Per_CPU_remove(
      &rival.Watchdog,
      cpu_self,
      &cpu_self->Watchdog.Header[ PER_CPU_WATCHDOG_RELATIVE ]
    );
    _ISR_Local_enable( level );
  }

  return status;
}

RTEMS_INLINE_ROUTINE Status_Control _MRSP_Seize(
  MRSP_Control         *mrsp,
  Thread_Control       *executing,
  bool                  wait,
  Thread_queue_Context *queue_context
)
{
  Status_Control  status;
  Resource_Node  *owner;

  _MRSP_Acquire_critical( mrsp, queue_context );

  owner = _Resource_Get_owner( &mrsp->Resource );

  if ( owner == NULL ) {
    status = _MRSP_Claim_ownership( mrsp, executing, queue_context );
  } else if (
    wait
      && _Resource_Node_get_root( owner ) != &executing->Resource_node
  ) {
    status = _MRSP_Wait_for_ownership( mrsp, owner, executing, queue_context );
  } else {
    _MRSP_Release( mrsp, queue_context );
    /* Not available, nested access or deadlock */
    status = STATUS_UNAVAILABLE;
  }

  return status;
}

RTEMS_INLINE_ROUTINE Status_Control _MRSP_Surrender(
  MRSP_Control         *mrsp,
  Thread_Control       *executing,
  Thread_queue_Context *queue_context
)
{
  ISR_lock_Context  giant_lock_context;
  Per_CPU_Control  *cpu_self;

  if ( _Resource_Get_owner( &mrsp->Resource ) != &executing->Resource_node ) {
    _ISR_lock_ISR_enable( &queue_context->Lock_context.Lock_context );
    return STATUS_NOT_OWNER;
  }

  if (
    !_Resource_Is_most_recently_obtained(
      &mrsp->Resource,
      &executing->Resource_node
    )
  ) {
    _ISR_lock_ISR_enable( &queue_context->Lock_context.Lock_context );
    return STATUS_RELEASE_ORDER_VIOLATION;
  }

  _MRSP_Acquire_critical( mrsp, queue_context );
  _MRSP_Remove_priority( executing, &mrsp->Ceiling_priority, queue_context );
  _MRSP_Giant_acquire( &giant_lock_context );

  _Resource_Extract( &mrsp->Resource );

  if ( _Chain_Is_empty( &mrsp->Rivals ) ) {
    _Resource_Set_owner( &mrsp->Resource, NULL );
  } else {
    MRSP_Rival     *rival;
    Thread_Control *new_owner;

    rival = (MRSP_Rival *) _Chain_Get_first_unprotected( &mrsp->Rivals );

    /*
     * This must be inside the critical section since the status prevents a
     * potential double extraction in _MRSP_Timeout().
     */
    rival->status = STATUS_SUCCESSFUL;

    new_owner = rival->thread;

    _MRSP_Replace_priority( mrsp, new_owner, rival );

    _Resource_Node_extract( &new_owner->Resource_node );
    _Resource_Node_set_dependency( &new_owner->Resource_node, NULL );
    _Resource_Node_add_resource( &new_owner->Resource_node, &mrsp->Resource );
    _Resource_Set_owner( &mrsp->Resource, &new_owner->Resource_node );
    _Scheduler_Thread_change_help_state( new_owner, SCHEDULER_HELP_ACTIVE_OWNER );
    _Scheduler_Thread_change_resource_root( new_owner, new_owner );
  }

  if ( !_Resource_Node_owns_resources( &executing->Resource_node ) ) {
    _Scheduler_Thread_change_help_state( executing, SCHEDULER_HELP_YOURSELF );
  }

  _MRSP_Giant_release( &giant_lock_context );

  cpu_self = _Thread_Dispatch_disable_critical(
    &queue_context->Lock_context.Lock_context
  );
  _MRSP_Release( mrsp, queue_context );

  _Thread_Priority_update( queue_context );

  _Thread_Dispatch_enable( cpu_self );

  return STATUS_SUCCESSFUL;
}

RTEMS_INLINE_ROUTINE Status_Control _MRSP_Can_destroy( MRSP_Control *mrsp )
{
  if ( _Resource_Get_owner( &mrsp->Resource ) != NULL ) {
    return STATUS_RESOURCE_IN_USE;
  }

  return STATUS_SUCCESSFUL;
}

RTEMS_INLINE_ROUTINE void _MRSP_Destroy(
  MRSP_Control         *mrsp,
  Thread_queue_Context *queue_context
)
{
  _MRSP_Release( mrsp, queue_context );
  _Thread_queue_Destroy( &mrsp->Wait_queue );
  _Workspace_Free( mrsp->ceiling_priorities );
}

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* RTEMS_SMP */

#endif /* _RTEMS_SCORE_MRSPIMPL_H */
