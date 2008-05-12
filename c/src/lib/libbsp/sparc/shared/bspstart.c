/*
 *  This set of routines starts the application.  It includes application,
 *  board, and monitor specific initialization and configuration.
 *  The generic CPU dependent initialization has been performed
 *  before any of these are invoked.
 *
 *  COPYRIGHT (c) 1989-2006.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 *  Ported to ERC32 implementation of the SPARC by On-Line Applications
 *  Research Corporation (OAR) under contract to the European Space
 *  Agency (ESA).
 *
 *  ERC32 modifications of respective RTEMS file: COPYRIGHT (c) 1995.
 *  European Space Agency.
 *
 *  $Id$
 */

/* must be identical to STACK_SIZE in start.S */
#define STACK_SIZE 16 * 1024

#include <string.h>

#include <bsp.h>
#include <rtems/libio.h>
#include <rtems/libcsupport.h>
#include <rtems/bspIo.h>

/*
 *  Tells us where to put the workspace in case remote debugger is present.
 */

extern uint32_t rdb_start;

#ifdef LEON2
/*
 * Tells us if data cache snooping is available
 */ 

int CPU_SPARC_HAS_SNOOPING;
#endif

/*
 *  Use the shared implementations of the following routines
 */

void bsp_libc_init( void *, uint32_t, int );
extern void bsp_spurious_initialize();

#ifdef LEON2
/*
 * set_snooping
 * 
 * Read the data cache configuration register to determine if
 * bus snooping is available. This is needed for some drivers so
 * that they can select the most efficient copy routines.  
 *
 */

static inline int set_snooping(void) 
{
        unsigned int tmp = *(unsigned int *)0x80000014; /* Cache control register */
        return ((tmp>>23) & 1); /* Data cache snooping enabled */
}
#endif

/*
 *  bsp_pretasking_hook
 *
 *  BSP pretasking hook.  Called just before drivers are initialized.
 *  Used to setup libc and install any BSP extensions.
 */

void bsp_pretasking_hook(void)
{
  extern int end;
  uint32_t heap_start;
  uint32_t heap_size;

  heap_start = (uint32_t) &end;
  if (heap_start & (CPU_ALIGNMENT-1))
    heap_start = (heap_start + CPU_ALIGNMENT) & ~(CPU_ALIGNMENT-1);

  heap_size = Configuration.work_space_start - (void *)&end - STACK_SIZE;
  heap_size &= 0xfffffff0;  /* keep it as a multiple of 16 bytes */

  bsp_libc_init((void *) heap_start, heap_size, 0);

#ifdef RTEMS_DEBUG
  rtems_debug_enable( RTEMS_DEBUG_ALL_MASK );
#endif

  bsp_spurious_initialize();
}

/*
 *  bsp_start
 *
 *  This routine does the bulk of the system initialization.
 */

void bsp_start( void )
{
  unsigned char *work_space_start;

  work_space_start =
    (unsigned char *)rdb_start - rtems_configuration_get_work_space_size();

  if ( work_space_start <= (unsigned char *)&end ) {
    printk( "bspstart: Not enough RAM!!!\n" );
    BSP_fatal_return();
  }

  Configuration.work_space_start = work_space_start;
  
  #ifdef LEON2
  CPU_SPARC_HAS_SNOOPING = set_snooping();
  #endif
}
