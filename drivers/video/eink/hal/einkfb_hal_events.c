/*
 *  linux/drivers/video/eink/hal/einkfb_hal_events.c --
 *  eInk frame buffer device HAL events
 *
 *      Copyright (C) 2008-2011 Amazon Technologies, Inc.
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include "einkfb_hal.h"

#if PRAGMAS
    #pragma mark Definitions/Globals
#endif

#define EINKFB_NUM_EVENTS               101     // n+1 -> n events + 1 for empty
#define EINKFB_EVENT_TIMER_DELAY        (HZ/4)  // minimum DU-style update time
#define EINKFB_EVENT_THREAD_NAME        EINKFB_NAME"_et"

static einkfb_event_t einkfb_event_queue[EINKFB_NUM_EVENTS];
static einkfb_event_t einkfb_event;

static int einkfb_event_queue_entry     = 0;
static int einkfb_event_queue_exit      = 0;
static int einkfb_events_access_count   = 0;

static DECLARE_WAIT_QUEUE_HEAD(einkfb_events_read_wait);

static bool einkfb_event_timer_active   = false;
static bool einkfb_event_timer_primed   = false;
static struct timer_list einkfb_event_timer;
static int einkfb_event_thread_exit = 0;
static THREAD_ID(einkfb_event_thread_id);
static DECLARE_COMPLETION(einkfb_event_thread_exited);
static DECLARE_COMPLETION(einkfb_event_thread_complete);

#if PRAGMAS
    #pragma mark -
    #pragma mark Local Utilities
    #pragma mark -
#endif

static bool einkfb_valid_event(einkfb_event_t *event)
{
    bool result = false;
    
    if ( event )
        result = einkfb_event_null != event->event;
    
    return ( result );
}

static bool einkfb_event_queue_empty(void)
{
    return ( einkfb_event_queue_entry == einkfb_event_queue_exit );
}

static void einkfb_enqueue_event(einkfb_event_t *event)
{
    if ( einkfb_valid_event(event) )
    {
        // Enqueue the passed-in event.
        //
        EINKFB_MEMCPYK(&einkfb_event_queue[einkfb_event_queue_entry++], event, sizeof(einkfb_event_t));

        // Wrap back to the start if we've reached the end.
        //
        if ( EINKFB_NUM_EVENTS == einkfb_event_queue_entry )
            einkfb_event_queue_entry = 0;

        // Ensure that we don't accidently go empty.
        //
        if ( einkfb_event_queue_empty() )
        {
            einkfb_event_queue_exit++;
        
            if ( EINKFB_NUM_EVENTS == einkfb_event_queue_exit )
                einkfb_event_queue_exit = 0;
        }
    }
}

static einkfb_event_t *einkfb_dequeue_event(void)
{
    einkfb_event_t *result = NULL;
    
    if ( !einkfb_event_queue_empty() )
    {
        // Dequeue the event.
        //
        EINKFB_MEMCPYK(&einkfb_event, &einkfb_event_queue[einkfb_event_queue_exit++], sizeof(einkfb_event_t));
        result = &einkfb_event;
        
        // Wrap back to the start if we've reached the end.
        //
        if ( EINKFB_NUM_EVENTS == einkfb_event_queue_exit )
            einkfb_event_queue_exit = 0;
    }
    
    return ( result );
}

static void exit_einkfb_event_thread(void)
{
    einkfb_event_thread_exit = 1;
    complete(&einkfb_event_thread_complete);
}

static void einkfb_event_thread_body(void)
{
    // Post that the prior update-display event(s) have completed once they have.
    //
    if ( einkfb_event_timer_active && einkfb_event_timer_primed )
    {
        einkfb_event_t event;
        
        einkfb_event_timer_primed = false;
        
        einkfb_init_event(&event);
        event.event = einkfb_event_update_display_complete;
        
        EINKFB_FSYNC();
        einkfb_post_event(&event);
    }
    
    wait_for_completion_interruptible(&einkfb_event_thread_complete);
}

static int einkfb_event_thread(void *data)
{
    int  *exit_thread = (int *)data;
    bool thread_active = true;
    
    THREAD_HEAD(EINKFB_EVENT_THREAD_NAME);

    while ( thread_active )
    {
        TRY_TO_FREEZE();

        if ( !THREAD_SHOULD_STOP() )
        {
            THREAD_BODY(*exit_thread, einkfb_event_thread_body);
        }
        else
        {
            thread_active = false;
        }
    }

    THREAD_TAIL(einkfb_event_thread_exited);
}

static void einkfb_start_event_thread(void)
{
    einkfb_event_thread_exit = 0;
    
    THREAD_START(einkfb_event_thread_id, &einkfb_event_thread_exit, einkfb_event_thread,
        EINKFB_EVENT_THREAD_NAME);
}

static void einkfb_stop_event_thread(void)
{
    THREAD_STOP(einkfb_event_thread_id, exit_einkfb_event_thread, einkfb_event_thread_exited);
}

static void einkfb_event_timer_function(unsigned long unused)
{
    complete(&einkfb_event_thread_complete);
}

#if PRAGMAS
    #pragma mark -
    #pragma mark External Interfaces
    #pragma mark -
#endif

void einkfb_init_event(einkfb_event_t *event)
{
    if ( event )
        einkfb_memset(event, einkfb_event_null, SIZEOF_EINK_EVENT);
}

void einkfb_post_event(einkfb_event_t *event)
{
    if ( einkfb_events_access_count )
    {
        einkfb_enqueue_event(event);
    
        if ( !einkfb_event_queue_empty() )
            wake_up_interruptible(&einkfb_events_read_wait);
    }
}

int einkfb_events_open(struct inode *inode, struct file *file)
{
    if ( 0 == einkfb_events_access_count++ )
    {
        int i;
        
        einkfb_event_queue_entry = einkfb_event_queue_exit = 0;
        
        for ( i = 0; i < EINKFB_NUM_EVENTS; i++ )
            einkfb_init_event(&einkfb_event_queue[i]);
    }
 
    return ( EINKFB_SUCCESS );
}

int einkfb_events_release(struct inode *inode, struct file *file)
{
    int result = EINKFB_SUCCESS;
    
    if ( 0 >= einkfb_events_access_count )
        result = EINKFB_EVENT_FAILURE;
    else
        einkfb_events_access_count--;

    return ( result );
}

ssize_t einkfb_events_read(struct file *file, char *buf, size_t count, loff_t *ofs)
{
    ssize_t result = 0;
    
    // Only deal with whole events once someone is looking for them.
    //
    if ( einkfb_events_access_count && (SIZEOF_EINK_EVENT <= count) )
    {
        einkfb_event_t *event = NULL;
        
        // Block until an event occurs.
        //
        wait_event_interruptible(einkfb_events_read_wait, !einkfb_event_queue_empty());
        event = einkfb_dequeue_event();

        if ( event )
        {
            if ( EINKFB_SUCCESS == EINKFB_MEMCPYUT(buf, event, SIZEOF_EINK_EVENT) )
                result = SIZEOF_EINK_EVENT;
        }
    }

    return ( result );
}

unsigned int einkfb_events_poll(struct file *file, poll_table *wait)
{
    unsigned int mask = 0;

    poll_wait(file, &einkfb_events_read_wait, wait);

    if ( !einkfb_event_queue_empty() )
        mask |= POLLIN | POLLRDNORM;
    
    return ( mask );
}

void einkfb_start_event_timer(void)
{
    if ( !einkfb_event_timer_active )
    {
        init_timer(&einkfb_event_timer);
        
        einkfb_event_timer.function = einkfb_event_timer_function;
        einkfb_event_timer.data = 0;
    
        einkfb_event_timer_active  = true;
        einkfb_event_timer_primed  = false;
        
        einkfb_start_event_thread();
    }
}

void einkfb_stop_event_timer(void)
{
    if ( einkfb_event_timer_active )
    {
        einkfb_event_timer_active  = false;
        einkfb_event_timer_primed  = false;

        einkfb_stop_event_thread();
        del_timer_sync(&einkfb_event_timer);
    }
}

void einkfb_prime_event_timer(bool delay_timer)
{
    if ( einkfb_event_timer_active && (einkfb_events_access_count > 0) )
    {
        einkfb_event_timer_primed = delay_timer;
        mod_timer(&einkfb_event_timer, TIMER_EXPIRES_JIFS(delay_timer ? EINKFB_EVENT_TIMER_DELAY
                                                                      : MIN_SCHEDULE_TIMEOUT));
    }
}

