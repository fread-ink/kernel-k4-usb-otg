/*
 *  linux/drivers/video/eink/hal/einkfb_hal_mem.c -- eInk frame buffer device HAL memory
 *
 *      Copyright (c) 2005-2011 Amazon Technologies
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include "einkfb_hal.h"

#if PRAGMAS
    #pragma mark Local Utilities
    #pragma mark -
#endif

static void einkfb_deferred_io(struct fb_info *fb_info, struct list_head *pagelist)
{
    if ( !EINKFB_DISPLAY_PAUSED() )
    {
        unsigned long beg, end;
        int y1, y2, miny, maxy;
        struct page *page;

        miny = INT_MAX;
        maxy = 0;
        list_for_each_entry(page, pagelist, lru)
        {
            beg = page->index << PAGE_SHIFT;
            end = beg + PAGE_SIZE - 1;
            y1 = beg / fb_info->fix.line_length;
            y2 = end / fb_info->fix.line_length;
            if (y2 > fb_info->var.yres)
                y2 = fb_info->var.yres;
            if (miny > y1)
                miny = y1;
            if (maxy < y2)
                maxy = y2;
        }

        // Use a full-display update instead of an area-update of the full
        // display if we're updating the entire display since full-display
        // updates are more efficient than an area-update of the full display.
        //
        if ( fb_info->var.yres == (maxy - miny) )
            EINKFB_IOCTL(FBIO_EINK_UPDATE_DISPLAY, fx_update_partial);
        else
        {
            update_area_t update_area;

            update_area.x1 = 0;
            update_area.y1 = miny;
            update_area.x2 = fb_info->var.xres;
            update_area.y2 = maxy;
            update_area.which_fx = fx_update_partial;
            update_area.buffer = NULL;
            
            EINKFB_IOCTL(FBIO_EINK_UPDATE_DISPLAY_AREA, (unsigned long)&update_area);
        }
    }
}

static struct fb_deferred_io einkfb_defio =
{
    .delay          = HZ/4,
    .deferred_io    = einkfb_deferred_io
};

#if PRAGMAS
    #pragma mark -
    #pragma mark External Interfaces
    #pragma mark -
#endif

int einkfb_mmap(FB_MMAP_PARAMS)
{
    struct einkfb_info hal_info;
    einkfb_get_info(&hal_info);
    
    if ( hal_info.dma )
        dma_mmap_writecombine(hal_info.fbinfo->device, vma, hal_info.start, hal_info.phys->addr,
            hal_info.phys->size);

    return ( EINKFB_SUCCESS );
}

void *einkfb_malloc(size_t size, einkfb_dma_addr_t *phys, bool mmap)
{
    void *result = NULL;
    
    if ( size )
    {
        struct einkfb_info info;
        einkfb_get_info(&info);
        
        if ( phys )
        {
            result = dma_alloc_writecombine(info.fbinfo->device, size, &phys->addr,
                GFP_DMA);
                
            if ( result )
                phys->size = size;
         }
        else
            result = vmalloc(size);
        
        // Use the deferred I/O mechanism to handle our page mapping.
        //
        if ( mmap && result )
        {
            info.fbinfo->fbdefio = &einkfb_defio;
            fb_deferred_io_init(info.fbinfo);
        }
    }
    
    return ( result );
}

void einkfb_free(void *ptr, einkfb_dma_addr_t *phys, bool mmap)
{
    if ( ptr )
    {
        struct einkfb_info info;
        einkfb_get_info(&info);
        
        if ( mmap )
        {
            fb_deferred_io_cleanup(info.fbinfo);
            info.fbinfo->fbdefio = NULL;
        }
            
        if ( phys )
            dma_free_writecombine(info.fbinfo->device, phys->size, ptr, phys->addr);
        else
            vfree(ptr);
    }
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31))
static void einkfb_memcpy_schedule(u8 *dst, const u8 *src, size_t dst_length, size_t src_length)
{
    bool set = 0 == src_length;
    int length = dst_length;
     
    if ( set )
    {
        int set_val = *((int *)src);
        memset(dst, set_val, length);
    }
    else
        memcpy(dst, src, length);
    
}
#else
static void einkfb_memcpy_schedule(u8 *dst, const u8 *src, size_t dst_length, size_t src_length)
{
    int  set_val = 0, i = 0, length = EINKFB_MEMCPY_MIN, num_loops = dst_length/length,
         remainder = dst_length % length;
    bool set = 0 == src_length, done = false;
    
    if ( 0 != num_loops )
        einkfb_debug("num_loops @ %d bytes = %d, remainder = %d\n",
            length, num_loops, remainder);

    if ( set )
        set_val = *((int *)src);
    
    // Set or copy EINKFB_MEMCPY_MIN bytes at a time.  While there are still
    // bytes to set or copy, yield the CPU.
    //
    do
    {
        if ( 0 >= num_loops )
            length = remainder;
        
        if ( set )
            memset(&dst[i], set_val, length);
        else
            memcpy(&dst[i], &src[i], length);
          
        i += length;
        
        if ( i < dst_length )
        {
            EINKFB_SCHEDULE();
            num_loops--;
        }
        else
            done = true;
    }
    while ( !done );
}
#endif

int einkfb_memcpy(bool direction, unsigned long flag, void *destination, const void *source, size_t length)
{
    int result = EINKFB_IOCTL_FAILURE;
    
    if ( destination && source && length )
    {
        result = EINKFB_SUCCESS;
        
        // Do a memcpy() in kernel space; otherwise, copy to/from user-space,
        // as specified.
        //
        if ( EINKFB_IOCTL_KERN == flag )
            einkfb_memcpy_schedule((u8 *)destination, (u8 *)source, length, length);
        else
        {
            if ( EINKFB_IOCTL_FROM_USER == direction )
                result = copy_from_user(destination, source, length);
            else
                result = copy_to_user(destination, source, length);
        }
    }
    
    return ( result );
}

void einkfb_memset(void *destination, int value, size_t length)
{
    if ( destination && length )
        einkfb_memcpy_schedule((u8 *)destination, (u8 *)&value, length, 0);
}

EXPORT_SYMBOL(einkfb_malloc);
EXPORT_SYMBOL(einkfb_free);
EXPORT_SYMBOL(einkfb_memcpy);
EXPORT_SYMBOL(einkfb_memset);
