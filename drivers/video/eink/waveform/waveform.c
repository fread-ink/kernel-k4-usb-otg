/*
 * waveform.c
 *
 * Copyright (C) 2010 Amazon Technologies
 *
 */

#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include "eink_waveform.h"
#include "eink_commands.h"

#define WF_TYPE_WAVEFORM    "waveform"
#define WF_TYPE_COMMANDS    "commands"

#define WF_UNKNOWN_VERSION  "???"

#define WF_BUFF_SIZE        EINK_WAVEFORM_FILESIZE
#define WF_PATH_SIZE        256
#define WF_CKSM_SIZE        16

#define WF_PATH()           (0 != wf_path[0])
#define GET_WF_PATH(f)      (WF_PATH() ? (f) : EINK_WF_UNKNOWN_PATH)

enum wf_type_t
{
    wf_type_waveform,
    wf_type_commands
};
typedef enum wf_type_t wf_type_t;

static wf_type_t wf_type = wf_type_waveform;

static char wf_buff[WF_BUFF_SIZE] = { 0 };
static char wf_path[WF_PATH_SIZE] = { 0 };
static char wf_cksm[WF_CKSM_SIZE] = { 0 };

static char wf_extract_path[WF_PATH_SIZE] = { 0 };

static char *wf_buffer      = wf_buff;
static int   wf_buffer_size = WF_BUFF_SIZE;

static unsigned char  *buffer_byte  = NULL;
static unsigned short *buffer_short = NULL;
static unsigned long  *buffer_long  = NULL;

// ----------------------------------------------- //

// CRC-32 algorithm from:
//  <http://glacier.lbl.gov/cgi-bin/viewcvs.cgi/dor-test/crc32.c?rev=HEAD>

/* Table of CRCs of all 8-bit messages. */
static unsigned crc_table[256];

/* Flag: has the table been computed? Initially false. */
static int crc_table_computed = 0;

/* Make the table for a fast CRC. */
static void make_crc_table(void) {
   unsigned c;
   int n, k;
   for (n = 0; n < 256; n++) {
      c = (unsigned) n;
      for (k = 0; k < 8; k++) {
         if (c & 1) {
            c = 0xedb88320L ^ (c >> 1);
         }
         else {
            c = c >> 1;
         }
      }
      crc_table[n] = c;
   }
   crc_table_computed = 1;
}

/*
  Update a running crc with the bytes buf[0..len-1] and
  the updated crc. The crc should be initialized to zero. Pre-
  post-conditioning (one's complement) is performed within
  function so it shouldn't be done by the caller. Usage example

  unsigned crc = 0L

  while (read_buffer(buffer, length) != EOF) {
     crc = update_crc(crc, buffer, length);
  }
  if (crc != original_crc) error();
*/
static unsigned update_crc(unsigned crc,
                           unsigned char *buf, int len) {
   unsigned c = crc ^ 0xffffffffL;
   int n;

   if (!crc_table_computed) make_crc_table();

   for (n = 0; n < len; n++) {
      c = crc_table[(c ^ buf[n]) & 0xff] ^ (c >> 8);
   }

   return c ^ 0xffffffffL;
}

/* Return the CRC of the bytes buf[0..len-1]. */
unsigned crc32(unsigned char *buf, int len) {
   return update_crc(0L, buf, len);
}

// ----------------------------------------------- //

/* Return the sum of the bytes buf[0..len-1]. */
unsigned sum32(unsigned char *buf, int len) {
    unsigned c = 0;
    int n;

    for (n = 0; n < len; n++)
        c += buf[n];

    return c;
}

// ----------------------------------------------- //

/* Return the sum of the bytes buf[0..len-1]. */
unsigned char sum8(unsigned char *buf, int len) {
    unsigned char c = 0;
    int n;

    for (n = 0; n < len; n++)
        c += buf[n];

    return c;
}

int eink_read_byte(unsigned long addr, unsigned char *data)
{
    if ( data && buffer_byte )
        *data = buffer_byte[addr];

    return ( NULL != buffer_byte );
}

int eink_read_short(unsigned long addr, unsigned short *data)
{
    if ( data && buffer_short && buffer_byte )
    {
        // Read 16 bits if we're 16-bit aligned.
        //
        if ( addr == ((addr >> 1) << 1) )
            *data = buffer_short[addr >> 1];
        else
            *data = (buffer_byte[addr + 0] << 0) |
                    (buffer_byte[addr + 1] << 8);
    }

    return ( NULL != buffer_short );
}

int eink_read_long(unsigned long addr, unsigned long *data)
{
    if ( data && buffer_long && buffer_byte )
    {
        // Read 32 bits if we're 32-bit aligned.
        //
        if ( addr == ((addr >> 2) << 2) )
            *data = buffer_long[addr >> 2];
        else
            *data = (buffer_byte[addr + 0] <<  0) |
                    (buffer_byte[addr + 1] <<  8) |
                    (buffer_byte[addr + 2] << 16) |
                    (buffer_byte[addr + 3] << 24);
    }

    return ( NULL != buffer_long );
}

#include "eink_waveform.c"
#include "eink_commands.c"
#include "eink_panel.c"

 void einkwf_set_buffer_size(int size)
 {
    // Clip to WF_BUFF_SIZE if we're using wf_buff.
    //
    if ( wf_buffer == wf_buff )
        wf_buffer_size = (WF_BUFF_SIZE >= size) ? size : WF_BUFF_SIZE;
    else
        wf_buffer_size = size;
 }
 
 int einkwf_get_buffer_size(void)
 {
    return ( wf_buffer_size );
 }
 
void einkwf_set_buffer(char *buffer)
{
    // Use the internal buffer by default.
    //
    wf_buffer = buffer ? buffer : wf_buff;   
}

char *einkwf_get_buffer(void)
{
    return ( wf_buffer );
}

static void einkwf_buffers_set(void)
{
    buffer_byte  = einkwf_get_buffer();
    buffer_short = (unsigned short *)buffer_byte;
    buffer_long  = (unsigned long *)buffer_byte;
}

static void einkwf_buffers_clr(void)
{
    buffer_byte  = NULL;
    buffer_short = NULL;
    buffer_long  = NULL;
}

void einkwf_get_waveform_info(eink_waveform_info_t *info)
{
    einkwf_buffers_set();
    
    eink_get_waveform_info(info);
    
    einkwf_buffers_clr();
}

char *einkwf_get_waveform_version_string(eink_waveform_version_string_t which_string)
{
    char *version_string = NULL;

    einkwf_buffers_set();

    version_string = eink_get_waveform_version_string(which_string);

    einkwf_buffers_clr();

    return ( version_string );
}

bool einkwf_waveform_valid(void)
{
    bool result = false;
    
    einkwf_buffers_set();
    
    result = eink_waveform_valid();
    
    einkwf_buffers_clr();
    
    return ( result );    
}

int einkwf_write_waveform_to_file(char *waveform_file_path)
{
    return ( eink_write_waveform_to_file(waveform_file_path) );
}

int einkwf_read_waveform_from_file(char *waveform_file_path)
{
    return ( eink_read_waveform_from_file(waveform_file_path) );
}

bool einkwf_get_wavform_proxy_from_waveform(char *waveform_file_path, char *waveform_proxy_path)
{
    bool result = false;
    
    // By convention, we assume that, if we're passed a non-null path string, the
    // length of the proxy path string will be identical.  Additionally, we assume
    // that the proxy string is filled will nulls so that, when we overlay
    // the original path onto it, we don't have to add a terminating null.
    //
    if ( (waveform_file_path && (0 != waveform_file_path[0])) && waveform_proxy_path )
    {
        // Check the waveform file path for the WBF extension.
        //
        char *extension = strstr(waveform_file_path, EINK_WF_WBF_EXTENSION);
        
        // If we found the WBF extension...
        //
        if ( extension )
        {
            // ...determine its position in the path name (it should be the last 4 characters).
            //
            size_t length_to_copy = (int)(extension - waveform_file_path);
            
            // Now, copy the path of the WBF up until the extension into the proxy,
            // and then put the WRF extension at the end of the proxy.
            //
            strncpy(waveform_proxy_path, waveform_file_path, length_to_copy);
            strcat(waveform_proxy_path, EINK_WF_RAW_EXTENSION);
            
            // Say that we were able to return the proxy path.
            //
            result = true;
        }
    }
    
    return ( result );
}

void einkwf_get_commands_info(eink_commands_info_t *info)
{
    einkwf_buffers_set();

    eink_get_commands_info(info);

    einkwf_buffers_clr();
}

char *einkwf_get_commands_version_string(void)
{
    char *version_string = NULL;

    einkwf_buffers_set();

    version_string = eink_get_commands_version_string();

    einkwf_buffers_clr();

    return ( version_string );
}

bool einkwf_commands_valid(void)
{
    bool result = false;
    
    einkwf_buffers_set();
    
    result = eink_commands_valid();
    
    einkwf_buffers_clr();
    
    return ( result );
}


bool einkwf_panel_flash_present(void)
{
    return panel_flash_present();
}

u8 *einkwf_panel_get_waveform(char *waveform_file_path, int *waveform_proxy_size)
{
    return panel_read_waveform_from_file(waveform_file_path, waveform_proxy_size);
}

void einkwf_panel_waveform_free(u8 *waveform_proxy)
{
    panel_waveform_free(waveform_proxy);
}

int einkwf_panel_get_waveform_mode(int upd_mode)
{
    return panel_get_waveform_mode(upd_mode);
}

bool einkwf_panel_supports_vcom(void)
{
    return panel_flash_present();
}

int einkwf_panel_get_vcom(void)
{
    return panel_get_vcom();
}

char *einkwf_panel_get_vcom_str(void)
{
    return panel_get_vcom_str();
}

void einkwf_panel_get_waveform_info(eink_waveform_info_t *info)
{
    panel_get_waveform_info(info);
}

char *einkwf_panel_get_waveform_version_string(eink_waveform_version_string_t which_string)
{
    return panel_get_waveform_version_string(which_string);
}

void einkwf_panel_set_update_modes(void)
{
    panel_set_upd_modes();
}

static int read_wf_file(void)
{
    int result = 0;
    
    if ( WF_PATH() )
    {
        einkwf_set_buffer_size(WF_BUFF_SIZE);
        einkwf_set_buffer(wf_buff);
        
        if ( 0 == einkwf_read_waveform_from_file(wf_path) )
            result = 1;
    }
    
    return ( result );
}

static char *get_wf_version_string(void)
{
    char *result = NULL;

    if ( read_wf_file() )
    {
        switch ( wf_type )
        {
            case wf_type_waveform:
            default:
	      einkwf_set_buffer(panel_get_proxy_buffer());
	      result = einkwf_get_waveform_version_string(eink_waveform_version_string);
            break;
            
            case wf_type_commands:
                result = einkwf_get_commands_version_string();
            break;
        }
    }
    
    if ( !result )
        result = WF_UNKNOWN_VERSION;
    
    return ( result );
}

static char *get_wf_embedded_checksum(void)
{
    unsigned long embedded_checksum = 0;

    if ( read_wf_file() )
    {
        switch ( wf_type )
        {
            case wf_type_waveform:
            default:
                embedded_checksum = eink_get_embedded_waveform_checksum(einkwf_get_buffer());
            break;
            
            case wf_type_commands:
                embedded_checksum = eink_get_embedded_commands_checksum(einkwf_get_buffer());
            break;
        }
    }
    
    sprintf(wf_cksm, "0x%08lX", embedded_checksum);

    return ( wf_cksm );
}

static char *get_wf_computed_checksum(void)
{
    unsigned long computed_checksum = 0;

    if ( read_wf_file() )
    {
        switch ( wf_type )
        {
            case wf_type_waveform:
            default:
                computed_checksum = eink_get_computed_waveform_checksum(einkwf_get_buffer());
            break;
            
            case wf_type_commands:
                computed_checksum = eink_get_computed_commands_checksum(einkwf_get_buffer());
            break;
        }
    }
    
    sprintf(wf_cksm, "0x%08lX", computed_checksum);

    return ( wf_cksm );
}

// read/write path
//
static int proc_path_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
     int result = 0;
    
    // We're done after one shot.
    //
    if ( 0 == off )
    {
        result = sprintf(page, "%s\n", GET_WF_PATH(wf_path));
        *eof = 1;
    }

    return ( result );
}

static int proc_path_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
    int result = count;
    
    // The passed in path name must not be longer than WF_PATH_SIZE.
    //
    if ( WF_PATH_SIZE > count )
    {
        memset(wf_path, 0, WF_PATH_SIZE);
        
        // We must be able to copy the passed-in path name.
        //
        if ( copy_from_user(wf_path, buf, count) )
            result = -EFAULT;
    }
    else
        result = -EINVAL;

    return ( result );
}

// read/write type
//
static char *get_wf_type(void)
{
    char *result = NULL;
    
    switch ( wf_type )
    {
        case wf_type_waveform:
        default:
            result = WF_TYPE_WAVEFORM;
        break;
        
        case wf_type_commands:
            result = WF_TYPE_COMMANDS;
        break;
    }
    
    return ( result );
}

static int proc_type_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int result = 0;
    
    // We're done after one shot.
    //
    if ( 0 == off )
    {
        result = sprintf(page, "%d = %s\n", wf_type, get_wf_type());
        *eof = 1;
    }

    return ( result );
}

static int proc_type_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
    // We'll be parsing a number here.
    //
    char lbuf[16] = { 0 };
    int result = count;
   
    // Read in a number-sized chunk.
    //
    if ( !copy_from_user(lbuf, buf, 8) )
    {
        wf_type_t type = (wf_type_t)simple_strtoul(lbuf, NULL, 0);
        
        // If something bogus is passed in, fix it.
        //
        switch ( type )
        {
            case wf_type_waveform:
            case wf_type_commands:
                wf_type = type;
            break;
            
            default:
                wf_type = wf_type_waveform;
            break;
        }
    }
    else
        result = -EFAULT;
        
    return ( result );
}

// read-only version
//
#define GET_WF_VERSION() GET_WF_PATH(get_wf_version_string())

static int proc_version_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int result = 0;
    
    // We're done after one shot.
    //
    if ( 0 == off )
    {
        result = sprintf(page, "%s\n", GET_WF_VERSION());
        *eof = 1;
    }
    
    return ( result );
}

static int proc_human_version_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int result = 0;
    
    // We're done after one shot.
    //
    if ( 0 == off )
    {
	char *human_str = NULL;

	if ( read_wf_file() )
	{
	    einkwf_set_buffer(panel_get_proxy_buffer());
	    einkwf_buffers_set();
	    human_str = eink_get_waveform_human_version_string(eink_waveform_version_string);
	    einkwf_buffers_clr();
	}
    
	if ( !human_str )
	    human_str = WF_UNKNOWN_VERSION;

        result = sprintf(page, "%s\n", human_str);
        *eof = 1;
    }
    
    return ( result );
}


// read-only embedded_checksum
//
#define GET_WF_EMBEDDED_CKSUM() GET_WF_PATH(get_wf_embedded_checksum())

static int proc_embedded_checksum_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int result = 0;
    
    // We're done after one shot.
    //
    if ( 0 == off )
    {
        result = sprintf(page, "%s\n", GET_WF_EMBEDDED_CKSUM());
        *eof = 1;
    }
    
    return ( result );
}

// read-only computed_checksum
//
#define GET_WF_COMPUTED_CKSUM() GET_WF_PATH(get_wf_computed_checksum())

static int proc_computed_checksum_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int result = 0;
    
    // We're done after one shot.
    //
    if ( 0 == off )
    {
        result = sprintf(page, "%s\n", GET_WF_COMPUTED_CKSUM());
        *eof = 1;
    }
    
    return ( result );
}

static int proc_panel_flash_readonly_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int result = 0;
    
    // We're done after one shot.
    //
    if ( 0 == off )
    {
        result = sprintf(page, "%d\n", (int) panel_flash_is_readonly());
        *eof = 1;
    }

    return ( result );
}

static int proc_panel_flash_select_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int result = 0;
    
    // We're done after one shot.
    //
    if ( 0 == off )
    {
        result = sprintf(page, "%d\n", get_flash_select());
        *eof = 1;
    }

    return ( result );
}

static int proc_panel_flash_select_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
    // We'll be parsing a number here.
    //
    char lbuf[16] = { 0 };
    int result = count;
   
    // Read in a number-sized chunk.
    //
    if ( !copy_from_user(lbuf, buf, 8) )
    {
        panel_flash_select select = (panel_flash_select) simple_strtoul(lbuf, NULL, 0);
        set_flash_select(select);
    }
    else
        result = -EFAULT;
        
    return ( result );
}

#define READ_SIZE   (1 << PAGE_SHIFT)
#define WRITE_SIZE  ((unsigned long)0x20000) 

static int proc_panel_flash_data_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int j = 0;

    if ( (off < panel_flash_size) && count )
    {
        int  i = off, total = min((unsigned long)(panel_flash_size - off), (unsigned long)count), length = READ_SIZE,
             num_loops = total/length, remainder = total % length;
        bool done = false;
            
        do
        {
            if ( 0 >= num_loops )
                length = remainder;
                
            panel_read_from_flash(i, (page + j), length);
            
            i += length; j += length;
            
            if ( i < total )
                num_loops--;
            else
                done = true;
        }
        while ( !done );
    }

    return ( j );
}

static int proc_panel_flash_data_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
    /* BEN - not implemented yet */
    return ( -EFAULT );
}

static int proc_panel_bcd_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int result = 0;
    
    // We're done after one shot.
    //
    if ( 0 == off )
        result = sprintf(page, "%s\n", panel_get_bcd());

    return ( result );
}

static int proc_panel_id_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int result = 0;
    
    // We're done after one shot.
    //
    if ( 0 == off )
        result = sprintf(page, "%s\n", panel_get_id());

    return ( result );
}

static int proc_panel_extract_path_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int result = 0;
    
    // We're done after one shot.
    //
    if ( 0 == off )
        result = sprintf(page, "%s\n", wf_extract_path);

    
    return ( result );
}

static int proc_panel_extract_path_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
    int result = count;
    char path[WF_PATH_SIZE+1];
    char *waveform_filename;
    int  str_len;

    if ( WF_PATH_SIZE < count ) {
	printk(KERN_ERR "waveform file path too long (%ld chars)\n", count);
	return -ENOMEM;
    }

    if ( sscanf(buf, "%s", path) <= 0 )
    {
	printk(KERN_ERR "Couldn't read waveform file path\n");
	return -EFAULT;
    }

    waveform_filename = einkwf_panel_get_waveform_version_string(eink_waveform_filename);
    str_len = strlen(path) + strlen(waveform_filename);
        
    pr_debug("read path %s, len %d wfm %s\n", path, strlen(path), waveform_filename);

    // Write out the waveform file to the passed-in path.
    //
    if ( WF_PATH_SIZE < str_len ) {
	printk(KERN_ERR "waveform file path too long (%d chars)\n", str_len);
	return -EFAULT;
    }

    // First, construct the full path name to the waveform file we need
    // for writing it out.
    //
    strcpy(wf_extract_path, path);
    strcat(wf_extract_path, waveform_filename);
            
    // Now, attempt to write it out.
    //
    if ( einkwf_write_waveform_to_file(wf_extract_path) ) {
	printk(KERN_ERR "failed to write out waveform file to %s\n", wf_extract_path);
	return -EFAULT;
    }
   
    return ( result );
}

static int panel_data_address = PNL_BASE;

static int proc_panel_data_addr_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int result = 0;
    
    // We're done after one shot.
    //
    if ( 0 == off )
        result = sprintf(page, "panel data addr to read from: 0x%02X\n\n", panel_data_address);

    
    return ( result );
}

static int proc_panel_data_addr_write(struct file *file, const char __user *buf, unsigned long count, void *data)
{
    int value = 0;

    if (sscanf(buf, "%x", &value) <= 0) {
        pr_debug("Could not store the panel data address\n");
        return -EINVAL;
    }

    if (!IN_RANGE(value, PNL_BASE, PNL_LAST)) {
        pr_debug("Invalid panel data address, range is 0x%02X - 0x%02X\n", PNL_BASE, PNL_LAST);
        return -EINVAL;
    }

    panel_data_address = value;

    return count;
}

static int proc_panel_data_value_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int result = 0;
    u8 value  = 0;

    if (off == 0) {
	panel_data_read(panel_data_address, &value, 1);

	result = sprintf(page, "panel data address : 0x%02X\n\nValue: %c\n\n", panel_data_address, (char) value);
    }

    return result;
}

static int proc_panel_data_whole_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    u8 buffer[PNL_SIZE];
    int result = 0;
    int i = 0, j = 0;
    
    if (off == 0) {
	panel_data_read(PNL_BASE, buffer, PNL_SIZE);
    
	for (i = 0; i < 16; i++) {
	    for (j = 0; j < 16; j++) {
		result += sprintf((page + result), "%c", (char)buffer[(i*16) + j]);
	    }
	    result += sprintf((page + result), "\n");
	}
    }

    return result;
}

#define WF_PROC_MODE_PARENT     (S_IFDIR | S_IRUGO | S_IXUGO)
#define WF_PROC_MODE_CHILD_RW   (S_IWUGO | S_IRUGO)
#define WF_PROC_MODE_CHILD_R    (S_IRUGO)

#define WF_PROC_PARENT          "wf"
#define WF_PROC_PATH            "path"
#define WF_PROC_TYPE            "type"
#define WF_PROC_VERSION         "version"
#define WF_PROC_HUMAN_VERSION   "human_version"
#define WF_PROC_EMBEDDED_CHKSUM "embedded_checksum"
#define WF_PROC_COMPUTED_CHKSUM "computed_checksum"

#define WF_PROC_PANEL_PARENT		"wf/panel"
#define WF_PROC_PANEL_FLASH_READONLY	"flash_readonly"
#define WF_PROC_PANEL_FLASH_SELECT	"flash_select"
#define WF_PROC_PANEL_FLASH_DATA	"flash_data"
#define WF_PROC_PANEL_BCD		"bcd"
#define WF_PROC_PANEL_ID		"panel_id"
#define WF_PROC_PANEL_EXTRACT_PATH	"extract_path"
#define WF_PROC_PANEL_PANEL_DATA_ADDR	"panel_data_addr"
#define WF_PROC_PANEL_PANEL_DATA_VALUE	"panel_data_value"
#define WF_PROC_PANEL_PANEL_DATA_WHOLE	"panel_data_whole"

static struct proc_dir_entry *proc_wf_parent            = NULL;
static struct proc_dir_entry *proc_wf_path              = NULL;
static struct proc_dir_entry *proc_wf_type              = NULL;
static struct proc_dir_entry *proc_wf_version           = NULL;
static struct proc_dir_entry *proc_wf_human_version     = NULL;
static struct proc_dir_entry *proc_wf_embedded_chksum   = NULL;
static struct proc_dir_entry *proc_wf_computed_chksum   = NULL;

static struct proc_dir_entry *proc_wf_panel_parent		= NULL;
static struct proc_dir_entry *proc_wf_panel_flash_readonly	= NULL;
static struct proc_dir_entry *proc_wf_panel_flash_select	= NULL;
static struct proc_dir_entry *proc_wf_panel_flash_data		= NULL;
static struct proc_dir_entry *proc_wf_panel_bcd			= NULL;
static struct proc_dir_entry *proc_wf_panel_id			= NULL;
static struct proc_dir_entry *proc_wf_panel_extract_path	= NULL;
static struct proc_dir_entry *proc_wf_panel_panel_data_addr	= NULL;
static struct proc_dir_entry *proc_wf_panel_panel_data_value	= NULL;
static struct proc_dir_entry *proc_wf_panel_panel_data_whole	= NULL;

static struct proc_dir_entry *create_wf_proc_entry(const char *name, mode_t mode, struct proc_dir_entry *parent,
    read_proc_t *read_proc, write_proc_t *write_proc)
{
    struct proc_dir_entry *wf_proc_entry = create_proc_entry(name, mode, parent);
    
    if ( wf_proc_entry )
    {
        wf_proc_entry->data       = NULL;
        wf_proc_entry->read_proc  = read_proc;
        wf_proc_entry->write_proc = write_proc;
    }
    
    return ( wf_proc_entry );
}

#define remove_wf_proc_entry(name, entry, parent)   \
    do                                              \
    if ( entry )                                    \
    {                                               \
        remove_proc_entry(name, parent);            \
        entry = NULL;                               \
    }                                               \
    while ( 0 )

static void wf_cleanup(void)
{
	if ( proc_wf_parent )
	{
	    if (proc_wf_panel_parent) 
	    {
		remove_wf_proc_entry(WF_PROC_PANEL_FLASH_READONLY, proc_wf_panel_flash_readonly, proc_wf_panel_parent);
		remove_wf_proc_entry(WF_PROC_PANEL_FLASH_SELECT, proc_wf_panel_flash_select, proc_wf_panel_parent);
		remove_wf_proc_entry(WF_PROC_PANEL_FLASH_DATA, proc_wf_panel_flash_data, proc_wf_panel_parent);
		remove_wf_proc_entry(WF_PROC_PANEL_BCD, proc_wf_panel_bcd, proc_wf_panel_parent);
		remove_wf_proc_entry(WF_PROC_PANEL_ID, proc_wf_panel_id, proc_wf_panel_parent);
		remove_wf_proc_entry(WF_PROC_PANEL_EXTRACT_PATH, proc_wf_panel_extract_path, proc_wf_panel_parent);
		remove_wf_proc_entry(WF_PROC_PANEL_PANEL_DATA_ADDR, proc_wf_panel_panel_data_addr, proc_wf_panel_parent);
		remove_wf_proc_entry(WF_PROC_PANEL_PANEL_DATA_VALUE, proc_wf_panel_panel_data_value, proc_wf_panel_parent);
		remove_wf_proc_entry(WF_PROC_PANEL_PANEL_DATA_WHOLE, proc_wf_panel_panel_data_whole, proc_wf_panel_parent);

		remove_wf_proc_entry(WF_PROC_PANEL_PARENT, proc_wf_panel_parent, NULL);
	    }

	    remove_wf_proc_entry(WF_PROC_PATH,              proc_wf_path,               proc_wf_parent);
	    remove_wf_proc_entry(WF_PROC_TYPE,              proc_wf_type,               proc_wf_parent);
	    remove_wf_proc_entry(WF_PROC_VERSION,           proc_wf_version,            proc_wf_parent);
	    remove_wf_proc_entry(WF_PROC_HUMAN_VERSION,     proc_wf_human_version,      proc_wf_parent);
	    remove_wf_proc_entry(WF_PROC_EMBEDDED_CHKSUM,   proc_wf_embedded_chksum,    proc_wf_parent);
	    remove_wf_proc_entry(WF_PROC_COMPUTED_CHKSUM,   proc_wf_computed_chksum,    proc_wf_parent);
	    
	    remove_wf_proc_entry(WF_PROC_PARENT,            proc_wf_parent,             NULL);

	}

}

static int __init wf_init(void)
{
    int result = -ENOMEM;
    int null_check = -1;
    
    // Parent:  /proc/wf.
    //
    proc_wf_parent = create_proc_entry(WF_PROC_PARENT, WF_PROC_MODE_PARENT, NULL);
    
    if ( proc_wf_parent )
    {
        // Child:   /proc/wf/path.
        //
        proc_wf_path = create_wf_proc_entry(WF_PROC_PATH, WF_PROC_MODE_CHILD_RW, proc_wf_parent,
            proc_path_read, proc_path_write);
        
        null_check &= (int)proc_wf_path;

        // Child:   /proc/wf/type.
        //
        proc_wf_type = create_wf_proc_entry(WF_PROC_TYPE, WF_PROC_MODE_CHILD_RW, proc_wf_parent,
            proc_type_read, proc_type_write);
            
        null_check &= (int)proc_wf_type;
        
        // Child:   /proc/wf/version.
        //
        proc_wf_version = create_wf_proc_entry(WF_PROC_VERSION, WF_PROC_MODE_CHILD_R, proc_wf_parent,
            proc_version_read, NULL);
            
        null_check &= (int)proc_wf_version;

        // Child:   /proc/wf/human_version.
        //
        proc_wf_human_version = create_wf_proc_entry(WF_PROC_HUMAN_VERSION, WF_PROC_MODE_CHILD_R, proc_wf_parent,
            proc_human_version_read, NULL);
            
        null_check &= (int)proc_wf_human_version;
        
        // Child:   /proc/wf/embedded_checksum
        //
        proc_wf_embedded_chksum = create_wf_proc_entry(WF_PROC_EMBEDDED_CHKSUM, WF_PROC_MODE_CHILD_R,
            proc_wf_parent, proc_embedded_checksum_read, NULL);
            
        null_check &= (int)proc_wf_embedded_chksum;

        // Child:   /proc/wf/computed_checksum
        //
        proc_wf_computed_chksum = create_wf_proc_entry(WF_PROC_COMPUTED_CHKSUM, WF_PROC_MODE_CHILD_R,
            proc_wf_parent, proc_computed_checksum_read, NULL);
            
        null_check &= (int)proc_wf_embedded_chksum;

        // Success? 
        //
        if ( 0 == null_check ) {

	    pr_debug("failed to create /proc entries!\n");
            wf_cleanup();
	    goto exit;
	} else {
            result = 0;
	}
    }

    if (!panel_flash_init() || !panel_flash_present()) {

	/* it's ok not to have panel flash */
	pr_debug("failed to init panel flash!\n");

	proc_wf_panel_parent = NULL;
	result = 0;

	goto exit;

    } else {

	null_check = -1;

        // Parent:   /proc/wf/panel.
        //
        proc_wf_panel_parent = create_proc_entry(WF_PROC_PANEL_PARENT, WF_PROC_MODE_PARENT, NULL);
        
	if ( proc_wf_panel_parent ) {
	
	    // Child:   /proc/wf/panel/flash_readonly
	    //
	    proc_wf_panel_flash_readonly = create_wf_proc_entry(WF_PROC_PANEL_FLASH_READONLY, WF_PROC_MODE_CHILD_R,
								proc_wf_panel_parent, proc_panel_flash_readonly_read, NULL);
            
	    null_check &= (int)proc_wf_panel_flash_readonly;

	    // Child:   /proc/wf/panel/flash_select
	    //
	    proc_wf_panel_flash_select = create_wf_proc_entry(WF_PROC_PANEL_FLASH_SELECT, WF_PROC_MODE_CHILD_RW, proc_wf_panel_parent,
							      proc_panel_flash_select_read, proc_panel_flash_select_write);
            
	    null_check &= (int)proc_wf_panel_flash_select;
        
	    // Child:   /proc/wf/panel/flash_data
	    //
	    proc_wf_panel_flash_data = create_wf_proc_entry(WF_PROC_PANEL_FLASH_DATA, WF_PROC_MODE_CHILD_RW, proc_wf_panel_parent,
							      proc_panel_flash_data_read, proc_panel_flash_data_write);
            
	    null_check &= (int)proc_wf_panel_flash_data;

	    // Child:   /proc/wf/panel/bcd
	    //
	    proc_wf_panel_bcd = create_wf_proc_entry(WF_PROC_PANEL_BCD, WF_PROC_MODE_CHILD_R, proc_wf_panel_parent,
							      proc_panel_bcd_read, NULL);
            
	    null_check &= (int)proc_wf_panel_bcd;

	    // Child:   /proc/wf/panel/panel_id
	    //
	    proc_wf_panel_id = create_wf_proc_entry(WF_PROC_PANEL_ID, WF_PROC_MODE_CHILD_R, proc_wf_panel_parent,
							      proc_panel_id_read, NULL);
            
	    null_check &= (int)proc_wf_panel_id;

	    // Child:   /proc/wf/panel/extract_path
	    //
	    proc_wf_panel_extract_path = create_wf_proc_entry(WF_PROC_PANEL_EXTRACT_PATH, WF_PROC_MODE_CHILD_RW, proc_wf_panel_parent,
							      proc_panel_extract_path_read, proc_panel_extract_path_write);
            
	    null_check &= (int)proc_wf_panel_extract_path;

	    // Child:   /proc/wf/panel/panel_data_addr
	    //
	    proc_wf_panel_panel_data_addr = create_wf_proc_entry(WF_PROC_PANEL_PANEL_DATA_ADDR, WF_PROC_MODE_CHILD_RW, proc_wf_panel_parent,
							      proc_panel_data_addr_read, proc_panel_data_addr_write);
            
	    null_check &= (int)proc_wf_panel_panel_data_addr;

	    // Child:   /proc/wf/panel/panel_data_value
	    //
	    proc_wf_panel_panel_data_value = create_wf_proc_entry(WF_PROC_PANEL_PANEL_DATA_VALUE, WF_PROC_MODE_CHILD_R, proc_wf_panel_parent,
							      proc_panel_data_value_read, NULL);
            
	    null_check &= (int)proc_wf_panel_panel_data_value;

	    // Child:   /proc/wf/panel/panel_data_whole
	    //
	    proc_wf_panel_panel_data_whole = create_wf_proc_entry(WF_PROC_PANEL_PANEL_DATA_WHOLE, WF_PROC_MODE_CHILD_R, proc_wf_panel_parent,
							      proc_panel_data_whole_read, NULL);
            
	    null_check &= (int)proc_wf_panel_panel_data_whole;

	}
	
        // Success? 
        //
        if ( 0 == null_check ) {
            wf_cleanup();
	    goto exit;
	} else {
            result = 0;
	}
    }

    // Done.
    //
  exit:
    return ( result );
}

static void __exit wf_exit(void)
{
    pr_debug("%s: begin\n", __FUNCTION__);

    panel_flash_exit();
    wf_cleanup();

    pr_debug("%s: done\n", __FUNCTION__);
}

module_init(wf_init);
module_exit(wf_exit);

EXPORT_SYMBOL(einkwf_set_buffer_size);
EXPORT_SYMBOL(einkwf_get_buffer_size);

EXPORT_SYMBOL(einkwf_set_buffer);
EXPORT_SYMBOL(einkwf_get_buffer);

EXPORT_SYMBOL(einkwf_get_waveform_info);
EXPORT_SYMBOL(einkwf_get_waveform_version_string);
EXPORT_SYMBOL(einkwf_waveform_valid);
EXPORT_SYMBOL(einkwf_write_waveform_to_file);
EXPORT_SYMBOL(einkwf_read_waveform_from_file);
EXPORT_SYMBOL(einkwf_get_wavform_proxy_from_waveform);

EXPORT_SYMBOL(einkwf_get_commands_info);
EXPORT_SYMBOL(einkwf_get_commands_version_string);
EXPORT_SYMBOL(einkwf_commands_valid);

EXPORT_SYMBOL(einkwf_panel_flash_present);
EXPORT_SYMBOL(einkwf_panel_get_waveform);
EXPORT_SYMBOL(einkwf_panel_waveform_free);
EXPORT_SYMBOL(einkwf_panel_get_waveform_mode);
EXPORT_SYMBOL(einkwf_panel_set_update_modes);

EXPORT_SYMBOL(einkwf_panel_get_vcom);
EXPORT_SYMBOL(einkwf_panel_get_vcom_str);
EXPORT_SYMBOL(einkwf_panel_supports_vcom);

EXPORT_SYMBOL(einkwf_panel_get_waveform_info);
EXPORT_SYMBOL(einkwf_panel_get_waveform_version_string);

MODULE_DESCRIPTION("eink waveform/commands header parser");
MODULE_AUTHOR("Lab126");
MODULE_LICENSE("GPL");
