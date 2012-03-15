
#include <linux/spi/spi.h>
#include <linux/zlib.h>
#include <linux/vmalloc.h>
#include <linux/mxcfb.h>

#include "eink_panel.h"
#include "builtin_waveform.h"

static u8   panel_waveform_buffer[EINK_WAVEFORM_FILESIZE] = { 0 };
static bool panel_waveform_header_only = false;
static int  panel_waveform_size = 0;
static bool spi_registered = false;
static char *waveform_proxy_buffer = NULL;

static panel_flash_select get_flash_select(void);
static void set_flash_select(panel_flash_select flash_select);
static int panel_read_from_flash(unsigned long addr, unsigned char *data, unsigned long size);
static u8 *panel_use_builtin_waveform(int *waveform_proxy_size);
static u8 *use_panel_proxy(int *waveform_proxy_size);

#define HEAD_CRC	2
#define EXTRA_FIELD	4
#define ORIG_NAME	8
#define COMMENT		0x10
#define RESERVED	0xe0

#define DEFLATED	8

#define HEADER_0	0x1f
#define HEADER_1	0x8b
#define BEST_COMP	2
#define UNIX_OS		3

static void *z_inflate_workspace = NULL;

static int init_z_inflate_workspace(void)
{
	int result = 0;
	
	if (!z_inflate_workspace) {
		z_inflate_workspace = kzalloc(zlib_inflate_workspacesize(), GFP_ATOMIC);
	}
	
	if (z_inflate_workspace) {
		result = 1;
	}

	return (result);
}

static void done_z_inflate_workspace(void)
{
	if (z_inflate_workspace) {
		kfree(z_inflate_workspace);
		z_inflate_workspace = NULL;
	}
}

static int panel_gunzip(unsigned char *dst, int dstlen, unsigned char *src, unsigned long *lenp)
{
	z_stream s;
	int r, i, flags;

	if (!init_z_inflate_workspace()) {
		pr_debug ("Error: gunzip failed to allocate workspace\n");
		return (-1);
	}
	
	/* skip header */
	i = 10;
	flags = src[3];
	if (src[2] != DEFLATED || (flags & RESERVED) != 0) {
		pr_debug ("Error: Bad gzipped data\n");
		return (-1);
	}
	if ((flags & EXTRA_FIELD) != 0)
		i = 12 + src[10] + (src[11] << 8);
	if ((flags & ORIG_NAME) != 0)
		while (src[i++] != 0)
			;
	if ((flags & COMMENT) != 0)
		while (src[i++] != 0)
			;
	if ((flags & HEAD_CRC) != 0)
		i += 2;
	if (i >= *lenp) {
		pr_debug ("Error: gunzip out of data in header\n");
		return (-1);
	}

	/* Initialize ourself. */
	s.workspace = z_inflate_workspace;
	r = zlib_inflateInit2(&s, -MAX_WBITS);
	if (r != Z_OK) {
		pr_debug ("Error: zlib_inflateInit2() returned %d\n", r);
		return (-1);
	}
	s.next_in = src + i;
	s.avail_in = *lenp - i;
	s.next_out = dst;
	s.avail_out = dstlen;
	r = zlib_inflate(&s, Z_FINISH);
	if (r != Z_OK && r != Z_STREAM_END) {
		pr_debug ("Error: zlib_inflate() returned %d\n", r);
		return (-1);
	}
	*lenp = s.next_out - (unsigned char *) dst;
	zlib_inflateEnd(&s);

	done_z_inflate_workspace();

	return (0);
}

/**
 * Waveform API
 **/

static u8 *panel_get_waveform_from_flash(int offset, u8 *buffer, int buffer_size)
{
    if ( panel_flash_present() )
    {
        panel_flash_select saved_flash_select = get_flash_select();
        
        pr_debug("Reading waveform.. (%d bytes)\n", buffer_size);
        set_flash_select(panel_flash_waveform);
        panel_read_from_flash(offset, buffer, buffer_size);
        
        set_flash_select(saved_flash_select);
    }
    else
        memset(&buffer[offset], 0, buffer_size);
    
    return ( buffer );
}

static void panel_get_waveform(u8 *buffer, int buffer_size)
{
    pr_debug("%s: begin\n", __FUNCTION__);

    // If we haven't read the waveform in before (or we need to read it again), read it now.
    //
    if ( 0 == panel_waveform_size )
    {
        u8 *which_buffer = buffer ? buffer      : panel_get_waveform_from_flash(0, panel_waveform_buffer, WFM_HDR_SIZE);
        int which_size   = buffer ? buffer_size : WFM_HDR_SIZE;
        
	pr_debug("%s: reading waveform\n", __FUNCTION__);

        memcpy(panel_waveform_buffer, which_buffer, which_size);
        einkwf_set_buffer_size(which_size);
        einkwf_set_buffer(panel_waveform_buffer);
    
        // We may end up having to re-read it if the initial read and/or subsequent
        // reads are invalid.
        //
        if ( einkwf_waveform_valid() )
        {
            eink_waveform_info_t waveform_info;
            einkwf_get_waveform_info(&waveform_info);

            // If we're actually reading from flash itself, we won't have been passed a
            // buffer to use.  And, in that case, we only read in the header to validate
            // with.  Now that we know it's valid, we read in the rest of it unless
            // we've been told not to.
            //
            if ( !buffer && !panel_waveform_header_only ) {

		pr_debug("%s: reading waveform from flash\n", __FUNCTION__);

                panel_get_waveform_from_flash(WFM_HDR_SIZE,
                    (which_buffer + WFM_HDR_SIZE),
                    (waveform_info.filesize - WFM_HDR_SIZE));
	    }

	    pr_debug("%s: read waveform size %ld\n", __FUNCTION__, waveform_info.filesize);

            panel_waveform_size = waveform_info.filesize;
            einkwf_set_buffer_size(panel_waveform_size);
        }
    }
    else
    {

	pr_debug("%s: setting buffer size = %d\n", __FUNCTION__, buffer_size);

        einkwf_set_buffer_size(panel_waveform_size);
        einkwf_set_buffer(panel_waveform_buffer);
    }
}

bool panel_set_waveform(u8 *buffer, int buffer_size)
{
    // Force us to re-validate the waveform (either from flash or the passed in buffer).
    //
    panel_waveform_size = 0;
    
    // Get the waveform into our local buffer.
    //
    panel_get_waveform(buffer, buffer_size);
    
    // Say whether we set it or not.
    //
    return ( 0 != panel_waveform_size );
}

void panel_get_waveform_info(eink_waveform_info_t *info)
{
    if ( info )
    {
	panel_get_waveform(NULL, 0);
        einkwf_get_waveform_info(info);	
    }
}

char *panel_get_waveform_version_string(eink_waveform_version_string_t which_string)
{
    panel_get_waveform(NULL, 0);
    return ( einkwf_get_waveform_version_string(which_string) );
}

bool panel_waveform_valid(void)
{
    panel_get_waveform(NULL, 0);
    return ( einkwf_waveform_valid() );    
}

static bool inline use_builtin_waveform(char *waveform_file_path)
{
    return ( (waveform_file_path == NULL) || (strlen(waveform_file_path) == 0) ||
	     (0 == strcmp(waveform_file_path, EINK_WF_USE_BUILTIN_WAVEFORM)) );
}

static u8 *read_waveform_file_proxy(char *waveform_file_proxy, int *waveform_proxy_size)
{
    u8 *result = vmalloc(*waveform_proxy_size);
    
    if ( result )
    {
        einkwf_set_buffer_size(*waveform_proxy_size);
        einkwf_set_buffer(result);
    
        if ( einkwf_read_waveform_from_file(waveform_file_proxy) )
        {
            vfree(result);
            result = NULL;
        }
        else
            *waveform_proxy_size = einkwf_get_buffer_size();
    }

    return ( result );
}

u8 *panel_read_waveform_from_file(char *waveform_file_path, int *waveform_proxy_size)
{
    u8 *result = NULL;

    if (waveform_file_path == NULL) {
	printk(KERN_ERR "%s: waveform path is NULL!\n", __FUNCTION__);
	return NULL;
    }
    
    if ( waveform_proxy_size )
    {
	if ( strlen(waveform_file_path) == 0 ) 
	{

	    // No file path specified, check to see whether the panel's proxy file has stored away.
	    //
	    result = use_panel_proxy(waveform_proxy_size);
    
	    if ( result != NULL )
	    {
		strcpy(waveform_file_path, "stored");
		waveform_proxy_buffer = result;
		return ( result );
	    } 
	    else 
	    {
		// Try the built-in waveform
		//
		strcpy(waveform_file_path, EINK_WF_USE_BUILTIN_WAVEFORM);
	    }
	}

        // Check to see whether we need to use the built-in waveform or not.
        //
        if ( use_builtin_waveform(waveform_file_path) )
        {
            result = panel_use_builtin_waveform(waveform_proxy_size);
        }
        else
        {
            einkwf_set_buffer_size(EINK_WAVEFORM_FILESIZE);
            einkwf_set_buffer(panel_waveform_buffer);
    
            // Otherwise, attempt to read the waveform file itself in.
            //
            if ( einkwf_read_waveform_from_file(waveform_file_path) == 0 )
            {
                // If it's valid...
                //
                if ( panel_set_waveform(panel_waveform_buffer, einkwf_get_buffer_size()) )
                {
                    char waveform_file_proxy[WF_PATH_LEN] = { 0 };
                    
                    // ...get the path to the proxy.
                    //
                    if ( einkwf_get_wavform_proxy_from_waveform(waveform_file_path, waveform_file_proxy) )
                    {
                        *waveform_proxy_size = EINK_WAVEFORM_PROXY_SIZE;
                        
                        // Now, attempt to read the waveform file proxy in.
                        //
                        result = read_waveform_file_proxy(waveform_file_proxy, waveform_proxy_size);
                    }
                }
            }
        }
    }
    
    waveform_proxy_buffer = result;
    return ( result );
}

#define WAVEFORM_PROXY_GZIPPED_PATH "/mnt/wfm/waveform_to_use.gz"

static u8 *use_panel_proxy(int *waveform_proxy_size)
{
    u8 *result = NULL, *waveform_proxy_gzipped = NULL;
    *waveform_proxy_size = EINK_WAVEFORM_FILESIZE;
    
    // First, attempt to read the gzipped waveform proxy file in.
    //
    waveform_proxy_gzipped = read_waveform_file_proxy(WAVEFORM_PROXY_GZIPPED_PATH, waveform_proxy_size);

    if ( waveform_proxy_gzipped )
    {
        result = vmalloc(EINK_WAVEFORM_PROXY_SIZE);
        
        if ( result )
        {
            // Now, attempt to gunzip the waveform proxy file.
            //
            if ( panel_gunzip(result, EINK_WAVEFORM_PROXY_SIZE, waveform_proxy_gzipped, (unsigned long *)waveform_proxy_size) == 0 )
            {
                pr_debug("using stored waveform\n");
                
                // Say that we just need the waveform header information from the panel.
                //
                panel_waveform_header_only = true;
                panel_set_waveform(NULL, 0);
            }
            else
            {
                vfree(result);
                result = NULL;
            }
        }
        
        // Free the gzipped buffer as we no longer need it.
        //
        vfree(waveform_proxy_gzipped);
    }
    
    return ( result );
}

static u8 *panel_use_builtin_waveform(int *waveform_proxy_size)
{
    u8 *result = NULL;
    unsigned long waveform_size = SIZEOF_BUILTIN_WBF_BUFF;
    
    pr_debug("trying to use builtin waveform\n");

    // If the built-in waveform is valid...
    //
    if ( panel_gunzip(panel_waveform_buffer, SIZEOF_BUILTIN_WBF_BUFF, builtin_wbf, &waveform_size) == 0 )
    {
	if ( panel_set_waveform(panel_waveform_buffer, waveform_size) )
	{
	    //...return its proxy.
	    //
	    result = vmalloc(SIZEOF_BUILTIN_WRF_BUFF);
                
	    if ( result )
	    {
		*waveform_proxy_size = SIZEOF_BUILTIN_WRF_FULL;
                    
		if ( panel_gunzip(result, SIZEOF_BUILTIN_WRF_BUFF, builtin_wrf, (unsigned long *)waveform_proxy_size) )
		{
		    vfree(result);

		    pr_debug(".wrf gunzip failed\n");
		    result = NULL;
		}
		else
		{
		    printk(KERN_ERR "Using 25C-only waveform!\n");
		}
	    } else {
		pr_debug("couldn't alloc memory for .wrf\n");
	    }
	} else {
	    pr_debug("couldn't set waveform\n");
	}
    } else {
	pr_debug("failed to unzip\n");
    }
    
    return ( result );
}

char *panel_get_proxy_buffer(void)
{
    return waveform_proxy_buffer;
}

static void panel_waveform_free(u8 *waveform_proxy)
{
    if ( waveform_proxy ) {
        vfree(waveform_proxy);
	waveform_proxy_buffer = NULL;
    }
}

/**
 * Waveform Update Modes.
 **/

#define WF_UPD_MODES_00         0       // V100 modes:
//#define WF_UPD_MODE_INIT      0       //  INIT (panel initialization)
//#define WF_UPD_MODE_MU        1       //  MU   (monochrome update)
//#define WF_UPD_MODE_GU        2       //  GU   (grayscale update)
//#define WF_UPD_MODE_GC        3       //  GC   (grayscale clear)
//#define WF_UPD_MODE_PU        4       //  PU   (pen update)

#define WF_UPD_MODES_01         1       // V110/V110A modes:
#define WF_UPD_MODES_02         2       //
//#define WF_UPD_MODE_INIT      0       //  INIT
#define WF_UPD_MODE_DU          1       //  DU   (direct update,    1bpp)
#define WF_UPD_MODE_GC16        2       //  GC16 (grayscale clear,  4bpp)
#define WF_UPD_MODE_GC4         3       //  GC4  (grayscale clear,  2bpp)

#define WF_UPD_MODES_03         3       // V220 50Hz/85Hz modes:
//#define WF_UPD_MODE_INIT      0       //  INIT
//#define WF_UPD_MODE_DU        1       //  DU
//#define WF_UPD_MODE_GC16      2       //  GC16
//#define WF_UPD_MODE_GC4       3       //  GC4
#define WF_UPD_MODE_AU          4       //  AU   (animation update, 1bpp)

#define WF_UPD_MODES_04         4       // V220 85Hz modes
//#define WF_UPD_MODE_INIT      0       //  INIT
//#define WF_UPD_MODE_DU        1       //  DU
//#define WF_UPD_MODE_GC16      2       //  GC16
#define WF_UPD_MODE_A2          3       //  AU
#define WF_UPD_MODE_GL          5       //  GL   (white transition, grayscale clear, 4bpp)
                                        //       (keeping it at 5, to match WAVEFORM_MODE_GL16)

#define WF_UPD_MODE(u)          \
    (wf_upd_modes ? wf_upd_modes[(u)] : (u))

static int wf_upd_modes_00[] =
{
    WF_UPD_MODE_INIT,
    WF_UPD_MODE_MU,
    WF_UPD_MODE_GU,
    WF_UPD_MODE_GC,
    WF_UPD_MODE_PU,
    WF_UPD_MODE_GC
};

// Note:  The names are in the already-translated positions.
//
static char *wf_upd_mode_00_names[] =
{
    "INIT",
    "MU",
    "GU",
    "GC",
    "PU",
    "GC",
};

static int wf_upd_modes_01[] =
{
    WF_UPD_MODE_INIT,
    WF_UPD_MODE_DU,
    WF_UPD_MODE_GC4,
    WF_UPD_MODE_GC16,
    WF_UPD_MODE_DU,
    WF_UPD_MODE_GC16,
};

static char *wf_upd_mode_01_names[] =
{
    "INIT",
    "DU",
    "GC4",
    "GC16",
    "DU",
    "GC16",
};

static int wf_upd_modes_03[] =
{
    WF_UPD_MODE_INIT,
    WF_UPD_MODE_DU,
    WF_UPD_MODE_GC4,
    WF_UPD_MODE_GC16,
    WF_UPD_MODE_AU,
    WF_UPD_MODE_GC16,
};

static char *wf_upd_mode_03_names[] =
{
    "INIT",
    "DU",
    "GC4",
    "GC16",
    "AU",
    "GC16",
};

static int wf_upd_modes_04[] =
{
    WF_UPD_MODE_INIT,
    WF_UPD_MODE_DU,
    WF_UPD_MODE_GC16,
    WF_UPD_MODE_GC16,
    WF_UPD_MODE_A2,
    WF_UPD_MODE_GL,
};

static char *wf_upd_mode_04_names[] =
{
    "INIT",
    "DU",
    "GC16",
    "GC16",
    "A2",
    "GL16",
};

static unsigned char wf_upd_mode_version = WF_UPD_MODES_00;
static char **wf_upd_mode_names = NULL;
static int *wf_upd_modes = NULL;

static void panel_set_upd_modes(void)
{
    // Don't re-read the waveform to set up the modes again unless we have to.
    //
    if ( !wf_upd_modes )
    {
        eink_waveform_info_t info;
        panel_get_waveform_info(&info);

        switch ( info.waveform.mode_version )
        {
            case WF_UPD_MODES_00:
            default:
                wf_upd_mode_version = WF_UPD_MODES_00;
                wf_upd_mode_names = wf_upd_mode_00_names;
                wf_upd_modes = wf_upd_modes_00;
            break;
            
            case WF_UPD_MODES_01:
            case WF_UPD_MODES_02:
                wf_upd_mode_version = WF_UPD_MODES_01;
                wf_upd_mode_names = wf_upd_mode_01_names;
                wf_upd_modes = wf_upd_modes_01;
            break;
            
            case WF_UPD_MODES_03:
                wf_upd_mode_version = WF_UPD_MODES_03;
                wf_upd_mode_names = wf_upd_mode_03_names;
                wf_upd_modes = wf_upd_modes_03;
            break;
            
            case WF_UPD_MODES_04:
                wf_upd_mode_version = WF_UPD_MODES_04;
                wf_upd_mode_names = wf_upd_mode_04_names;
                wf_upd_modes = wf_upd_modes_04;
            break;
        }
        
        pr_debug("waveform mode version = %d\n", wf_upd_mode_version);
    }
}

static bool panel_upd_mode_valid(int upd_mode)
{
    bool result = false;
    
    switch ( upd_mode )
    {
        // The upd_mode needs to be one of the canonical values, even though
        // it might (will) be translated into something different.
        //
        case WF_UPD_MODE_INIT:
        case WF_UPD_MODE_MU:
        case WF_UPD_MODE_GU:
        case WF_UPD_MODE_GC:
        case WF_UPD_MODE_PU:
        case WF_UPD_MODE_GL:
            result = true;
        break;
    }
    
    return ( result );
}

static int panel_get_waveform_mode(int upd_mode)
{
    int result = WF_UPD_MODE_INVALID;
    
    // Translate the canonical mode into what
    // the waveform in use can do.
    //
    if ( panel_upd_mode_valid(upd_mode) )
	result = WF_UPD_MODE(upd_mode);
    else
    {
	// But pass the auto-update "mode" through directly.
	//
	if ( WF_UPD_MODE_AUTO == upd_mode )
	    result = WF_UPD_MODE_AUTO;
	else
	    pr_debug("invalid upd_mode = %d/n", upd_mode);
    }

    return ( result );
}

static char *panel_get_waveform_mode_name(int upd_mode)
{
    char *result = "??";

    if ( !panel_upd_mode_valid(upd_mode) )
    {
        if ( WF_UPD_MODE_AUTO == upd_mode )
            result = "auto";
    }
    else
    {
        if ( wf_upd_mode_names )
            result = wf_upd_mode_names[upd_mode];
    }
    
    return ( result );
}

/**
 * Panel Data API
 **/

enum panel_data_characters
{
    zero = 0x0, one, two, three, four, five, six, seven, eight, nine,
    underline = 0x0a, dot = 0x0b, negative = 0x0c,
    _a = 0xcb, _b, _c, _d, _e, _f, _g, _h, _i, _j, _k, _l, _m, _n,
               _o, _p, _q, _r, _s, _t, _u, _v, _w, _x, _y, _z,
    
    _A = 0xe5, _B, _C, _D, _E, _F, _G, _H, _I, _J, _K, _L, _M, _N,
               _O, _P, _Q, _R, _S, _T, _U, _V, _W, _X, _Y, _Z
};
typedef enum panel_data_characters panel_data_characters;

static void panel_data_translate(u8 *buffer, int to_read) {
    int i = 0;
    
    for (i = 0; i < to_read; i++) {
        if (buffer[i] >= _a && buffer[i] <= _z) {
            buffer[i] = 'a' + (buffer[i] - _a);
        } else if (buffer[i] >= _A && buffer[i] <= _Z) {
            buffer[i] = 'A' + (buffer[i] - _A);
        } else if (/* buffer[i] >= zero && */ buffer[i] <= nine) {
            buffer[i] = '0' + (buffer[i] - zero);
        } else if (buffer[i] == underline) {
            buffer[i] = '_';
        } else if (buffer[i] == dot) {
            buffer[i] = '.';
        } else if (buffer[i] == negative) {
            buffer[i] = '-';
        } else {
            buffer[i] = PNL_CHAR_UNKNOWN;
        }
    }
}

static bool supports_panel_data_read(void) {
    return panel_flash_present();
}

static int panel_data_read(u32 start_addr, u8 *buffer, int to_read) {
    int result = -1;
    
    if (supports_panel_data_read() && buffer && IN_RANGE(to_read, 1, PNL_SIZE)) {
        panel_flash_select saved_flash_select = get_flash_select();
        set_flash_select(panel_flash_commands);
        
        panel_read_from_flash((PNL_FLASH_BASE+start_addr), buffer, to_read);
        set_flash_select(saved_flash_select);
        
        panel_data_translate((u8 *)buffer, to_read);
        result = to_read;
    }

    return result;
}

#define PANEL_ID_UNKNOWN    "????_???_??_???"
#define PNL_SIZE_ID_STR     32

static char panel_bcd[PNL_SIZE_BCD_STR] = { 0 };
static char panel_vcom_str[PNL_SIZE_VCOM_STR] = { 0 };
static char panel_id[PNL_SIZE_ID_STR]   = { 0 };
static int  panel_vcom = 0;

bool supports_panel_bcd(void)
{
    return ( panel_flash_present() );
}

char *panel_get_bcd(void)
{
    if ( !supports_panel_bcd()) {
	return NULL;
    }

    // If the panel ID hasn't already been read in, then read it in now.
    //
    if ( panel_bcd[0] == 0 )
    {
        u8 bcd[PNL_SIZE_BCD] = { 0 };

        panel_data_read(PNL_BASE_BCD, bcd, PNL_SIZE_BCD);
        strncpy(panel_bcd, bcd, PNL_SIZE_BCD);
        panel_bcd[PNL_SIZE_BCD] = '\0';
    }

    pr_debug("%s: panel bcd=%s\n", __FUNCTION__, panel_bcd);
    
    return ( panel_bcd );
}

static bool supports_panel_id(void)
{
    return ( panel_flash_present() );
}

static bool panel_data_valid(char *panel_data)
{
    bool result = false;
    
    if ( panel_data )
    {
        if ( strchr(panel_data, PNL_CHAR_UNKNOWN) )
        {
            printk(KERN_ERR "Unrecognized values in panel data\n");
            pr_debug("panel data = %s\n", panel_data);
        }
        else
            result = true;
    }
    
    return ( result );
}

char *panel_get_id(void)
{
    if ( ! supports_panel_id() )
	return NULL;

    // If the panel ID hasn't already been read in, then read it in now.
    //
    if ( !(('_' == panel_id[4]) && ('_' == panel_id[8]) && ('_' == panel_id[11])) )
    {
        u8 panel_buffer[PNL_SIZE] = { 0 };
        char *part_number;
	int cur;

        // Waveform file names are of the form PPPP_XLLL_DD_TTVVSS_B, and
        // panel IDs are of the form PPPP_LLL_DD_MMM.
        //
        panel_data_read(PNL_BASE, panel_buffer, PNL_SIZE);

        // The platform is (usually) the PPPP substring.  And, in those cases, we copy
        // the platform data from the EEPROM's waveform name.  However, we must special-case
        // the V220E waveforms since EINK isn't using the same convention as they did in
        // the V110A case (i.e., they named V110A waveforms 110A but they are just
        // calling the V220E waveforms V220 with a run-type of E; run-type is the X
        // field in the PPPP_XLLL_DD_TTVVSS_B part of waveform file names).
        //
        switch ( panel_buffer[PNL_BASE_WAVEFORM+5] )
        {
            case 'E':
                panel_id[0] = '2';
                panel_id[1] = '2';
                panel_id[2] = '0';
                panel_id[3] = 'E';
            break;

            default:
                panel_id[0] = panel_buffer[PNL_BASE_WAVEFORM+0];
                panel_id[1] = panel_buffer[PNL_BASE_WAVEFORM+1];
                panel_id[2] = panel_buffer[PNL_BASE_WAVEFORM+2];
                panel_id[3] = panel_buffer[PNL_BASE_WAVEFORM+3];
            break;
        }

        panel_id[ 4] = '_';

        // The lot number (aka FPL) is the the LLL substring:  Just
        // copy the number itself, skipping the batch (X) designation.
        //
        panel_id[ 5] = panel_buffer[PNL_BASE_FPL+1];
        panel_id[ 6] = panel_buffer[PNL_BASE_FPL+2];
        panel_id[ 7] = panel_buffer[PNL_BASE_FPL+3];

        panel_id[ 8] = '_';

        // The display size is the the DD substring.
        //
        panel_id[ 9] = panel_buffer[PNL_BASE_WAVEFORM+10];
        panel_id[10] = panel_buffer[PNL_BASE_WAVEFORM+11];
        panel_id[11] = '_';

	/* Copy in the full part number */
	part_number = &panel_buffer[PNL_BASE_PART_NUMBER];
	cur = 0;
	while (cur < PNL_SIZE_PART_NUMBER && part_number[cur] != PNL_CHAR_UNKNOWN) {
	    panel_id[12+cur] = part_number[cur];
	    cur++;
	}

	panel_id[12+cur] = 0;

        if ( !panel_data_valid(panel_id) )
            strcpy(panel_id, PANEL_ID_UNKNOWN);
    }

    pr_debug("%s: panel id=%s\n", __FUNCTION__, panel_id);

    return ( panel_id );
}

#define VCOM_INT_TO_STR(i, s)   \
    sprintf((s), "-%1d.%02d", ((i)/100)%10, (i)%100)

#define VCOM_STR_READ()      \
    (('-' == panel_vcom_str[0]) && ('.' == panel_vcom_str[2]))

static char *panel_get_vcom_str(void)
{   
    pr_debug("%s begin\n", __FUNCTION__);

    // If the VCOM hasn't already been read in, read it in now.
    //
    if ( !VCOM_STR_READ() )
    {
        u8 vcom_str[PNL_SIZE_VCOM] = { 0 };

        panel_data_read(PNL_BASE_VCOM, vcom_str, PNL_SIZE_VCOM);
        strncpy(panel_vcom_str, vcom_str, PNL_SIZE_VCOM);
        
        // If the VCOM string returned from the panel data is invalid, then
        // use the default one instead.
        //
        panel_vcom_str[PNL_SIZE_VCOM] = '\0';
        
        if ( !panel_data_valid(panel_vcom_str) )
        {
	    return NULL;
        }
    }

    pr_debug("%s vcom=%s\n", __FUNCTION__, panel_vcom_str);

    return ( panel_vcom_str );
}

#define VCOM_READ() \
    (0 != panel_vcom)

static int panel_get_vcom(void)
{
    pr_debug("%s begin\n", __FUNCTION__);

    if ( !VCOM_READ() )
    {
        char *vcom_str = panel_get_vcom_str();
        int i;

	if (vcom_str == NULL) {
	    panel_vcom = 0;
	    return false;
	}
	
        // Parse the VCOM value.
        //
        if ('-' == (char)vcom_str[0])
        {
            // Skip the negative sign (i.e., i = 1, instead of i = 0).
            //
            for( i = 1; i < PNL_SIZE_VCOM; i++ )
            {
                // Skip the dot.
                //
                if ( '.' == (char)vcom_str[i] )
                    continue;

                if ( (vcom_str[i] >= '0') && (vcom_str[i] <= '9') )
                {
                    panel_vcom *= 10;
                    panel_vcom += ((char)vcom_str[i] - '0');
                }
            }
        }
    }

    pr_debug("%s vcom=%d\n", __FUNCTION__, panel_vcom);

    return ( panel_vcom );
}

/**
 * SPI Flash API
 **/

#define SFM_PP               0x02
#define SFM_READ             0x03
#define SFM_WRDI             0x04
#define SFM_RDSR             0x05
#define SFM_WREN             0x06
#define SFM_FAST_READ        0x0B
#define SFM_SE               0xD8
#define SFM_RES              0xAB
#define SFM_ID               0x9F

#define PANEL_FLASH_SIZE       (1024 * 128)

static struct spi_device *panel_flash_spi = NULL;
static panel_flash_select eink_rom_select = panel_flash_waveform;
static bool panel_rom_select_locked = false;
static int panel_flash_size = PANEL_FLASH_SIZE;
static int panel_sfm_size = SFM_SIZE_256K;
static int panel_tst_addr = TST_ADDR_256K;

static panel_flash_select get_flash_select(void)
{
    return ( eink_rom_select );
}

static void set_flash_select(panel_flash_select flash_select)
{
    /* BEN TODO - mutex here */
    if ( !panel_rom_select_locked )
    {
        switch ( flash_select )
        {
            case panel_flash_waveform:
            case panel_flash_commands:
            case panel_flash_test:
                eink_rom_select = flash_select;
            break;
            
            // Prevent the compiler from complaining.
            //
            default:
            break;
        }
    }
}

static unsigned long get_flash_base(void)
{
    unsigned long result;

    switch ( get_flash_select() )
    {
        case panel_flash_waveform:
            result = WFM_ADDR;
        break;

        case panel_flash_commands:
            result = CMD_ADDR;
        break;

        case panel_flash_test:
        default:
            result = panel_tst_addr;
        break;
    }

    return ( result );
}

#define MXC_SPI_MAX_CHARS 28
#define SFM_READ_CMD_LEN 4

static int panel_read_from_flash(unsigned long addr, unsigned char *data, unsigned long size)
{
    struct spi_device *spi = panel_flash_spi;
    unsigned long start = get_flash_base() + addr;
    struct spi_transfer t;
    struct spi_message m;
    u32 len, xfer_len, xmit_len, extra;
    u8 *tx_buf, *rx_buf, *xmit_buf;
    int ret, i;
    u32 *rcv_buf;

    if (spi == NULL) {
        pr_debug("uninitialized!\n");
        return -1;
    }

    pr_debug("start = 0x%lx, size = %ld dest=0x%x\n", start, size, (u32) data);

    if ( panel_sfm_size < (start + size) )
    {
        pr_debug("Attempting to read off the end of flash, start = %ld, length %ld\n",
                start, size);
        return -1;
    }

    // BEN TODO - split into separate spi transfers
    tx_buf = kzalloc(MXC_SPI_MAX_CHARS, GFP_KERNEL);
    if (!tx_buf) {
        pr_debug("Can't alloc spi tx buffer, length %d\n", MXC_SPI_MAX_CHARS);
        return -1;
    }

    // BEN TODO - optimize this
    rx_buf = kzalloc(MXC_SPI_MAX_CHARS, GFP_KERNEL);
    if (!rx_buf) {
        pr_debug("Can't alloc spi rx buffer, length %d\n", MXC_SPI_MAX_CHARS);
        return -1;
    }

    len = size;
    xmit_buf = data;

    while (len > 0) {

    /* BEN TODO: fix hardcoded hack */
    xfer_len = ((len + SFM_READ_CMD_LEN) > MXC_SPI_MAX_CHARS) ? 
        MXC_SPI_MAX_CHARS : (len + SFM_READ_CMD_LEN);

    /* handle small reads */
    if (xfer_len % 4) {
        extra = (4 - (xfer_len % 4));
        xfer_len += extra;
    } else {
        extra = 0;
    }

    spi->mode = SPI_MODE_0;
    spi->bits_per_word = (xfer_len * 8);
    spi_setup(spi);

    /* command is 1 byte, addr is 3 bytes, MSB first */
    tx_buf[3] = SFM_READ;
    tx_buf[2] = (start >> 16) & 0xFF;
    tx_buf[1] = (start >> 8) & 0xFF;
    tx_buf[0] = start & 0xFF;

    memset(&t, sizeof(t), 0);
    t.tx_buf = (const void *) tx_buf;
    t.rx_buf = (void *) rx_buf;
    t.len = xfer_len / 4;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    if (spi_sync(spi, &m) != 0 || m.status != 0) {
        printk(KERN_ERR "err on cmd %d\n", m.status);
        ret = -1;
        break;
    }

    if ((xfer_len - (m.actual_length * 4)) != 0) {
        printk(KERN_ERR "only %d bytes sent\n", (m.actual_length * 4));
        ret = -1;
        break;
    }

    xmit_len = (xfer_len - SFM_READ_CMD_LEN - extra);
    rcv_buf = (u32 *)(rx_buf + SFM_READ_CMD_LEN);

    /* need to byteswap */
    for (i = 0; i < (xmit_len / 4); i++) {
        *((__u32 *) xmit_buf) = __swab32p(rcv_buf);
        xmit_buf += 4;
        rcv_buf++;
    }

    /* handle requests smaller than 4 bytes */
    if (extra) {
        for (i = 0; i < (4 - extra); i++) {
            ((u8 *) xmit_buf)[i] = (rcv_buf[0] >> ((3 - i) * 8)) & 0xFF; 
        }
    }

    start += xmit_len;
    len -= xmit_len;

    ret = 0;
    }

    kfree(tx_buf);
    kfree(rx_buf);
    
    return ret;
}

int panel_program_flash(unsigned long start_addr, unsigned char *buffer, unsigned long blen)
{
    int result = 0;

    pr_debug("%s: not implemented\n", __FUNCTION__);

#if 0 // BEN TODO
    unsigned long flash_base = get_flash_base(),
                  start = flash_base + start_addr;

    if ( panel_sfm_size >= (start + blen) )
    {
        fsl_sfm_start();
        fsl_sfm_write(start, blen, buffer);
        fsl_sfm_end();

        switch ( flash_base )
        {
            // Soft reset after flashing the waveform.
            //
            case WFM_ADDR:
                fsl_sw_init(DONT_BRINGUP_CONTROLLER, DONT_BRINGUP_PANEL);
                if ( !fsl_bootstrap )
                    fsl_cmd_ld_img_upd_data(fx_update_partial, UPD_DATA_RESTORE);
            break;

            // Hard reset after flashing the commands (allow
            // the test area of flash to act like commands
            // when in bootstrap mode).
            //
            case FSL_TST_ADDR_128K:
            case FSL_TST_ADDR_256K:
                if ( !fsl_bootstrap )
                    break;

            case FSL_CMD_ADDR:
                fsl_sw_init(FULL_BRINGUP_CONTROLLER, DONT_BRINGUP_PANEL);
                if ( !fsl_bootstrap )
                    fsl_cmd_ld_img_upd_data(fx_update_partial, UPD_DATA_RESTORE);
            break;
        }

        result = 1;
    }
    else
        einkfb_print_warn("Attempting to write off the end of flash, start = %ld, length %ld\n",
            start, blen);
#endif
    return ( result );
}

/*!
 * This function is called whenever the SPI slave device is detected.
 *
 * @param	spi	the SPI slave device
 *
 * @return 	Returns 0 on SUCCESS and error on FAILURE.
 */
static int panel_flash_probe(struct spi_device *spi)
{
    u32 cmd, res;
    struct spi_transfer t;
    struct spi_message m;
    u8 mfg;

    pr_debug("%s begin\n", __FUNCTION__);

    /* Setup the SPI slave */
    panel_flash_spi = spi;

    spi->mode = SPI_MODE_0;
    spi->bits_per_word = 32;
    spi_setup(spi);

    /* command is 1 byte, addr is 3 bytes */
    cmd = (SFM_ID << 24);
    res = 0;

    memset(&t, sizeof(t), 0);
    t.tx_buf = (const void *) &cmd;
    t.rx_buf = (void *) &res;
    t.len = 1;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    if (spi_sync(spi, &m) != 0 || m.status != 0) {
        printk(KERN_ERR "%s: err on cmd %d\n", __FUNCTION__, m.status);
        return -1;
    }

    // see if we received a valid response
    mfg = (res >> 16) & 0xFF;
    switch (mfg) {
      case 0x20:
        pr_debug("M25P20 flash detected\n");
        break;
      case 0xc2:
        pr_debug("MX25L2005 flash detected\n");
        break;
      case 0xef:
        pr_debug("MX25U4035 flash detected\n");
        break;
        
      case 0x00:
	/* BEN TODO - fix this */
        printk(KERN_ERR "Bad flash signature: 0x%x\n", res);
        panel_flash_spi = NULL;
        return -1;
   
      default:
        pr_debug("Unrecognized flash: 0x%x\n", mfg);
        break;	
    }

    return 0;
}

/*!
 * This function is called whenever the SPI slave device is removed.
 *
 * @param	spi	the SPI slave device
 *
 * @return 	Returns 0 on SUCCESS and error on FAILURE.
 */
static int panel_flash_remove(struct spi_device *spi)
{
    printk(KERN_INFO "Device %s removed\n", dev_name(&spi->dev));
    panel_flash_spi = NULL;

    return 0;
}

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct spi_driver panel_flash_driver = {
	.driver = {
		   .name = "panel_flash_spi",
		   .bus = &spi_bus_type,
		   .owner = THIS_MODULE,
		   },
	.probe = panel_flash_probe,
	.remove = panel_flash_remove,
};

/*
 * SPI Flash API
 */

bool panel_flash_is_readonly(void)
{
    return ( panel_flash_present() );
}

bool panel_supports_flash(void)
{
    return ( panel_flash_present() );
}

static int panel_read_from_flash_byte(unsigned long addr, unsigned char *data)
{
    return ( panel_read_from_flash(addr, data, sizeof(unsigned char)) );
}

static int panel_read_from_flash_short(unsigned long addr, unsigned short *data)
{
    return ( panel_read_from_flash(addr, (unsigned char *)data, sizeof(unsigned short)) );
}

static int panel_read_from_flash_long(unsigned long addr, unsigned long *data)
{
    return ( panel_read_from_flash(addr, (unsigned char *)data, sizeof(unsigned long)) );
}

/*
 * Initialization, Exit, and Presence.
 */

bool panel_flash_present(void)
{
    return ( (NULL != panel_flash_spi) && spi_registered );
}

bool panel_flash_init(void)
{
    int ret = spi_register_driver(&panel_flash_driver);
    
    if (ret) {
	/* BEN TODO - fix logging */
        printk(KERN_ERR "spi driver registration failed: %d\n", ret);
        spi_registered = false;
    }
    else {
        spi_registered = true;
    }
    
    return spi_registered;
}

void panel_flash_exit(void)
{
    if (spi_registered) {
        spi_unregister_driver(&panel_flash_driver);
        spi_registered = false;
    }
}
