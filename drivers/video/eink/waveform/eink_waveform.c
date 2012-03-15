/*
 * eink_waveform.c
 *
 * Copyright (C) 2005-2010 Amazon Technologies, Inc.
 *
 */

#define waveform_version_string_max 64
#define waveform_unknown_char       '?'
#define waveform_seperator          "_"

#define waveform_filename_extension ".wbf"

static char waveform_version_string[waveform_version_string_max];
static char waveform_human_version_string[waveform_version_string_max];

enum run_types
{
    rt_b, rt_t, rt_p, rt_q, rt_a, rt_c, rt_d, rt_e, rt_f, rt_g, rt_h, rt_i, rt_j, rt_k, rt_l, 
    rt_m, rt_n, unknown_run_type, num_run_types,
    
    rt_dummy = -1
};
typedef enum run_types run_types;

enum wf_types
{
    wft_wx, wft_wy, wft_wp, wft_wz, wft_wq, wft_ta, wft_wu, wft_tb, wft_td, wft_wv, wft_wt, wft_te,
    wft_xa, wft_xb, wft_we, wft_wd, wft_xc, wft_ve, wft_xd, wft_xe, wft_xf, wft_wj, wft_wk, wft_wl,
    wft_vj, unknown_wf_type, num_wf_types
};
typedef enum wf_types wf_types;

enum platforms
{
    matrix_2_0_unsupported, matrix_2_1_unsupported, matrix_Vizplex_100, matrix_Vizplex_110,
    matrix_Vizplex_110A, matrix_Vizplex_unknown, matrix_Vizplex_220, matrix_Vizplex_250,
    matrix_Vizplex_220E, unknown_platform, num_platforms
};
typedef enum platforms platforms;

enum panel_sizes
{
    five_oh, six_oh, six_one, six_three, eight_oh, nine_seven, nine_nine, unknown_panel_size,
    num_panel_sizes,
    
    five_inch = 50, six_inch = 60, six_one_inch = 61, six_three_inch = 63, eight_inch = 80,
    nine_seven_inch = 97, nine_nine_inch = 99
};
typedef enum panel_sizes panel_sizes;

enum tuning_biases
{
    standard_bias, increased_ds_blooming_1, increased_ds_blooming_2, improved_temperature_range,
    unknown_tuning_bias, num_tuning_biases
};
typedef enum tuning_biases tuning_biases;

static char *run_type_names[num_run_types] =
{
    "B", "T", "P", "Q", "A", "C", "D", "E", "F", "G", "H", "I",
    "J", "K", "L", "M", "N", "?"
};

static char *wf_type_names[num_wf_types] =
{
    "WX", "WY", "WP", "WZ", "WQ", "TA", "WU", "TB", "TD", "WV",
    "WT", "TE", "??", "??", "WE", "WD", "??", "VE", "??", "??",
    "??", "WJ", "WK", "WL", "VJ", "??"
};

static char *platform_names[num_platforms] =
{
    "????", "????", "V100", "V110", "110A", "????", "V220", "V250",
    "220E", "????"
};

static char *panel_size_names[num_panel_sizes] =
{
    "50", "60", "61", "63", "80", "97", "99", "??"
};

static char *tuning_bias_names[num_tuning_biases] =
{
    "S", "D", "D", "T", "?"
};

#define IS_VIZP(v) ((matrix_Vizplex_110  == (v)) || (matrix_Vizplex_110A == (v)) || \
                    (matrix_Vizplex_220  == (v)) || (matrix_Vizplex_250  == (v)) || \
                    (matrix_Vizplex_220E == (v)))

#define IN_RANGE(n, m, M) (((n) >= (m)) && ((n) <= (M)))

#define has_valid_serial_number(r, s) \
    ((0 != (s)) && (UINT_MAX != (s)) && (rt_t != (r)) && (rt_q != (r)))

#define has_valid_fpl_rate(r) \
    ((EINK_FPL_RATE_50 == (r)) || (EINK_FPL_RATE_60 == (r)) || (EINK_FPL_RATE_85 == (r)))

#define EINK_CHECKSUM(c1, c2) (((c2) << 16) | (c1))

void eink_get_waveform_info(struct eink_waveform_info_t *info)
{
    if ( info )
    {
        unsigned char checksum1, checksum2;

        eink_read_byte(EINK_ADDR_WAVEFORM_VERSION,     &info->waveform.version);
        eink_read_byte(EINK_ADDR_WAVEFORM_SUBVERSION,  &info->waveform.subversion);
        eink_read_byte(EINK_ADDR_WAVEFORM_TYPE,        &info->waveform.type);
        eink_read_byte(EINK_ADDR_RUN_TYPE,             &info->waveform.run_type);
        eink_read_byte(EINK_ADDR_FPL_PLATFORM,         &info->fpl.platform);
        eink_read_byte(EINK_ADDR_FPL_SIZE,             &info->fpl.size);
        eink_read_byte(EINK_ADDR_ADHESIVE_RUN_NUM,     &info->fpl.adhesive_run_number);
        eink_read_byte(EINK_ADDR_MODE_VERSION,         &info->waveform.mode_version);
        eink_read_byte(EINK_ADDR_MFG_CODE,             &info->waveform.mfg_code);
        eink_read_byte(EINK_ADDR_CHECKSUM1,            &checksum1);
        eink_read_byte(EINK_ADDR_CHECKSUM2,            &checksum2);
        eink_read_byte(EINK_ADDR_WAVEFORM_TUNING_BIAS, &info->waveform.tuning_bias);
        eink_read_byte(EINK_ADDR_FPL_RATE,             &info->waveform.fpl_rate);

        eink_read_short(EINK_ADDR_FPL_LOT,             &info->fpl.lot);

        eink_read_long(EINK_ADDR_CHECKSUM,             &info->checksum);
        eink_read_long(EINK_ADDR_FILESIZE,             &info->filesize);
        eink_read_long(EINK_ADDR_SERIAL_NUMBER,        &info->waveform.serial_number);

        if ( 0 == info->filesize ) {
            info->checksum = EINK_CHECKSUM(checksum1, checksum2);
	    info->waveform.parse_wf_hex  = false;
	} else {
	    info->waveform.parse_wf_hex  = false;
	}

        pr_debug(   "\n"
                        " Waveform version:  0x%02X\n"
                        "       subversion:  0x%02X\n"
                        "             type:  0x%02X\n"
                        "         run type:  0x%02X\n"
                        "     mode version:  0x%02X\n"
                        "      tuning bias:  0x%02X\n"
                        "       frame rate:  0x%02X\n"
                        "\n"
                        "     FPL platform:  0x%02X\n"
                        "              lot:  0x%04X\n"
                        "             size:  0x%02X\n"
                        " adhesive run no.:  0x%02X\n"
                        "\n"
                        "        File size:  0x%08lX\n"
                        "         Mfg code:  0x%02X\n"
                        "       Serial no.:  0x%08lX\n"
                        "         Checksum:  0x%08lX\n",

                        info->waveform.version,
                        info->waveform.subversion,
                        info->waveform.type,
                        info->waveform.run_type,
                        info->waveform.mode_version,
                        info->waveform.tuning_bias,
                        info->waveform.fpl_rate,

                        info->fpl.platform,
                        info->fpl.lot,
                        info->fpl.size,
                        info->fpl.adhesive_run_number,

                        info->filesize,
                        info->waveform.mfg_code,
                        info->waveform.serial_number,
                        info->checksum);
    } 
}

char *eink_get_waveform_version_string(eink_waveform_version_string_t which_string)
{
    struct eink_waveform_info_t info;
    
    eink_get_waveform_info(&info);

    // Build up a waveform version string in the following way:
    //
    //      <FPL PLATFORM>_<RUN TYPE>_<FPL LOT NUMBER>_<FPL SIZE>_
    //      <WF TYPE><WF VERSION><WF SUBVERSION>_
    //      <TUNING BIAS>_<MFG CODE>_<S/N>_<FRAME RATE>
    sprintf(waveform_version_string, 
	    "%02x_%02x_%04x_%02x_%02x%02x%02x_%02x_%02x_%08x_%02x",
	    info.fpl.platform,
	    info.waveform.run_type,
	    info.fpl.lot,
	    info.fpl.size,
	    info.waveform.type,
	    info.waveform.version,
	    info.waveform.subversion,
	    info.waveform.tuning_bias,
	    info.waveform.mfg_code,
	    (unsigned int) info.waveform.serial_number,
	    info.waveform.fpl_rate);

    if (which_string == eink_waveform_filename) {
	strcat(waveform_version_string, waveform_filename_extension);
    }

    return waveform_version_string;
}

char *eink_get_waveform_human_version_string(eink_waveform_version_string_t which_string)
{
    char temp_string[waveform_version_string_max];
    struct eink_waveform_info_t info;
    
    panel_sizes panel_size;
    run_types run_type;

    int valid_serial_number,
        valid_fpl_rate;

    eink_get_waveform_info(&info);

    waveform_human_version_string[0] = '\0';
    run_type = info.waveform.run_type;
    panel_size = info.fpl.size;

    pr_debug("%s: %02x_%02x_%04x_%02x_%02x%02x%02x_%02x_%02x_%08x_%02x\n",
	     __FUNCTION__,
	    info.fpl.platform,
	    info.waveform.run_type,
	    info.fpl.lot,
	    info.fpl.size,
	    info.waveform.type,
	    info.waveform.version,
	    info.waveform.subversion,
	    info.waveform.tuning_bias,
	    info.waveform.mfg_code,
	    (unsigned int) info.waveform.serial_number,
	    info.waveform.fpl_rate);

    // Build up a waveform version string in the following way:
    //
    //      <FPL PLATFORM>_<RUN TYPE><FPL LOT NUMBER>_<FPL SIZE>_
    //      <WF TYPE><WF VERSION><WF SUBVERSION>
    //      _<TUNING BIAS> (MFG CODE, S/N XXX, FRAME RATE)|.wbf
    //
    // FPL PLATFORM
    //
    switch ( info.fpl.platform )
    {
        case matrix_Vizplex_100:
        case matrix_Vizplex_110:
        case matrix_Vizplex_110A:
        case matrix_Vizplex_220:
        case matrix_Vizplex_250:
        case matrix_Vizplex_220E:
            strcat(waveform_human_version_string, platform_names[info.fpl.platform]);
        break;

        case unknown_platform:
        default:
            strcat(waveform_human_version_string, platform_names[unknown_platform]);
        break;
    }

    if ( IS_VIZP(info.fpl.platform) )
        strcat(waveform_human_version_string, waveform_seperator);

    // RUN TYPE
    //
    if ( IN_RANGE(run_type, rt_b, rt_n) )
        strcat(waveform_human_version_string, run_type_names[run_type]);
    else
        strcat(waveform_human_version_string, run_type_names[unknown_run_type]);

    // FPL LOT NUMBER
    //
    sprintf(temp_string, "%03d", (info.fpl.lot % 1000));
    strcat(waveform_human_version_string, temp_string);

    // ADHESIVE RUN NUMBER
    //
    if ( !IS_VIZP(info.fpl.platform) )
    {
        sprintf(temp_string, "%02d", (info.fpl.adhesive_run_number % 100));
        strcat(waveform_human_version_string, temp_string);
    }

    strcat(waveform_human_version_string, waveform_seperator);

    // FPL SIZE
    //
    switch ( info.fpl.size )
    {
        case five_inch:
            panel_size = five_oh;
        break;

        case six_inch:
            panel_size = six_oh;
        break;

        case six_one_inch:
            panel_size = six_one;
        break;

        case six_three_inch:
            panel_size = six_three;
        break;

        case eight_inch:
            panel_size = eight_oh;
        break;

        case nine_seven_inch:
            panel_size = nine_seven;
        break;
        
        case nine_nine_inch:
            panel_size = nine_nine;
        break;

        default:
            panel_size = unknown_panel_size;
        break;
    }

    switch ( panel_size )
    {
        case five_oh:
        case six_oh:
        case six_one:
        case six_three:
        case eight_oh:
        case nine_seven:
        case nine_nine:
            strcat(waveform_human_version_string, panel_size_names[panel_size]);
        break;

        case unknown_panel_size:
        default:
            strcat(waveform_human_version_string, panel_size_names[unknown_panel_size]);
        break;
    }
    strcat(waveform_human_version_string, waveform_seperator);

    // WF TYPE
    //
    switch ( info.waveform.type )
    {
        case wft_wx:
        case wft_wy:
        case wft_wp:
        case wft_wz:
        case wft_wq:
        case wft_ta:
        case wft_wu:
        case wft_tb:
        case wft_td:
        case wft_wv:
        case wft_wt:
        case wft_te:
        case wft_we:
        case wft_ve:
        case wft_wj:
        case wft_wk:
        case wft_wl:
        case wft_vj:
            strcat(waveform_human_version_string, wf_type_names[info.waveform.type]);
        break;

        case unknown_wf_type:
        default:
            strcat(waveform_human_version_string, wf_type_names[unknown_wf_type]);
        break;
    }

    // WF VERSION
    //
    if ( info.waveform.parse_wf_hex )
        sprintf(temp_string, "%02X", info.waveform.version);
    else
        sprintf(temp_string, "%02d", (info.waveform.version % 100));

    strcat(waveform_human_version_string, temp_string);

    // WF SUBVERSION
    //
    if ( info.waveform.parse_wf_hex )
        sprintf(temp_string, "%02X", info.waveform.subversion);
    else
        sprintf(temp_string, "%02d", (info.waveform.subversion % 100));

    strcat(waveform_human_version_string, temp_string);

    // Return the parenthetical information, including the TUNING BIAS, only if we're generating
    // the waveform version string itself.
    //
    if ( eink_waveform_version_string == which_string )
    {
        strcat(waveform_human_version_string, waveform_seperator);

        // TUNING BIAS
        //
        switch ( info.waveform.tuning_bias )
        {
            case standard_bias:
            case increased_ds_blooming_1:
            case increased_ds_blooming_2:
            case improved_temperature_range:
                strcat(waveform_human_version_string, tuning_bias_names[info.waveform.tuning_bias]);
            break;
    
            case unknown_tuning_bias:
            default:
                strcat(waveform_human_version_string, tuning_bias_names[unknown_tuning_bias]);
            break;
        }

        // MFG CODE, SERIAL NUMBER, FRAME RATE
        //
        valid_serial_number = has_valid_serial_number(run_type, info.waveform.serial_number);
        valid_fpl_rate = has_valid_fpl_rate(info.waveform.fpl_rate);
        temp_string[0] = '\0';
    
        if ( valid_serial_number && valid_fpl_rate )
            sprintf(temp_string, " (M%02X, S/N %lu, %02XHz)", info.waveform.mfg_code, info.waveform.serial_number,
                info.waveform.fpl_rate);
        
        else if ( valid_fpl_rate )
                sprintf(temp_string, " (M%02X, %02XHz)", info.waveform.mfg_code,
                    info.waveform.fpl_rate);
            
            else if ( valid_serial_number )
                    sprintf(temp_string, " (M%02X, S/N %lu)", info.waveform.mfg_code,
                        info.waveform.serial_number);
    
        if ( strlen(temp_string) )
            strcat(waveform_human_version_string, temp_string);
    }
    
    // Concatenate the normal waveform file name extension if we're being asked to generate
    // a waveform file name as the string.
    //
    if ( eink_waveform_filename == which_string )
        strcat(waveform_human_version_string, waveform_filename_extension);

    // All done.
    //
    return ( waveform_human_version_string );
}

unsigned long eink_get_embedded_waveform_checksum(unsigned char *buffer)
{
    unsigned long checksum = 0;
    
    if ( buffer )
    {
        unsigned long *long_buffer = (unsigned long *)buffer,
                       filesize = long_buffer[EINK_ADDR_FILESIZE >> 2];
        
        if ( filesize )
            checksum = (buffer[EINK_ADDR_CHECKSUM + 0] <<  0) |
                       (buffer[EINK_ADDR_CHECKSUM + 1] <<  8) |
                       (buffer[EINK_ADDR_CHECKSUM + 2] << 16) |
                       (buffer[EINK_ADDR_CHECKSUM + 3] << 24);
        else
            checksum = EINK_CHECKSUM(buffer[EINK_ADDR_CHECKSUM1], buffer[EINK_ADDR_CHECKSUM2]);
    }
    
    return ( checksum );
}

unsigned long eink_get_computed_waveform_checksum(unsigned char *buffer)
{
    unsigned long checksum = 0;
    
    if ( buffer )
    {
        unsigned long *long_buffer = (unsigned long *)buffer,
                       filesize = long_buffer[EINK_ADDR_FILESIZE >> 2];
        
        if ( filesize )
        {
            unsigned long saved_embedded_checksum;
            
            // Save the buffer's embedded checksum and then set it zero.
            //
            saved_embedded_checksum = long_buffer[EINK_ADDR_CHECKSUM >> 2];
            long_buffer[EINK_ADDR_CHECKSUM >> 2] = 0;
            
            // Compute the checkum over the entire buffer, including
            // the zeroed-out embedded checksum area, and then restore
            // the embedded checksum.
            //
            checksum = crc32((unsigned char *)buffer, filesize);
            long_buffer[EINK_ADDR_CHECKSUM >> 2] = saved_embedded_checksum;
        }
        else
        {
            unsigned char checksum1, checksum2;
            int start, length;
    
            // Checksum bytes 0..(EINK_ADDR_CHECKSUM1 - 1).
            //
            start     = 0;
            length    = EINK_ADDR_CHECKSUM1;
            checksum1 = sum8(&buffer[start], length);
            
            // Checksum bytes (EINK_ADDR_CHECKSUM1 + 1)..(EINK_ADDR_CHECKSUM2 - 1).
            //
            start     = EINK_ADDR_CHECKSUM1 + 1;
            length    = EINK_ADDR_CHECKSUM2 - start;
            checksum2 = sum8(&buffer[start], length);
            
            checksum  = EINK_CHECKSUM(checksum1, checksum2);
        }
    }
    
    return ( checksum );
}

bool eink_waveform_valid(void)
{
    char *waveform_version = eink_get_waveform_version_string(eink_waveform_version_string);
    bool result = true;

    if ( strchr(waveform_version, waveform_unknown_char) )
    {
        printk(KERN_ERR "Unrecognized values in waveform header\n");
        pr_debug("waveform_version = %s\n", waveform_version);
        result = false;
    }

    return ( result );
}

int eink_write_waveform_to_file(char *waveform_file_path)
{
    int result = -1;
    
    if ( waveform_file_path )
    {
        mm_segment_t saved_fs = get_fs();
        int waveform_file = -1;
        
        set_fs(get_ds());
        
        waveform_file = sys_open(waveform_file_path, (O_WRONLY | O_TRUNC | O_CREAT), 0);
        
        if ( 0 < waveform_file )
        {
            if ( 0 < sys_write(waveform_file, einkwf_get_buffer(), einkwf_get_buffer_size()) )
                result = 0;
            
            sys_close(waveform_file);
        }
        
        set_fs(saved_fs);
    }
    
    return ( result );
}

int eink_read_waveform_from_file(char *waveform_file_path)
{
    int result = -1, waveform_file = -1;
    
    if ( waveform_file_path )
    {
        mm_segment_t saved_fs = get_fs();
        set_fs(get_ds());
        
        waveform_file = sys_open(waveform_file_path, O_RDONLY, 0);
    
        if ( 0 < waveform_file )
        {
            int len = -1;
            
            len = sys_read(waveform_file, einkwf_get_buffer(), einkwf_get_buffer_size());
    
            if ( 0 < len )
            {
                einkwf_set_buffer_size(len);
                result = 0;
            }
    
            sys_close(waveform_file);
        }
        
        set_fs(saved_fs);        
    }
    
    pr_debug("path = %s, file = %d, size = %d\n",
        (waveform_file_path ? waveform_file_path : EINK_WF_UNKNOWN_PATH),
        waveform_file,
        einkwf_get_buffer_size());
    
    return ( result );
}
