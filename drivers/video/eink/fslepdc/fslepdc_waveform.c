/*
 *  linux/drivers/video/eink/fslepdc/fslepdc_waveform.c --
 *  eInk frame buffer device HAL fslepdc waveform code
 *
 *      Copyright (c) 2005-2011 Amazon Technologies
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

static int  override_upd_mode = WF_UPD_MODE_INIT;

void fslepdc_set_override_upd_mode(int upd_mode)
{
    switch ( upd_mode )
    {
        // Only override with valid upd_modes.
        //
        case WF_UPD_MODE_MU:
        case WF_UPD_MODE_GU:
        case WF_UPD_MODE_GC:
        case WF_UPD_MODE_PU:
            override_upd_mode = upd_mode;
        break;
        
        // Default to not overriding if the value
        // is invalid.
        //
        default:
            override_upd_mode = WF_UPD_MODE_INIT;
        break;
    }
}

int fslepdc_get_override_upd_mode(void)
{
    return ( override_upd_mode );
}

int fslepdc_get_waveform_mode(int upd_mode)
{
    int result = WF_UPD_MODE_INVALID;
    
    // If we have an override in place...
    //
    if ( WF_UPD_MODE_INIT != override_upd_mode )
    {
        // ...use that.
        //
        result = einkwf_panel_get_waveform_mode(override_upd_mode);
    }
    else
    {
	result = einkwf_panel_get_waveform_mode(upd_mode);
    }

    return ( result ); 
}

void fsledpc_set_waveform_modes(void)
{
    struct mxcfb_waveform_modes waveform_modes;

    einkwf_panel_set_update_modes();
    
    // Tell the EPDC what the waveform update modes are, based on
    // what we've read back from the waveform itself.  These are
    // what is used when we tell the EPDC to use WF_UPD_MODE_AUTO. 
    //
    waveform_modes.mode_init = einkwf_panel_get_waveform_mode(WF_UPD_MODE_INIT);
    waveform_modes.mode_du   = einkwf_panel_get_waveform_mode(WF_UPD_MODE_MU);
    waveform_modes.mode_gc4  = einkwf_panel_get_waveform_mode(WF_UPD_MODE_GU);
    waveform_modes.mode_gc8  = einkwf_panel_get_waveform_mode(WF_UPD_MODE_GC);
    waveform_modes.mode_gc16 = einkwf_panel_get_waveform_mode(WF_UPD_MODE_GC);
    waveform_modes.mode_gc32 = einkwf_panel_get_waveform_mode(WF_UPD_MODE_GC);
    waveform_modes.mode_gl16 = einkwf_panel_get_waveform_mode(WF_UPD_MODE_GL);
    
    mxc_epdc_fb_set_waveform_modes(&waveform_modes, NULL);
}

#define FSL_PNL_SIZE_VCOM_STR 6

static char fsl_vcom_override_str[FSL_PNL_SIZE_VCOM_STR] = { 0 };
static int  fsl_vcom_override = 0;

#define FSL_VCOM_INT_TO_STR(i, s)   \
    sprintf((s), "-%1d.%02d", ((i)/100)%10, (i)%100)

void fslepdc_override_vcom(int vcom)
{
    if ( einkwf_panel_supports_vcom() )
    {
        // Allow us to unset an override VCOM by passing in 0.
        //
        if ( vcom )
        {
            fsl_vcom_override = fslepdc_pmic_set_vcom(vcom);
            FSL_VCOM_INT_TO_STR(fsl_vcom_override,
				fsl_vcom_override_str);
        }
        else
        {
            fslepdc_pmic_set_vcom(einkwf_panel_get_vcom());
            fsl_vcom_override = 0;
        }
    }
}

char *fslepdc_get_vcom_str(void) 
{
    if (fsl_vcom_override) {
	return fsl_vcom_override_str;
    }
    
    return einkwf_panel_get_vcom_str();
}
