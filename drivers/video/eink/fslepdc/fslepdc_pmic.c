/*
 *  linux/drivers/video/eink/fslepdc/fslepdc_pmic.c --
 *  eInk frame buffer device HAL PMIC hardware access
 *
 *      Copyright (c) 2010-2011 Amazon Technologies, Inc.
 *
 *  This file is subject to the terms and conditions of the GNU General Public
 *  License. See the file COPYING in the main directory of this archive for
 *  more details.
 */

#include "../hal/einkfb_hal.h"
#include "fslepdc.h"

#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>

#if PRAGMAS
    #pragma mark Definitions/Globals
#endif

#define PMIC_TEMP_MIN       -10     // -10C
#define PMIC_TEMP_MAX        85     //  85C
#define PMIC_TEMP_DEFAULT    25     //  25C

#define PMIC_VCOM_MIN         0     //  0.00V
#define PMIC_VCOM_MAX       275000
#define PMIC_VCOM_DEFAULT   186000  /* PAPYRUS_TO_VCOM(186000/1000) -> -2V */

#define PMIC_DISP_REGULATOR "DISPLAY"
#define PMIC_VCOM_REGULATOR "VCOM"
#define PMIC_TEMP_REGULATOR "TMST"

#define mV_to_uV(mV)        ((mV) * 1000)
#define uV_to_mV(uV)        ((uV) / 1000)
#define V_to_uV(V)          (mV_to_uV((V) * 1000))
#define uV_to_V(uV)         (uV_to_mV(uV) / 1000)

static int pmic_temp = PMIC_TEMP_DEFAULT;

struct regulator *pmic_disp_regulator = NULL;
struct regulator *pmic_vcom_regulator = NULL;
struct regulator *pmic_temp_regulator = NULL;

static bool pmic_present = false;

static bool pmic_enable_regulator(void)
{
    bool disable_regulator = true;
    
    // Attempt to enable the display regulator first and,...
    //
    if ( 0 != regulator_enable(pmic_disp_regulator) )
    {
        einkfb_print_warn("unable to enable display regulator\n");
        disable_regulator = false;
    }
    else
    {
        // ...if successful, enable the VCOM regulator.
        //
        if ( 0 != regulator_enable(pmic_vcom_regulator) )
        {
            einkfb_print_warn("unable to enable VCOM regulator\n");
            
            regulator_disable(pmic_disp_regulator);
            disable_regulator = false;
        }
     }
     
    return ( disable_regulator );
}

static void pmic_disable_regulator(void)
{
    regulator_disable(pmic_vcom_regulator);
    regulator_disable(pmic_disp_regulator);
}

// static int pmic_get_temp(void)
// {
//     // Enable the regulator if necessary and, if so, remember to disable it.
//     //
//     bool disable_regulator = pmic_enable_regulator();
//     
//     // Note:  We're overloading the get-voltage operation here since we're using the regulator
//     //        interface but there is no temperature-getting operation, per se.
//     //
//     int temp = regulator_get_voltage(pmic_temp_regulator);
//     
//     // Only disable the regulator if we need to.
//     //
//     if ( disable_regulator )
//         pmic_disable_regulator();
// 
//     return ( temp );
// }

static int pmic_get_temp(void)
{
    // The papyrus-regulator periodically reads the temperature and stores it in a
    // global.  We just return that.
    //
    return ( papyrus_temp );
}

static void pmic_set_vcom(int vcom)
{
    // Enable the regulator if necessary and, if so, remember to disable it.
    //
    bool disable_regulator = pmic_enable_regulator();

    // Note:  The set-voltage operation takes a min and max value in uV.  But we really
    //        only use the max value to set VCOM with.
    //
    if ( 0 != regulator_set_voltage(pmic_vcom_regulator, 0, mV_to_uV(vcom) ) )
    {
        einkfb_print_warn("unable to set VCOM = %d\n", vcom);
    }
    else
    {
        einkfb_debug("VCOM = %d\n", uV_to_mV(regulator_get_voltage(pmic_vcom_regulator)));
    }
    
    // Only disable the regulator if we need to.
    //
    if ( disable_regulator )
        pmic_disable_regulator();
}

#if PRAGMAS
    #pragma mark -
    #pragma mark FSL EPDC PMIC API
    #pragma mark -
#endif

bool fslepdc_pmic_present(void)
{
    return ( pmic_present );
}

bool fslepdc_pmic_init(void)
{
    if ( !pmic_present )
    {
        pmic_disp_regulator = regulator_get(NULL, PMIC_DISP_REGULATOR);
        pmic_vcom_regulator = regulator_get(NULL, PMIC_VCOM_REGULATOR);
        pmic_temp_regulator = regulator_get(NULL, PMIC_TEMP_REGULATOR);
        
        if ( IS_ERR(pmic_disp_regulator) || IS_ERR(pmic_vcom_regulator) || IS_ERR(pmic_temp_regulator) )
        {
            regulator_put(pmic_disp_regulator); pmic_disp_regulator = NULL;
            regulator_put(pmic_vcom_regulator); pmic_vcom_regulator = NULL;
            regulator_put(pmic_temp_regulator); pmic_temp_regulator = NULL;
            
            einkfb_print_warn("FSL EPDC PMIC access not available\n");
        }
        else
            pmic_present = true;
    }
        
    return ( pmic_present );
}

void fslepdc_pmic_done(void)
{
    if ( pmic_present )
    {
        regulator_put(pmic_disp_regulator);
        regulator_put(pmic_vcom_regulator);
        regulator_put(pmic_temp_regulator);
        
        pmic_present = false;
    }
}

int fslepdc_pmic_get_temperature(void)
{
    int temp = PMIC_TEMP_DEFAULT;
    
    if ( pmic_present )
    {
        pmic_temp = pmic_get_temp();

        if ( IN_RANGE(pmic_temp, PMIC_TEMP_MIN, PMIC_TEMP_MAX) )
            temp = pmic_temp;
        else
            einkfb_print_warn("temp = %dC out of range, using default = %dC\n",
                pmic_temp, temp);
    }

    return ( temp );
}

int fslepdc_pmic_get_vcom_default(void)
{
    // Return regardless of whether the PMIC is present or not.
    //
    return ( PMIC_VCOM_DEFAULT );
}

int fslepdc_pmic_set_vcom(int vcom)
{
    int result = PMIC_VCOM_DEFAULT;
    
    if ( pmic_present )
    {
        if ( IN_RANGE(vcom, PMIC_VCOM_MIN, PMIC_VCOM_MAX) )
            result = vcom;

        pmic_set_vcom(result);
    }
    
    return ( result );
}
