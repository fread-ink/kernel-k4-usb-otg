/*
 * eink_waveform.h
 *
 * Copyright (C) 2005-2010 Amazon Technologies, Inc.
 *
 */

#ifndef _EINK_WAVEFORM_H
#define _EINK_WAVEFORM_H

#include <linux/einkwf.h>

#define EINK_WAVEFORM_FILESIZE          262144  // 256K..
#define EINK_WAVEFORM_COUNT             0       // ..max

#define EINK_WAVEFORM_PROXY_SIZE        (1024 * 1024 * 2)

#define EINK_ADDR_CHECKSUM1             0x001F  // 1 byte  (checksum of bytes 0x00-0x1E)
#define EINK_ADDR_CHECKSUM2             0x002F  // 1 byte  (checksum of bytes 0x20-0x2E)
#define EINK_ADDR_CHECKSUM              0x0000  // 4 bytes (was EINK_ADDR_MFG_DATA_DEVICE)
#define EINK_ADDR_FILESIZE              0x0004  // 4 bytes (was EINK_ADDR_MFG_DATA_DISPLAY)
#define EINK_ADDR_MFG_CODE              0x0015  // 1 byte  (0x00..0xFF -> M00..MFF)
#define EINK_ADDR_SERIAL_NUMBER         0x0008  // 4 bytes (little-endian)

#define EINK_ADDR_RUN_TYPE              0x000C  // 1 byte  (0x00=[B]aseline, 0x01=[T]est/trial, 0x02=[P]roduction, 0x03=[Q]ualification, 0x04=V110[A],
                                                //          0x05=V220[C], 0x06=D, 0x07=V220[E], 0x08-0x10=F-N)

#define EINK_ADDR_FPL_PLATFORM          0x000D  // 1 byte  (0x00=2.0, 0x01=2.1, 0x02=2.3; 0x03=V110, 0x04=V110A, 0x06=V220, 0x07=V250, 0x08=V220E)
#define EINK_ADDR_FPL_SIZE              0x0014  // 1 byte  (0x32=5", 0x3C=6", 0x3F=6" 0x50=8", 0x61=9.7", 0x63=9.7")
#define EINK_ADDR_FPL_LOT               0x000E  // 2 bytes (little-endian)
#define EINK_ADDR_ADHESIVE_RUN_NUM      0x0010  // 1 byte  (mode version when EINK_ADDR_FPL_PLATFORM is 0x03 or later)
#define EINK_ADDR_MODE_VERSION          0x0010  // 1 byte  (0x00=MU/GU/GC/PU, 0x01,0x02=DU/GC16/GC4, 0x03=DU/GC16/GC4/AU, 0x04=DU/GC16/AU)

#define EINK_ADDR_WAVEFORM_VERSION      0x0011  // 1 byte  (BCD)
#define EINK_ADDR_WAVEFORM_SUBVERSION   0x0012  // 1 byte  (BCD)
#define EINK_ADDR_WAVEFORM_TYPE         0x0013  // 1 byte  (0x0B=TE, 0x0E=WE, 0x15=WJ, 0x16=WK, 0x17=WL, 0x18=VJ)
#define EINK_ADDR_WAVEFORM_TUNING_BIAS  0x0016  // 1 byte  (0x00=Standard, 0x01=Increased DS Blooming V110/V110E, 0x02=Increased DS Blooming V220/V220E,
                                                //          0x03=0x02=Increased DS Blooming V220/V220E & Improved Temperature Range V220/V220E)

#define EINK_ADDR_FPL_RATE              0x00017 // 1 byte  (0x50=50Hz, 0x60=60Hz, 0x85=85Hz)

// PVI/EIH panel values
#define EINK_MFG_CODE_EIH_E40           0x33    // ED060SCF (V220 6” Tequila)
#define EINK_MFG_CODE_EIH_E48           0x34    // ED060SCFH1 (V220 Tequila Hydis – Line 2)
#define EINK_MFG_CODE_EIH_E49           0x35    // ED060SCFH1 (V220 Tequila Hydis – Line 3)
#define EINK_MFG_CODE_EIH_E4A           0x36    // ED060SCFC1 (V220 Tequila CMO)
#define EINK_MFG_CODE_EIH_E4B           0x37    // ED060SCFT1 (V220 Tequila CPT)

// LGD panel values
#define EINK_MFG_CODE_LGD_L00           0xA0
#define EINK_MFG_CODE_LGD_L01           0xA1
#define EINK_MFG_CODE_LGD_L02           0xA2
#define EINK_MFG_CODE_LGD_L03           0xA3    // LB060S03-RD02 (LGD Tequila Line 1)
#define EINK_MFG_CODE_LGD_L04           0xA4    // Reserved for 2nd LGD Tequila Line
#define EINK_MFG_CODE_LGD_L07           0xA7
#define EINK_MFG_CODE_LGD_L08           0xA8
#define EINK_WAVEFORM_TYPE_LGD_VJ       0x18

#define EINK_FPL_SIZE_60                0x3C    // 6.0-inch panel,  800 x  600
#define EINK_FPL_SIZE_63                0x3F    // 6.0-inch panel,  800 x  600
#define EINK_FPL_SIZE_61                0x3D    // 6.1-inch panel, 1024 x  768
#define EINK_FPL_SIZE_97                0x61    // 9.7-inch panel, 1200 x  825
#define EINK_FPL_SIZE_99                0x63    // 9.7-inch panel, 1600 x 1200

#define EINK_FPL_RATE_50                0x50    // 50Hz waveform
#define EINK_FPL_RATE_60                0x60    // 60Hz waveform
#define EINK_FPL_RATE_85                0x85    // 85Hz waveform

#define EINK_IMPROVED_TEMP_RANGE        0x03    // Don't clip the temperature if we see this value in EINK_ADDR_WAVEFORM_TUNING_BIAS.

extern void eink_get_waveform_info(eink_waveform_info_t *info);
extern char *eink_get_waveform_version_string(eink_waveform_version_string_t which_string);
extern char *eink_get_waveform_human_version_string(eink_waveform_version_string_t which_string);
extern bool eink_waveform_valid(void);

extern int  eink_write_waveform_to_file(char *waveform_file_path);
extern int  eink_read_waveform_from_file(char *waveform_file_path);

extern unsigned long eink_get_embedded_waveform_checksum(unsigned char *buffer);
extern unsigned long eink_get_computed_waveform_checksum(unsigned char *buffer);

#endif // _EINK_WAVEFORM_H

