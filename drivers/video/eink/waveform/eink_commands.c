/*
 * eink_commands.c
 *
 * Copyright (C) 2005-2010 Amazon Technologies
 *
 */

#define COMMANDS_VERSION_STRING_MAX     32
#define COMMANDS_VERSION_UNKNOWN_CHAR   '?'
#define COMMANDS_VERSION_SEPERATOR      "_"

#define COMMANDS_TYPE_0             0x0000
#define COMMANDS_TYPE_1             0x6273

enum commands_types
{
    commands_type_0, commands_type_1, commands_type_unknown, num_commands_types
};
typedef enum commands_types commands_types;

static char commands_version_string[COMMANDS_VERSION_STRING_MAX];

static char *commands_types_names[num_commands_types] =
{
    "T", "P", "?"
};

void eink_get_commands_info(eink_commands_info_t *info)
{
    if ( info )
    {
        if ( IS_BROADSHEET() )
        {
            unsigned short type;
            
            eink_read_byte(EINK_ADDR_COMMANDS_VERS_MAJOR,  &info->vers_major);
            eink_read_byte(EINK_ADDR_COMMANDS_VERS_MINOR,  &info->vers_minor);
            eink_read_short(EINK_ADDR_COMMANDS_TYPE,       &type);
            eink_read_long(EINK_ADDR_COMMANDS_CHECKSUM,    &info->checksum);
        
            info->type  = ((type & 0xFF00) >> 8) | ((type & 0x00FF) << 8);
            info->which = EINK_COMMANDS_BROADSHEET;
        }
        else
        {
            unsigned short code_size;
            unsigned long  version;
            int file_size;
            
            eink_read_long(EINK_ADDR_COMMANDS_VERSION,     &version);
            eink_read_short(EINK_ADDR_COMMANDS_CODE_SIZE,  &code_size);
            
            version     = ((version & 0xFF000000) >> 24) |
                          ((version & 0x00FF0000) >>  8) |
                          ((version & 0x0000FF00) <<  8) |
                          ((version & 0x000000FF) << 24);

            file_size   = ((code_size + 1) << 1) + EINK_COMMANDS_FIXED_CODE_SIZE;

            eink_read_long((file_size - 4),                &info->checksum);
            
            info->which = EINK_COMMANDS_ISIS;
            info->version = version;
        }
    }
}

char *eink_get_commands_version_string(void)
{
    char temp_string[COMMANDS_VERSION_STRING_MAX];
    eink_commands_info_t info;
    commands_types commands_type;
    
    eink_get_commands_info(&info);
    commands_version_string[0] = '\0';

    // Build a commands version string of the form:
    //
    //  Broadsheet:
    //
    //      <TYPE>_<VERS_MAJOR.VERS_MINOR> (C/S = <CHECKSUM>).
    //
    //  ISIS:
    //
    //      V<VERSION> (C/S = <CHECKSUM>).
    //
    
    if ( EINK_COMMANDS_BROADSHEET == info.which )
    {
        // TYPE
        //
        switch ( info.type )
        {
            case COMMANDS_TYPE_0:
                commands_type = commands_type_0;
            break;
            
            case COMMANDS_TYPE_1:
                commands_type = commands_type_1;
            break;
            
            default:
                commands_type = commands_type_unknown;
            break;
        }
        strcat(commands_version_string, commands_types_names[commands_type]);
        strcat(commands_version_string, COMMANDS_VERSION_SEPERATOR);
        
        // VERSION
        //
        sprintf(temp_string, "%02X.%02X", info.vers_major, info.vers_minor);
        strcat(commands_version_string, temp_string);
    }
    else
    {
        // VERSION
        //
        sprintf(temp_string, "V%04lX", (info.version % 0x10000));
        strcat(commands_version_string, temp_string);
    }
    
    // CHECKSUM
    //
    if ( info.checksum )
    {
        sprintf(temp_string, " (C/S %08lX)", info.checksum);
        strcat(commands_version_string, temp_string);
    }
    
    return ( commands_version_string );
}

unsigned long eink_get_embedded_commands_checksum(unsigned char *buffer)
{
    unsigned long checksum = 0;
    
    if ( buffer )
    {
        if ( IS_BROADSHEET() )
        {
            checksum = (buffer[EINK_ADDR_COMMANDS_CHECKSUM + 0] <<  0) |
                       (buffer[EINK_ADDR_COMMANDS_CHECKSUM + 1] <<  8) |
                       (buffer[EINK_ADDR_COMMANDS_CHECKSUM + 2] << 16) |
                       (buffer[EINK_ADDR_COMMANDS_CHECKSUM + 3] << 24);
        }
        else
        {
            unsigned short *short_buffer = (unsigned short *)buffer,
                            code_size = short_buffer[EINK_ADDR_COMMANDS_CODE_SIZE >> 1];
            int file_size = ((code_size + 1) << 1) + EINK_COMMANDS_FIXED_CODE_SIZE;
            
            checksum = (buffer[file_size - 4] <<  0) |
                       (buffer[file_size - 3] <<  8) |
                       (buffer[file_size - 2] << 16) |
                       (buffer[file_size - 1] << 24);
         }
    }
        
    return ( checksum );
}

unsigned long eink_get_computed_commands_checksum(unsigned char *buffer)
{
    unsigned long checksum = 0;
    
    if ( buffer )
    {
        if ( IS_BROADSHEET() )
            checksum = crc32((unsigned char *)buffer, (EINK_COMMANDS_FILESIZE - 4));
        else
        {
            unsigned short *short_buffer = (unsigned short *)buffer,
                            code_size = short_buffer[EINK_ADDR_COMMANDS_CODE_SIZE >> 1];
            int file_size = ((code_size + 1) << 1) + EINK_COMMANDS_FIXED_CODE_SIZE,
                start     = EINK_ADDR_COMMANDS_CODE_SIZE,
                length    = (file_size - 4) - EINK_ADDR_COMMANDS_CODE_SIZE;
            
            checksum = sum32(&buffer[start], length);
        }
    }
    
    return ( checksum );
}

bool eink_commands_valid(void)
{
    char *commands_version = eink_get_commands_version_string();
    bool result = true;
    
    if ( strchr(commands_version, COMMANDS_VERSION_UNKNOWN_CHAR) )
    {
        printk(KERN_ERR "Unrecognized values in commands header\n");
        result = false;
    }
        
    return ( result );
}
