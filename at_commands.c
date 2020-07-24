#include <stdlib.h>
#include "at_commands.h"


static at_msg_t _at_msgs[] = {
    { AT_GET_RECORD, "#GPSGR" },
    { AT_PEEK_RECORD, "#GPSPR" },
    { AT_GET_RECORD_COUNT, "#GPSGRC" },
    { AT_CLEAR_BUFFER, "#GPSCLRB" },
/* Append your commands here */
    { AT_UNKNOWN, NULL } // EOF
};



atmsg_id
at_which_command(const char *cmd)
{
    return AT_UNKNOWN;
}
