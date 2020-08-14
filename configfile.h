#ifndef __CONFIGFILE__
#define __CONFIGFILE__


#include "utils.h"


typedef enum _opttype {
    TYPE_STRING,        // strdup() used
    TYPE_CHAR_ARRAY,    // strcpy() used
    TYPE_INT,
    TYPE_FLOAT,
    TYPE_BOOL,
    TYPE_IPADDR,
} opttype_t;


typedef struct _confoption {
    char *keyword;      // Option keyword
    opttype_t type;     // Argument type
    void *argptr;       // Where to store result
} confoption_t;


#ifdef __cplusplus
extern "C" {
#endif

extern int load_config(const char *, options_t *);

#ifdef __cplusplus
}
#endif


#endif
