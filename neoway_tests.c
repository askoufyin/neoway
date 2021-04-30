#include "adb.h"

#ifdef __TESTS__

enum {
    NWY_ERROR = -1,
    NWY_OK = 0,
    NWY_NO_DEVICE,          // No device connected
    NWY_TOO_MANY_DEVICES,   // Need to keep only one device to proceed
    NWY_WRONG_DEVICE_TYPE,  // Neither BSN nor BS connected
    NWY_NO_FIRMWARE_LOADED, // No vaild firmware loaded at module
};


/* Neoway test scenario
 */
int 
nwy_test_device(void)
{
    const char* res;

    /* Check device presence 
     */
    res = adb_shell("devices", NULL);
    if (NULL == res) {
        return -1;
    }

    return 0; // Found
}


int 
nwy_test_device_type(void)
{
    res = adb_shell("uname", "-n");
    if (NULL == res) {
        return NWY_ERROR;
    }

    if (0 == strncmp(res, "mdm", 3)) {
        /* Generic Neoway */
        return NWY_NO_FIRMWARE_LOADED;
    }

    if (0 == strcmp(res, "bsn-ksc") || 0 == strcmp(res, "bs-ksc") {
        /* Valid device */
        return NWY_OK;
    }

    return NWY_WRONG_DEVICE_TYPE;
}


int 
nwy_test_device_fw(void)
{
    return 0;
}

#endif

