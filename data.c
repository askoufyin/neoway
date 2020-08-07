#include <netdb.h>

#include "data.h"

#include <nwy_common.h>
#include <nwy_data.h>
#include <nwy_error.h>


static char _APN[] = { "217.118.87.98" };
static char _USER_NAME[] = { "beeline" };
static char _PASSWORD[] = { "beeline" };
static char _POST_CONNECT[] = { "post-connect.sh" };


int 
data_thread_init(options_t *opts)
{
    /* For now, there is no initialization. Reserved for future use */
    return 0;
}


static void
data_call_state_cb(int hndl, nwy_data_call_state_t ind_state, void *ind_struct)
{
    switch(ind_state) {
        case NWY_DATA_CALL_CONNECTED:
            printf("Connected\n");
            system(_POST_CONNECT);
            break;
        case NWY_DATA_CALL_DISCONNECTED:
            printf("Disonnected\n");
            break;
        case NWY_DATA_CALL_INVALID:
            break;
    }
}


void
establish_data_connection(options_t *opts)
{
    nwy_data_start_call_v02_t callp;
    nwy_data_profile_info_t info;
    struct hostent *hent;
    char apn_ip[20];
    int res, handle;

    
    printf("Establishing connection...\n");

    /* Resolve APN name */
    hent = gethostbyname(_APN);
    if(NULL == hent) {
        printf("Cannot resolve host name %s\n", _APN);
        return;
    }

    inet_ntop(AF_INET, hent->h_addr_list[0], apn_ip, sizeof(apn_ip));
    printf("%s is %s\n", _APN, apn_ip);

    handle = nwy_data_get_srv_handle(data_call_state_cb);

    /* Setup profile */
    
    info.pdp_type = NWY_DATA_PDP_TYPE_PPP;          // PPP dial or IPV4
    strcpy(info.apn, apn_ip);                       // Access point name
    info.auth_proto = NWY_DATA_AUTH_PROTO_PAP_CHAP; // Authorization protocol
    strcpy(info.user_name, _USER_NAME);
    strcpy(info.pwd, _PASSWORD);

    res = nwy_data_set_profile(1, NWY_DATA_PROFILE_3GPP, &info); // 3GPP=UMTS, 3GPP2=CDMA
    if(res < 0) {
        printf("Cannot set profile. Error=%d\n", res);
        return;
    }

    callp.auto_connect = TRUE;
    callp.ip_version = 4; // 4=IPV4, 6=IPV6, 10=IPV4+IPV6.
    callp.profile_idx = 1;

#if 1
    /* Initiate call */
    if(!nwy_data_regs_ready()){
        printf("Data service not ready!\n");
        return;
    }
#endif
    printf("Initiating connection...\n");
    res = nwy_data_start_call(handle, &callp);
    if(res < 0) {
        printf("Call failed! Error=%d\n", res);
        return;
    }
}