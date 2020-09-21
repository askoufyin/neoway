#include <netdb.h>
#include <linux/limits.h>

#include "data.h"

#include <nwy_common.h>
#include <nwy_data.h>
#include <nwy_gpio.h>
#include <nwy_error.h>


static char _POST_CONNECT[] = { "/data/post-connect.sh" };


/* Static globals. This is compelled move since we can't pass a pointer to the
 * user data to the callback function
 */
static nwy_data_start_call_v02_t callp;
static int handle;
static int retry_interval = 30; // secs


int
data_thread_init(options_t *opts)
{
    /* For now, there is no initialization. Reserved for future use */
    return 0;
}


static void
data_call_state_cb(int hndl, nwy_data_call_state_t ind_state, void *ind_struct)
{
    options_t *opts = (options_t *)ind_struct;
    char dev_name[16];
    char cmd[PATH_MAX];
    nwy_data_addr_t_info info;
    int res, valid_ip_cnt;
    char ip_str[16], gw_str[16], pdns[16], sdns[16];

    switch(ind_state) {
        case NWY_DATA_CALL_CONNECTED:
            nwy_gpio_set_val(NWY_GPIO_78, NWY_HIGH); // GSM LED on

            res = nwy_data_get_device_name(hndl, dev_name, sizeof(dev_name));
            if (NWY_RES_OK != res) {
                printf("Get device name failed. Error=%d\n", res);
                return;
            }

            res = nwy_data_get_ip_addr(hndl, &info, &valid_ip_cnt);
            if(0 == info.iface_addr_s.valid_addr){
                printf("Get IP address failed. Error=%d\n", res);
                return;
            } //

            inet_ntop(AF_INET, info.iface_addr_s.addr.__ss_padding, ip_str, sizeof(ip_str));
            inet_ntop(AF_INET, info.gtwy_addr_s.addr.__ss_padding, gw_str, sizeof(gw_str));
            inet_ntop(AF_INET, info.dnsp_addr_s.addr.__ss_padding, pdns, sizeof(pdns));
            inet_ntop(AF_INET, info.dnsp_addr_s.addr.__ss_padding, sdns, sizeof(sdns));

            printf("Connected, device name is \"%s\"\n", dev_name);
            printf("IP %s, Gateway %s, Primary DNS %s, Secondary DNS %s\n", ip_str, gw_str, pdns, sdns);

            sprintf(cmd, "%s %s %s %s %s %s", _POST_CONNECT, dev_name, ip_str, gw_str, pdns, sdns);
            system(cmd);
            break;

        case NWY_DATA_CALL_DISCONNECTED:
            nwy_gpio_set_val(NWY_GPIO_78, NWY_LOW); // GSM LED off

            printf("Disonnected. Retry in %d secs.\n", retry_interval);
            sleep(retry_interval);

            printf("Initiating connection...\n");
            res = nwy_data_start_call(handle, &callp);
            if(res < 0) {
                printf("Call failed! Error=%d\n", res);
                return;
            }
            break;

        case NWY_DATA_CALL_INVALID:
            break;
    }
}


void
establish_data_connection(options_t *opts)
{
    nwy_data_profile_info_t info;
    struct hostent *hent;
    char apn_ip[20];
    int res;


    printf("GPRS: Init\n");

    /* Resolve APN name */
    hent = gethostbyname(opts->gprs_apn);
    if(NULL == hent) {
        printf("Cannot resolve host name %s\n", opts->gprs_apn);
        return;
    }

    inet_ntop(AF_INET, hent->h_addr_list[0], apn_ip, sizeof(apn_ip));
    printf("%s is %s\n", opts->gprs_apn, apn_ip);

    handle = nwy_data_get_srv_handle(data_call_state_cb);

    /* Setup profile */

    info.pdp_type = NWY_DATA_PDP_TYPE_IPV4;         // PPP dial or IPV4
    strcpy(info.apn, apn_ip);                       // Access point name
    info.auth_proto = NWY_DATA_AUTH_PROTO_PAP_CHAP; // Authorization protocol
    strcpy(info.user_name, opts->gprs_user);
    strcpy(info.pwd, opts->gprs_password);

    res = nwy_data_set_profile(1, NWY_DATA_PROFILE_3GPP, &info); // 3GPP=UMTS, 3GPP2=CDMA
    if(res < 0) {
        printf("Cannot set profile. Error=%d\n", res);
        return;
    }

    callp.auto_connect = TRUE;
    callp.ip_version = 4; // 4=IPV4, 6=IPV6, 10=IPV4+IPV6.
    callp.profile_idx = 1;

#if 0
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