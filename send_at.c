#include "thread_funcs.h"
#include "uart.h"
#include "send_at.h"
#include "stdio.h"
#include <strings.h>
#include <sys/select.h>

//#include "nwy_loc.h"
//#include "nwy_uart.h"
//#include "nwy_error.h"
//#include "nwy_common.h"
//#include "nwy_sim.h"
//#define NWY_AT_PORT      "/dev/smd9"
int lat_D, lon_D;
float lat_M = 0, lon_M = 0;
int hh, mm, ss, ms;

#define WEBPOSTGETINCONSOLE  //убрать если отвлекает вывод пост гет запросов в консоли

void At_init(void* web_opts)
 {
     options_t* opts = (options_t*)web_opts;
     //int ret = nwy_at_port_init(NWY_AT_PORT);
     //if (ret != 0)
     //{
     //     printf("Init port error %s\n", NWY_AT_PORT);
     //}
     //else
     //{
         send_at_cmd("AT$MYGPSPWR=1\0", opts);
         printf("AT: OK\n");
     //}
 }

 void send_at_cmd(char* at_com, void* web_opts)
 {
     options_t* opts = (options_t*)web_opts;
     char* p_resp = NULL;
     char* p_result = NULL;
     int i = 0, j = 0;
             printf("wait mutex Sanya!\n");
     pthread_mutex_lock(&opts->mutex_modem);
     int ret = nwy_at_send_cmd(at_com, &p_resp, &p_result);
     if (ret != 0)
     {
         #ifdef WEBPOSTGETINCONSOLE
         printf("Send at cmd %s: Fail\n", at_com);
         #endif
         return;
     }
     else
     {
         #ifdef WEBPOSTGETINCONSOLE
         printf("Send at cmd %s: OK\n", at_com);
         #endif
     }
     if (p_resp != NULL)
     {
         #ifdef WEBPOSTGETINCONSOLE
         printf("Recv at response:\n%s\n%s\n", p_resp, p_result);
         #endif

         if (!strcmp(at_com, "AT+CSQ\0") && p_resp[0] == '+' && p_resp[1] == 'C' && p_resp[2] == 'S' && p_resp[3] == 'Q')
         {
             while (p_resp[i] != ' ')
             {
                 i++;
             }
             i++;
             while (p_resp[i] != ',')
             {
                 opts->rssi[j] = p_resp[i];
                 i++; j++;
             }
             opts->rssi[j] = '\0';
             int rssi_val = -113 + atoi(opts->rssi) * 2;
             snprintf(opts->rssi, sizeof(opts->rssi), "%d", rssi_val);
         }
         else if (!strcmp(at_com, "AT+CIMI\0") && p_resp[0] == '+' && p_resp[1] == 'C' && p_resp[2] == 'I' && p_resp[3] == 'M' && p_resp[4] == 'I')
         {
             while (p_resp[i] != ' ')
             {
                i++;
             }
             i++;
             while (p_resp[i] != '\0')
             {
                 if (j < sizeof(opts->imsi))
                 {
                     opts->imsi[j] = p_resp[i];
                     if (j <= 2)
                     {
                         opts->country_cod[j] = p_resp[i];
                         opts->country_cod[j + 1] = '\0';
                     }
                     if (j == 3 || j == 4)
                     {
                         opts->operator_cod[j - 3] = p_resp[i];
                         opts->operator_cod[j - 2] = '\0';
                     }
                     i++; j++;
                 }
             }
             switch (atoi(opts->country_cod))
             {
             case 250:
                 snprintf(opts->country_cod, sizeof(opts->country_cod), "RUS");
                 break;
             default:
                 snprintf(opts->country_cod, sizeof(opts->country_cod), "");
                 break;
             }
             switch (atoi(opts->operator_cod))
             {
             case 1:
                 snprintf(opts->operator_cod, sizeof(opts->operator_cod), "MTS");
                 break;
             case 2:
                 snprintf(opts->operator_cod, sizeof(opts->operator_cod), "Megafon");
                 break;
             case 11:
                 snprintf(opts->operator_cod, sizeof(opts->operator_cod), "Yota");
                 break;
             case 14:
                 snprintf(opts->operator_cod, sizeof(opts->operator_cod), "Megafon");
                 break;
             case 20:
                 snprintf(opts->operator_cod, sizeof(opts->operator_cod), "Tele2");
                 break;
             case 28:
                 snprintf(opts->operator_cod, sizeof(opts->operator_cod), "Beeline");
                 break;
             case 62:
                 snprintf(opts->operator_cod, sizeof(opts->operator_cod), "Tinkoff");
                 break;
             case 99:
                 snprintf(opts->operator_cod, sizeof(opts->operator_cod), "Beeline");
                 break;
             default:
                 snprintf(opts->operator_cod, sizeof(opts->operator_cod), "");
                 break;
             }
             opts->imsi[j] = '\0';
         }
         else if (!strcmp(at_com, "AT+CGSN\0") && p_resp[0] == '+' && p_resp[1] == 'C' && p_resp[2] == 'G' && p_resp[3] == 'S' && p_resp[4] == 'N')
         {
             while (p_resp[i] != ' ')
             {
                 i++;
             }
             i++;
             while (p_resp[i] != '\0')
             {
                 if (j < sizeof(opts->imei))
                 {
                     opts->imei[j] = p_resp[i];
                     i++; j++;
                 }
             }
             opts->imei[j] = '\0';
         }
         else if (!strcmp(at_com, "AT$MYGPSPOS=3\0") && p_resp[0] == '$' && p_resp[1] == 'M' && p_resp[2] == 'Y' && p_resp[3] == 'G' && p_resp[4] == 'P')
         {
             int n = sscanf(p_resp, "$MYGPSPOS: $GPRMC,%2d%2d%2d.%2d,%c,%2d%f,%c,%3d%f,%c",  &hh, &mm, &ss, &ms, &opts->valid_GPRMC, &lat_D, &lat_M, &opts->lat_sign, &lon_D, &lon_M, &opts->lon_sign);
             if (opts->lat_sign == '\0' || opts->lon_sign == '\0' || n != 11)
             {
                 opts->lat = 0;
                 opts->lon = 0;
                 opts->lat_sign = '-';
                 opts->lon_sign = '-';
             }
             else
             {
                 opts->lat = lat_D + lat_M / 60;
                 opts->lon = lon_D + lon_M / 60;
             }
             snprintf(opts->sput_time, sizeof(opts->sput_time),"%d:%d:%d", hh+3, mm, ss);
         }
         else if (!strcmp(at_com, "AT$MYGPSPOS=1\0") && p_resp[0] == '$' && p_resp[1] == 'M' && p_resp[2] == 'Y' && p_resp[3] == 'G' && p_resp[4] == 'P')
         {
             while (p_resp[i] != ',')
             {
                 i++;
             }
             i++;
             while (p_resp[i] != ',')
             {
                 i++;
             }
             i++;
             while (p_resp[i] != ',')
             {
                 if (p_resp[i] == '1')
                 {
                     snprintf(opts->threed_fix, sizeof(opts->threed_fix), "invalid\0");
                 }
                 if (p_resp[i] == '2')
                 {
                     snprintf(opts->threed_fix, sizeof(opts->threed_fix), "2D FIX\0");
                 }
                 if (p_resp[i] == '3')
                 {
                     snprintf(opts->threed_fix, sizeof(opts->threed_fix), "3D FIX\0");
                 }
                 i++;
             }
             i++;
             opts->num_sput_val = 0;
             while (opts->num_sput_val < 12)
             {
                 if (p_resp[i] == ',')
                 {
                     opts->num_sput_val++;
                     if (p_resp[i + 1] == ',')
                     {
                         break;
                     }
                 }
                 i++;
             }
             opts->gps_cords[j] = ' ';
             j++;
             while (p_resp[i] != ',')
             {
                 i++;
             }
             i++;
             while (p_resp[i] != ',')
             {
                 opts->gps_cords[j] = p_resp[i];
                 i++; j++;
                 if (j == 15)
                 {
                     j++;
                     j++;
                 }
                 if (p_resp[i] == '.')
                 {
                     i++;
                 }
             }
             opts->gps_cords[j] = '\0';
         }
         else if (!strcmp(at_com, "AT$MYSYSINFO\0"))
         {
             while (p_resp[i] != ' ')
             {
                 i++;
             }
             i++;
             switch (atoi(&p_resp[i]))
             {
             case 0:
                 snprintf(opts->mobile_data, sizeof(opts->mobile_data), "No Signal\0");
                 break;
             case 2:
                 snprintf(opts->mobile_data, sizeof(opts->mobile_data), "2G\0");
                 break;
             case 3:
                 snprintf(opts->mobile_data, sizeof(opts->mobile_data), "3G\0");
                 break;
             case 4:
                 snprintf(opts->mobile_data, sizeof(opts->mobile_data), "LTE\0");
                 break;
             }
         }
         else{
             printf("\033[92mUnknown command or mistake response!\nLOOOKHERE\nLOOOKBLYAT\nNOW\nLOOK!!!!\033[0m\n");
             sleep (3);
         }
     }
     else{
     #ifdef WEBPOSTGETINCONSOLE
         printf("Recv at cmd result: %s\n", p_result);
     #endif
     }
     pthread_mutex_unlock(&opts->mutex_modem);
     at_free(&p_resp);
     at_free(&p_result);
     return;
 }

 static void at_free(char** p)
 {
     if ((*p) != NULL)
     {
         free(*p);
         (*p) = NULL;
     }
 }
