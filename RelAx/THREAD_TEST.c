#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>
#include <regex.h>

#include "arpa/inet.h"
#include "nwy_at.h"
#include "nwy_common.h"
#include "nwy_gpio.h"
#include "nwy_pm.h"
#include "nwy_error.h"
#include "nwy_sms.h"


/*---------------------------Macro Definition---------------------------*/
#define PORT 80               //Требует Rут права

#undef LOG_FORMAT
#define LOG_FORMAT       LOG_SIMPLE
#undef LOG_MSG_TAG
#define LOG_MSG_TAG      "AT"
#undef LOG_MSG_LEVEL
#define LOG_MSG_LEVEL    LOG_DEBUG
#undef LOG_MSG_MASK
#define LOG_MSG_MASK     LOG_MASK_STD

#define NWY_AT_PORT                "/dev/smd8"
#define BUFLEN 512 //Max length of buffer
#define SERVER "10.7.254.6" //Адресс локальной машины пк связанной с железкой сейчас



char buffer[65000]  = {0};    //Буфер для вычитывания файлов и принятия внешних запросов
//Переменные, для вычитывания данных из внешнего запроса
char method[10]     = {0},    //Метод Post, get или что то еще
     fileadrr[100]  = {0},                //Адрес файла к которому обращение
     filetype[10]   = {0},                //Расширение файла
     postcomand[20] = {0},                //Команда для сервера через POST
     postbody[1000] = {0};                //Тело Post запроса
//Переменные, для хранения основных параметров монитора
char rssi[10] = {0},
     rsrq[10] = {0},
     snr[10] = {0},
     rxlen[10] = {0},
     spn[10] = {0},
     threed_fix[10] = {0},
     gps_cords[30] ={0},
     reg_in_mesh[10] = {0},
     mobile_data[10] = {0},
     imsi[20] = {0},
     imei[20] = {0},
     carrige_mileage[10] = {0},
     last_mileage[10] = {0},
     power_type[10] = {0},
     up_time_string[20] = {"0 : 0 : 0\0"};

int rssi_val = 0,
    rsrq_val = 0,
    snr_val = 0,
    rxlen_val = 0,
    spn_val = 0,
    threed_fix_val = 0,
    num_sput_val = 0,
    reg_in_mesh_val = 0,
    carrige_mileage_val = 0,
    last_mileage_val = 0,
    power_type_val = 0;

char country_cod[10], operator_cod[10];
int hh, mm, ss, ms;                 //Переменные времени от GPS идут в системное время
char fix_state;                     //Состояние фиксации спутниками нет/плоскость/3-х мерное пространство
int lat_D, lon_D;
float lat_M = 0, lon_M = 0, lat = 0, lon = 0;
char lat_sign = '\0', lon_sign= '\0';

char phone_num[13], sms_text[200]={'\0'}; //Переменные для работы с смс
char sms_queue_to_send[1000][200];
char sms_queue_to_recv[1000][200];
char sms_sended[1000][200];
char sms_recived[1000][200];
char sms_deleted[1000][200];
int sms_sended_index = 0;
int sms_recived_index = 0;
int sms_deleted_index = 0;
int sms_queue_to_send_index = 0;

char web_log[64000]= {0};
char inet_web[20] = {0};
char netmask_web[20] = {0};
char gps_time_web[100] = {0};

int opt = 1, rc;
int user_id = 0;                   //2 - Admin, 1 - Viewer, 0 - Her kakoi-to
int ret;
time_t start,end;
int server_fd, new_tcp_socket, valread;  //

struct sockaddr_in address;
int addrlen = sizeof(address);

void Header_Parse();        //Функция которая разбирает ключевые заголовки из buffer и складывает в предназначенные для этого массивы
void Start_Socket();             //Socket+bind+listen
void At_init();            //Инициализация для работы с AT
void send_at_cmd(char *at_com);    //Отправка АТ команды
static void at_free(char **p);
static void* TCP_server (void *arg);
static void* UDP_server (void *arg);
static void* HeartBit (void *arg);
static int do_send_sms(char *number, int encoding, int length, char *context, int async);
static void test_sms_evt_handler(nwy_mt_sms_event_t ind_type, void *ind_struct);

int main(int argc, char const *argv[])
{
        sprintf(web_log, "%s%s", web_log, "Start systems...\\n");
        system("ifconfig bridge0 inet 10.7.254.10 netmask 255.255.255.0\n");
        system("ethtool -s eth0 speed 100 duplex full autoneg on\n");

        pthread_t thread, thread2, thread3;
        pthread_create(&thread, NULL, TCP_server, "1");
        pthread_create(&thread2, NULL, UDP_server, "2");
        pthread_create(&thread3, NULL, HeartBit, "3");
        while(1)
        {
                printf("\033[92mNORMAL WORK MAIN\033[0m\n");
                sleep(5);
        }
        return 0;
}




void Start_Socket()
{
        //Создание дескриптера сокета
        if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
        {
                perror("socket failed");
                exit(EXIT_FAILURE);
        }
        start = time(NULL);
        printf("SOCKET: OK\n");
        // Forcefully attaching socket to the port 8080
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)))
        {
                perror("setsockopt");
                exit(EXIT_FAILURE);
        }

        address.sin_family = AF_INET;                         //Adress Family - Семейство адрессов для интернет домена либой IP сети
        address.sin_addr.s_addr = INADDR_ANY;                         //Если комп имеет несколько сетевых адрессов, и готовы соединяться с клиентом по любому из них
        address.sin_port = htons( PORT );                         //При указании IP-адреса и номера порта необходимо преобразовать число из порядка хоста в сетевой
        //htons (Host TO Network Short) и htonl (Host TO Network Long). Обратное преобразование выполняют функции ntohs и ntohl

        // Forcefully attaching socket to the port 8080
        if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0)
        {
                perror("bind failed");
                exit(EXIT_FAILURE);
        }
        printf("BIND: OK\n");
        if (listen(server_fd, 10) < 0)
        {
                perror("listen");
                exit(EXIT_FAILURE);
        }
        printf("LISTEN: OK\n");
}

void Header_Parse()
{
        int i = 0;
        int j = 0;
        fileadrr[0] = '\0';                                  //Костыль для корректной работе при перезагрузке страницы
        filetype[0] = '\0';
        postcomand[0] = '\0';
        postbody[0] = '\0';
        while (buffer[i] != ' ')                             //Обработка параметров первой строки
        {                                                    //Вида "GET / HTTP1.1"
                if (buffer[0] == '\0') break;
                method[j] = buffer[i];                       //POST or GET
                i++;
                j++;
        }
        method[j] = '\0';
        i++; j = 0; i++;
        while (buffer[i] != ' ')
        {
                if (buffer[i] == '\0') break;
                fileadrr[j] = buffer[i];                      //"index.sFile"?
                i++;
                j++;
                if (buffer[i] == '.')                         //Если в запрсе точка
                {
                        fileadrr[j] = '\0';                   //закончили читать имя файла
                        i++; j=0;
                        while (buffer[i] != ' ')              //читаем расширение файла
                        {
                                filetype[j] = buffer[i];      //"index.sFile"?
                                i++;
                                j++;
                        }
                        filetype[j] = '\0';
                        break;
                }
                else if (buffer[i] == ' ')
                {
                        fileadrr[j] = '\0';
                        break;
                }
        }
        if (method[0] == 'P')
        {
                while ((buffer[i] != '\n') || (buffer[i-2] != '\n'))
                {
                        i++;
                }
                i++;
                j = 0;
                while (buffer[i] != '\n')
                {
                        postcomand[j] = buffer[i];
                        i++;
                        j++;
                }
                postcomand[j] = '\0';
                i++; j = 0;
                while (buffer[i] != '\0')
                {
                        postbody[j] = buffer[i];
                        i++;
                        j++;
                }
                postbody[j] = '\0';
        }
        if (fileadrr[0] == '\0') {fileadrr[0] = 'i'; fileadrr[1] = 'n'; fileadrr[2] = 'd'; fileadrr[3] = 'e'; fileadrr[4] = 'x'; fileadrr[5] = '\0';}
        if (filetype[0] == '\0') {filetype[0] = 'h'; filetype[1] = 't'; filetype[2] = 'm'; filetype[3] = 'l'; filetype[4] = '\0';}
}

void At_init()
{
        ret = nwy_at_port_init(NWY_AT_PORT);
        if (ret != 0)
        {
                LOGI("Init port error \n", NWY_AT_PORT);
        }
        else
        {
                send_at_cmd("AT$MYGPSPWR=1\0");
        }
}

void send_at_cmd(char *at_com)
{
        char *p_resp = NULL;
        char *p_result = NULL;
        int ret;
        int i=0, j=0;
        ret = nwy_at_send_cmd(at_com, &p_resp, &p_result);
        if (ret != 0)
        {
                printf("Send at cmd %s failed\n", "AT+CSQ");
                return;
        }
        if(p_resp != NULL)
        {
                printf("Recv at response:\n%s\n%s\n", p_resp, p_result);

                if (!strcmp(at_com,"AT+CSQ\0"))
                {
                        while (p_resp[i]!=' ')
                        {
                                i++;
                        }
                        i++;
                        while (p_resp[i]!=',')
                        {
                                rssi[j]=p_resp[i];
                                i++; j++;
                        }
                        rssi[j]='\0';
                        rssi_val = -113 + atoi(rssi)*2;
                }
                else if (!strcmp(at_com,"AT+CIMI\0"))
                {
                        while (p_resp[i]!=' ')
                        {
                                i++;
                        }
                        i++;
                        while (p_resp[i]!= '\0')
                        {
                                imsi[j]=p_resp[i];
                                if(j <= 2)
                                {
                                        country_cod[j] = p_resp[i];
                                        country_cod[j+1] = '\0';
                                }
                                if(j == 3 || j == 4)
                                {
                                        operator_cod[j-3] = p_resp[i];
                                        operator_cod[j-2] = '\0';
                                }
                                i++; j++;
                        }
                        switch (atoi(country_cod))
                        {
                        case 250:
                                sprintf(country_cod, "RUS\0");
                                break;
                        default:
                                sprintf(country_cod, "\0");
                                break;
                        }
                        switch (atoi(operator_cod))
                        {
                        case 1:
                                sprintf(operator_cod, "MTS\0");
                                break;
                        case 2:
                                sprintf(operator_cod, "Megafon\0");
                                break;
                        case 11:
                                sprintf(operator_cod, "Yota\0");
                                break;
                        case 14:
                                sprintf(operator_cod, "Megafon\0");
                                break;
                        case 20:
                                sprintf(operator_cod, "Tele2\0");
                                break;
                        case 28:
                                sprintf(operator_cod, "Beeline\0");
                                break;
                        case 62:
                                sprintf(operator_cod, "Tinkoff\0");
                                break;
                        case 99:
                                sprintf(operator_cod, "Beeline\0");
                                break;
                        default:
                                sprintf(operator_cod, "\0");
                                break;
                        }
                        imsi[j]='\0';
                }
                else if (!strcmp(at_com,"AT+CGSN\0"))
                {
                        while (p_resp[i]!=' ')
                        {
                                i++;
                        }
                        i++;
                        while (p_resp[i]!= '\0')
                        {
                                imei[j]=p_resp[i];
                                i++; j++;
                        }
                        imei[j]='\0';
                }
                else if (!strcmp(at_com,"AT$MYGPSPOS=3\0"))
                {
                        sscanf(p_resp, "$MYGPSPOS: $GPRMC,%2d%2d%2d.%2d,%c,%2d%f,%c,%3d%f,%c",&hh,&mm,&ss,&ms,&fix_state,&lat_D,&lat_M,&lat_sign, &lon_D, &lon_M, &lon_sign);
                        if(lat_sign == '\0' || lon_sign == '\0')
                        {
                                lat = 0;
                                lon = 0;
                                lat_sign = '-';
                                lon_sign = '-';
                        }
                        else
                        {
                                lat = lat_D + lat_M/60;
                                lon = lon_D + lon_M/60;
                        }
                }
                else if (!strcmp(at_com,"AT$MYGPSPOS=1\0"))
                {
                        while (p_resp[i]!=',')
                        {
                                i++;
                        }
                        i++;
                        while (p_resp[i]!=',')
                        {
                                i++;
                        }
                        i++;
                        while (p_resp[i]!=',')
                        {
                                if(p_resp[i]=='1')
                                {
                                        sprintf(threed_fix, "invalid\0");
                                }
                                if(p_resp[i]=='2')
                                {
                                        sprintf(threed_fix, "2D FIX\0");
                                }
                                if(p_resp[i]=='3')
                                {
                                        sprintf(threed_fix, "3D FIX\0");
                                }
                                i++;
                        }
                        i++;
                        num_sput_val = 0;
                        while (num_sput_val < 12)
                        {
                                if (p_resp[i]== ',')
                                {
                                        num_sput_val++;
                                        if(p_resp[i+1]==',')
                                        {
                                                break;
                                        }
                                }
                                i++;
                        }
                        gps_cords[j]=' ';
                        j++;
                        while (p_resp[i]!=',')
                        {
                                i++;
                        }
                        i++;
                        while (p_resp[i]!= ',')
                        {
                                gps_cords[j]=p_resp[i];
                                i++; j++;
                                if(j == 15)
                                {
                                        j++;
                                        j++;
                                }
                                if (p_resp[i]=='.')
                                {
                                        i++;
                                }
                        }
                        gps_cords[j]='\0';
                }
                else if (!strcmp(at_com,"AT$MYSYSINFO\0"))
                {
                        while (p_resp[i]!=' ')
                        {
                                i++;
                        }
                        i++;
                        switch (atoi(&p_resp[i]))
                        {
                        case 0:
                                sprintf(mobile_data,"No Signal\0");
                                break;
                        case 2:
                                sprintf(mobile_data,"2G\0");
                                break;
                        case 3:
                                sprintf(mobile_data,"3G\0");
                                break;
                        case 4:
                                sprintf(mobile_data,"LTE\0");
                                break;
                        }
                }

        }
        else
                printf("Recv at response:\n%s\n", p_result);

        printf("Send at cmd %s success\n", ")(");

        at_free(&p_resp);
        at_free(&p_result);
        return;
}

static void at_free(char **p)
{
        if((*p) != NULL)
        {
                free(*p);
                (*p) = NULL;
        }
}


static void* TCP_server (void *arg)
{
        nwy_sms_add_mtmessage_handler(test_sms_evt_handler, NULL);
        char *at_com = malloc(sizeof(char)*100);
        int i = 0;
        int j = 0;
        FILE *sFile;                         //Открытие файлов
        long int nFileLen;                            //Сюда пишем позицию
        signal(SIGPIPE,SIG_IGN);                         //Игнорим ситуацию если пакет послан, но не был принят
        Start_Socket();                         //Запуск Socket+bind+listen
        At_init();                         //Инициализация для работы с AT
        //Работа с клиентом
        while(1)
        {
                new_tcp_socket = accept(server_fd, NULL, NULL);                                                 //(struct sockaddr *)&address, (socklen_t*)&addrlen);
                if (new_tcp_socket == -1)
                {
                        perror("accept");
                        continue;                                                                        //exit(EXIT_FAILURE);
                }
                end = time(NULL);
                printf("\033[91mACCEPT:\033[0m OK, %f seconds from start\n",difftime(end, start));
                valread = recv(new_tcp_socket, buffer, sizeof(buffer)-1, 0);                                                //Разбираем запрос от браузера
                end = time(NULL);
                printf("\033[91mRecv:\033[0m End, %f seconds from start\n",difftime(end, start));
                if (valread <= 0)
                {
                        send(new_tcp_socket, "HTTP/1.1 400\r\n", strlen("HTTP/1.1 400\r\n"), 0 );
                        //printf("Fatalerror\n");
                        close (new_tcp_socket);
                        continue;
                }
                buffer[valread] = '\0';                                                 //Добавляем конец строки на позиции конца строки
                printf("\033[91mREAD:\033[0m OK, %d symbols\n", valread);

                Header_Parse();
                //Вывод на экран всего что там напарсилось
                printf("%s\n",buffer);
                printf("\033[92mMethod:\033[0m %s\n",method);
                printf("\033[92mAddr:\033[0m %s \n",fileadrr);
                printf("\033[92mType:\033[0m %s\n",filetype);
                printf("\033[92mComand:\033[0m %s\n",postcomand);
                printf("\033[92mBody:\033[0m %s\n",postbody);
                sprintf(fileadrr,"%s.%s", fileadrr, filetype);                                                 //Собираем целый адресс из имени и расширения
                printf("\033[92mAddr:\033[0m %s \n",fileadrr);

                if (method[0] == 'P' && method[1] == 'O' && method[2] == 'S' && method[3] == 'T')
                {
                        //Авторизация под пользователем
                        if (!strcmp(postcomand, "validate\0")&&(!strcmp(postbody, "Admin 12345\0")))
                        {
                                send(new_tcp_socket, "HTTP/1.1 200 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
                                user_id = 2;
                        }
                        else if (!strcmp(postcomand, "validate\0")&&(!strcmp(postbody, "User 123\0")))
                        {
                                send(new_tcp_socket, "HTTP/1.1 200 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
                                user_id = 1;
                        }
                        else if (!strcmp(postcomand, "logout\0"))
                        {
                                send(new_tcp_socket, "HTTP/1.1 200 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
                                if(user_id == 2)
                                {
                                    sprintf(web_log, "%s%s%s", web_log, up_time_string, "Admin is unlogged\\n");
                                }
                                else if (user_id == 1)
                                {
                                    sprintf(web_log, "%s%s%s", web_log, up_time_string, "User is unlogged\\n");
                                }
                                user_id = 0;
                        }
                        else if (!strcmp(postcomand, "SMS_send\0"))
                        {
                                send(new_tcp_socket, "HTTP/1.1 200 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
                                int i = 0, j = 0;
                                while(postbody[i] != ' ')
                                {
                                        phone_num[j] = postbody[i];
                                        i++;
                                        j++;
                                }
                                phone_num[j] = '\0';
                                j = 0; i++;
                                while(postbody[i] != '\0')
                                {
                                        sms_text[j] = postbody[i];
                                        i++;
                                        j++;
                                }
                                sms_text[j] = '\0';
                                printf("Num: %s\n SMS: %s\n Len: %d\n", phone_num, sms_text, strlen(sms_text));
                                do_send_sms(phone_num, 0, strlen(sms_text), sms_text, 0);
                        }
                        else if (!strcmp(postcomand, "save_netstat\0"))
                        {
                                send(new_tcp_socket, "HTTP/1.1 200 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
                                int i = 0, j = 0;
                                while(postbody[i] != ' ')
                                {
                                        inet_web[j] = postbody[i];
                                        i++;
                                        j++;
                                }
                                inet_web[j] = '\0';
                                j = 0; i++;
                                while(postbody[i] != ' ')
                                {
                                        netmask_web[j] = postbody[i];
                                        i++;
                                        j++;
                                }
                                netmask_web[j] = '\0';
                                j = 0; i++;
                                while(postbody[i] != '\0')
                                {
                                        gps_time_web[j] = postbody[i];
                                        i++;
                                        j++;
                                }
                                gps_time_web[j] = '\0';
                                printf("inet: %s\nnetmask: %s\ngps_time: %s\n", inet_web, netmask_web, gps_time_web);
                                char sys_com[100] = {0};
                                sprintf (sys_com, "ifconfig bridge0 inet %s netmask %s\n", inet_web, netmask_web);
                                system(sys_com);
                                sprintf(web_log, "%s%sСетевая конфигурация обновлена!\\nip:%s\\nnetmask:%s\\n", web_log, up_time_string, inet_web, netmask_web);
                                //printf("%s", sys_com);
                                // j = 0; i++;
                        }
                        else
                        {
                                send(new_tcp_socket, "HTTP/1.1 400 Ok \r\n\r\n", strlen("HTTP/1.1 200 Ok \r\n\r\n"), 0);
                        }
                        close (new_tcp_socket);
                        end = time(NULL);
                        printf("\033[91mCLOSE:\033[0m OK, %f seconds from start\n", difftime(end, start));
                }



                if (method[0] == 'G' && method[1] == 'E' && method[2] == 'T')
                {
                        switch (user_id)
                        {
                        case 0:
                                if (!strcmp(fileadrr,"index.html\0") || !strcmp(filetype,"ico\0") || !strcmp(filetype,"png\0") || !strcmp(fileadrr,"blocked.html\0") || !strcmp(fileadrr,"login.js\0") || !strcmp(fileadrr,"picnic.css\0"))
                                {
                                        printf("Guest Ok\n");
                                }
                                else
                                {
                                        sprintf(fileadrr, "404.html");
                                        sprintf(filetype, "html");
                                        printf("Guest sheet %s\n", fileadrr);
                                }
                                break;
                        case 1:
                                if (!strcmp(fileadrr,"index.html\0"))
                                {
                                        sprintf(fileadrr, "indexWebViewer.html");
                                        sprintf(web_log, "%s%s%s", web_log, up_time_string, "User is logged\\n");
                                }
                                printf ("Hallo User :)\n");
                                break;
                        case 2:
                                if (!strcmp(fileadrr,"index.html\0"))
                                {
                                        sprintf(fileadrr, "indexWeb.html");
                                        sprintf(web_log, "%s%s%s", web_log, up_time_string, "Admin is logged\\n");
                                }
                                printf ("Hallo Admin :)\n");
                                break;
                        }


                        if (send(new_tcp_socket, "HTTP/1.1 200 OK\r\n", strlen("HTTP/1.1 200 OK\r\n"), 0 ) == -1)
                                printf("ERROR SEND\n");
                        if (fileadrr[0] == '.')                                                                         //Если запрос пустой, то кидаем на стартовую страницу
                        {
                                send(new_tcp_socket, "Content-Type: text/html;charset=utf-8\r\n\r\n", strlen("Content-Type: text/html;charset=utf-8\r\n\r\n"), 0 );
                        }
                        else                                                                                  //Иначе смотрим расширение запрошеного файла
                        {
                                if (filetype[0] == 'h' && filetype[1] == 't' && filetype[2] == 'm' && filetype[3] == 'l')
                                {
                                        send(new_tcp_socket, "Content-Type: text/html;charset=utf-8\r\n\r\n", strlen("Content-Type: text/html;charset=utf-8\r\n\r\n"), 0 );
                                }
                                if (filetype[0] == 'p' && filetype[1] == 'n' && filetype[2] == 'g')
                                {
                                        send(new_tcp_socket, "Content-Type: image/png;charset=utf-8\r\n\r\n", strlen("Content-Type: image/png;charset=utf-8\r\n\r\n"), 0 );
                                }
                                if (filetype[0] == 'i' && filetype[1] == 'c' && filetype[2] == 'o')
                                {
                                        send(new_tcp_socket, "Content-Type: image/webp;charset=utf-8\r\n\r\n", strlen("Content-Type: image/webp;charset=utf-8\r\n\r\n"), 0 );
                                }
                                if (filetype[0] == 't' && filetype[1] == 'x' && filetype[2] == 't')
                                {
                                        send(new_tcp_socket, "Content-Type: text/html;charset=utf-8\r\n\r\n", strlen("Content-Type: text/html;charset=utf-8\r\n\r\n"), 0 );
                                }
                                if (filetype[0] == 'j' && filetype[1] == 's' && filetype[2] == '\0')
                                {
                                        send(new_tcp_socket, "Content-Type: text/javascript;charset=utf-8\r\n\r\n", strlen("Content-Type: text/javascript;charset=utf-8\r\n\r\n"), 0 );
                                }
                                if (filetype[0] == 'j' && filetype[1] == 's' && filetype[2] == 'o' && filetype[3] == 'n')
                                {
                                        send(new_tcp_socket, "Content-Type: application/json;charset=utf-8\r\n\r\n", strlen("Content-Type: application/json;charset=utf-8\r\n\r\n"), 0 );
                                }
                                if (filetype[0] == 'c' && filetype[1] == 's' && filetype[2] == 's')
                                {
                                        send(new_tcp_socket, "Content-Type: text/css;charset=utf-8\r\n\r\n", strlen("Content-Type: text/css;charset=utf-8\r\n\r\n"), 0 );
                                }
                        }

                        if (!strcmp(fileadrr,"data.json\0"))
                        {
                                //strcpy(at_com, "AT+CSQ\0");
                                send_at_cmd("AT+CSQ\0");
                                send_at_cmd("AT+CIMI\0");
                                send_at_cmd("AT+CGSN\0");
                                send_at_cmd("AT$MYGPSPOS=1\0");
                                send_at_cmd("AT$MYGPSPOS=3\0");
                                send_at_cmd("AT$MYSYSINFO\0");
                                //strcpy(at_com, "AT+CIMI\0");
                                //send_at_cmd(at_com);
                                end = time(NULL);
                                //float time_f = difftime(end, start);
                                sprintf (up_time_string, "%2d : %2d : %2d\0", (int)(difftime(end, start)/3600)%60, (int)(difftime(end, start)/60)%60, (int)difftime(end, start)%60);
                                sprintf(buffer, "{\"rssi\" : \"%d дБм\",\n\"rsrq\" : \" - \",\n\"snr\" : \" - \",\n\"rxlen\" : \" - \"\n,\"spn\" : \" -\"\n,\"threed_fix\" : \"%s\"\n,\"gps_cords\" : \" %c %f\xC2\xB0 %c %f\xC2\xB0\"\n,\"sys_time\" : \" %d:%d:%d (GMT +3)\"\n,\"num_sput\" : \"%d\"\n,\"reg_in_mesh\" : \" %s %s\"\n,\"mobile_data\" : \"%s\"\n,\"imsi\" : \"%s\"\n,\"imei\" : \"%s\"\n,\"uptime\" : \"%s\"\n,\"carrige_mileage\" : \" - \"\n,\"last_mileage\" : \" -\"\n,\"power_type\" : \" -\"\n}\0", rssi_val, threed_fix, lat_sign, lat, lon_sign, lon, hh+3, mm, ss,num_sput_val, operator_cod, country_cod, mobile_data, imsi, imei, up_time_string);
                                send(new_tcp_socket, buffer, strlen(buffer), 0);
                                at_com = NULL;
                        }
                        else if (!strcmp(fileadrr,"log.json\0"))
                        {
                                sprintf(buffer, "{\"log\" : \"%s\"\n}\0", web_log);
                                send(new_tcp_socket, buffer, strlen(buffer), 0);
                        }
                        else
                        {
                                //sprintf(fileadrr, "/data/www/Server/", fileadrr);
                                sFile = fopen (fileadrr,"r");
                                if (sFile == NULL) printf ("ошибка\n");
                                else printf ("выполнено\n");
                                fseek (sFile, 0, SEEK_END);                                    //Открываем файл и перемещаем каретку в конечное положение
                                nFileLen = ftell(sFile);                                       //Получаем текущее значение указателя
                                fseek (sFile, 0, SEEK_SET);                                    //Перемещаем каретку в начало, чтобы корректно работать с файлом

                                for (i = 0; (rc = getc(sFile)) != EOF && i < nFileLen; buffer[i++] = rc);    //Посимвольно считываем все биты из файла пока они не закончатся или не переполнится буффер
                                buffer[i] = '\0';
                                send(new_tcp_socket, buffer, nFileLen, 0);                                   //Выдаем буфер браузеру
                                // Закрываем файл
                                printf ("Закрытие файла: ");
                                if ( fclose (sFile) == EOF) printf ("ошибка\n");
                                else printf ("выполнено\n");
                        }
                        close (new_tcp_socket);
                        end = time(NULL);
                        printf("\033[91mCLOSE:\033[0m OK, %f seconds from start\n", difftime(end, start));
                }
                printf("-------------------------\n");
        }

        close (server_fd);
        printf("\033[91mSERVER CLOSED:\033[0m OK \n");
        printf("-------------------------\n");
}

void die(char *s)
{
        perror(s);
        exit(1);
}

static void* UDP_server (void *arg)
{
        struct sockaddr_in si_other;
        int udp_fd, i, slen=sizeof(si_other);
        //char buf[BUFLEN];
        char message[BUFLEN];

        if ( (udp_fd=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
        {
                die("socket");
        }

        memset((char *) &si_other, 0, sizeof(si_other));
        si_other.sin_family = AF_INET;
        si_other.sin_port = htons(PORT);

        if (inet_aton(SERVER, &si_other.sin_addr) == 0)
        {
                fprintf(stderr, "inet_aton() failed\n");
                exit(1);
        }

        sprintf (message, "**UPD IS WORK\0");

        while(1)
        {
                printf("Enter message : %s\n", message);
                //send the message
                if (sendto(udp_fd, message, strlen(message), 0, (struct sockaddr *) &si_other, slen)==-1)
                {
                        die("sendto()");
                }
                sleep(1);
                //receive a reply and print it
                //clear the buffer by filling null, it might have previously received data
                //memset(buf,'&#92;&#48;', BUFLEN);
                //try to receive some data, this is a blocking call
                //if (recvfrom(udp_fd, buf, BUFLEN, 0, (struct sockaddr *) &si_other, &slen) == -1)
                //{
                //	die("recvfrom()");
                //}
        }

        close(udp_fd);
        return 0;
}

static void* HeartBit (void *arg)
{
        bool flag = false;
        while (1)
        {
                nwy_gpio_set_val(NWY_GPIO_76, flag);
                flag = !flag;
        }
}


static int do_send_sms(char *number, int encoding, int length, char *context, int async)
{
        int result = 0;
        /*
           char phone_num[NWY_SMS_MAX_ADDR_LENGTH];
           uint32_t msg_context_len;
           char msg_context[NWY_SMS_MAX_MO_MSG_LENGTH + 1];
           nwy_sms_msg_format_type_t msg_format;
         */

        nwy_sms_info_type_t sms_data = {0};

        strcpy(sms_data.phone_num, number);
        sms_data.msg_context_len = length;



        if (encoding == 1) {
                sms_data.msg_format = NWY_SMS_MSG_FORMAT_TEXT_UTF8;
        }
        else if (encoding == 2) {
                sms_data.msg_format = NWY_SMS_MSG_FORMAT_TEXT_GBK;
                length = strlen(context);
        }
        else if (encoding == 0)
                sms_data.msg_format = NWY_SMS_MSG_FORMAT_TEXT_ASCII;
        else if (encoding == 3)
                sms_data.msg_format = NWY_SMS_MSG_FORMAT_BIN;


        memcpy(sms_data.msg_context, context, length + 1);

        //int nwy_sms_send_message ( nwy_sms_info_type_t *p_sms_data );
        /*if (async == 1) {

            char *h = input_fgets("please set the max waiting time (by second)\n");
            int r = atoi(h);

            if (is_all_dig(h) != 1) {
                printf("input incorrect\n");
                return -1;
            }

            result = nwy_sms_send_message_async(&sms_data, r);
           }
           else if (async == 2) {

            char *h = input_fgets("please set the repeat time\n");
            int r = atoi(h);
            int c = 0;
            int i = 0;

            if (is_all_dig(h) != 1) {
                printf("input incorrect\n");
                return -1;
            }

            for (i = 0; i < r; i++) {
                result = nwy_sms_send_message(&sms_data);
                if (result != 0) {
                    c++;
                }
                printf("this time result is %d\n", result);
                printf("send %d time failed %d \n", i, c);
            }
            printf("send %d time failed %d \n", i, c);
           }
           else {*/
        result = nwy_sms_send_message(&sms_data);
        printf("%d", result);
        //}

        return result;
}

static void test_sms_evt_handler(nwy_mt_sms_event_t ind_type, void *ind_struct)
{
        static int c = 0;

        printf("Nwy_mt_sms_event %x\n", ind_type);
        switch (ind_type) {
        case NWY_SMS_PP_IND:
        {
                nwy_sms_pp_info_type_t *sms_pp;
                sms_pp = ind_struct;
                printf("recv msg from %s\n", sms_pp->source_phone_num);
                printf("recv msg type %d\n", sms_pp->msg_format);
                if (sms_pp->concat_sms_total > 0) {
                        printf("this is the part %d of long message has %d message entry\n",
                               sms_pp->concat_sms_cur, sms_pp->concat_sms_total);
                        printf("concat msg id %d\n", sms_pp->concat_sms_id);
                }
                if (sms_pp->msg_format == NWY_SMS_MSG_FORMAT_TEXT_ASCII)
                        printf("recv msg text %s\n", sms_pp->msg_content);
                else {
                        int i = 0;
                        printf("recv msg data:\n", sms_pp->msg_content);
                        for (i = 0; i < sms_pp->msg_content_len; i++) {
                                printf("%02x", sms_pp->msg_content[i]);
                        }
                        printf("\n");;
                }

                if (sms_pp->context_decode_type == NWY_SMS_ENCODING_GBK) {

                        printf("gbk data is :\n%s\n", sms_pp->msg_decoded_content);
                }

                c++;
        }
        break;
        case NWY_SMS_SEND_IND:
        {
                nwy_sms_send_ind_t *send_result = ind_struct;
                if (send_result->result == 0) {
                        printf("we send msg to %s result ok\n", send_result->phone_num);
                } else {
                        printf("we send msg to %s result failed\n", send_result->phone_num);
                }
        }
        break;
        default:
                printf("we do not support this event now %x\n", ind_type);
        }
        printf("have recv %d sms entry\n");
        printf("--------------------------------------\n");
        fflush(stdout);
}
