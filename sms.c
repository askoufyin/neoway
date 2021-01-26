#include "uart.h"
#include "modem.h"
#include "thread_funcs.h"

#include <strings.h>
#include <sys/select.h>
#include <termios.h>

//#include "nwy_loc.h"
#include "nwy_uart.h"
#include "nwy_error.h"
#include "nwy_common.h"
#include "nwy_sim.h"


 #define MAX_SMS_IN_PRIORITY 20 //Предельное количество смс в каждом приоритете. Приоритеты 1-7
 #define MAX_SMS_LENGTH 160     //Предельное количество символов в смс, годится только для отправки латиницы


 int Write_smsTo_txt(const char* file, void* web_opts) {
     FILE* fp;
     char Write_sms_buff[MAX_SMS_IN_PRIORITY * MAX_SMS_LENGTH * 7 * 4];               //buffer для записи всех смс в файл
     memset(Write_sms_buff, 0, MAX_SMS_IN_PRIORITY * MAX_SMS_LENGTH * 7 * 4);
     char sended_sms[MAX_SMS_IN_PRIORITY * MAX_SMS_LENGTH * 7] = { 0 };
     char recved_sms[MAX_SMS_IN_PRIORITY * MAX_SMS_LENGTH * 7] = { 0 };
     char queue_sms[MAX_SMS_IN_PRIORITY * MAX_SMS_LENGTH * 7] = { 0 };
     char deleted_sms[MAX_SMS_IN_PRIORITY * MAX_SMS_LENGTH * 7] = { 0 };
     int i, j, s_p;
     options_t* opts = (options_t*)web_opts;

     printf("Saving sms memory to %s\n", file);

     for (s_p = 7; s_p >= 0; s_p--)
     {
         for (i = 0; i <= opts->sended[s_p].j - 1; i++) {
             snprintf(sended_sms, sizeof(sended_sms), "%s%d %s %s\r\n", sended_sms, s_p, opts->sended[s_p].phone[i], opts->sended[s_p].text[i]);
         }
         for (i = 0; i <= opts->recved[s_p].j - 1; i++) {
             snprintf(recved_sms, sizeof(sended_sms), "%s%d %s %s\r\n", recved_sms, s_p, opts->recved[s_p].phone[i], opts->recved[s_p].text[i]);
         }
         for (i = 0; i <= opts->queue[s_p].j - 1; i++) {
             snprintf(queue_sms, sizeof(sended_sms), "%s%d %s %s\r\n", queue_sms, s_p, opts->queue[s_p].phone[i], opts->queue[s_p].text[i]);
         }
         for (i = 0; i <= opts->deleted[s_p].j - 1; i++) {
             snprintf(deleted_sms, sizeof(sended_sms), "%s%d %s %s\r\n", deleted_sms, s_p, opts->deleted[s_p].phone[i], opts->deleted[s_p].text[i]);
         }
     }
     snprintf(Write_sms_buff, sizeof(Write_sms_buff), "Sended\r\n%sRecved\r\n%sQueue\r\n%sDeleted\r\n%s\0", sended_sms, recved_sms, queue_sms, deleted_sms);
     //printf("%s", buff); //отладка
     char fullpath[30];
     strcat(fullpath, opts->web_dir_i_path);   //собираем адрсс из PATH в neowayhelper.conf
     strcat(fullpath, "/");                    //слэша
     strcat(fullpath, file);               //и имени файла
     printf("Fopen: %s\n", fullpath);
     fp = fopen(fullpath, "wb");
     if (NULL == fp) {
         //perror("fopen()");
         printf("fd = NULL\n");
         //return -1;
     }
     /*Тут запись в файл fwrite*/
     //snprintf(buff, "%s\0", "Sended\r\n3 +79108441072 Some sms text one\r\n3 +79108441072 Some sms text two\r\n5 +79108441072 Some sms text three\r\n3 +79108441072 Some sms text four\r\n7 +79108441072 Some sms text Five\r\nQueue\r\nRecved\r\nDeleted\r\n6 +79108441072 Some sms text six\r\n3 +79108441072 Some sms text Seven");
     fwrite(&Write_sms_buff, 1, sizeof(Write_sms_buff), fp);
     //fflush(fp);
     fclose(fp);
     return 0;
 }

 int Read_smsFrom_txt(const char* file, void* web_opts) {
     FILE* fp;
     char str[1024];
     char Read_sms_buf[500];
     char* s, * e;
     int i, line, s_p, j;
     options_t* opts = (options_t*)web_opts;
     int flag = 0;    //Определяет в какие переменные складываем строку
     printf("Loading configuration from %s: ", file);

     /* First, copy contents of passed options into the temporary variable */
     //memcpy(&_opts, opts, sizeof(_opts));
     /* Now open and parse configuration file */
     char fullpath[30] = {0};
     strcat(fullpath, opts->web_dir_i_path);   //собираем адрсс из PATH в neowayhelper.conf
     strcat(fullpath, "/");                    //слэша
     strcat(fullpath, file);               //и имени файла
     fp = fopen(fullpath, "r");
     if (NULL == fp) {
         fp = fopen(fullpath, "wb");
         if(fp != NULL)
         {
             fclose(fp);
             printf("sms_memory.txt was create\n");
             fp = fopen(fullpath, "r");
         }
     }
         if (fp != NULL)
         {
             line = 1;
             while (!feof(fp)) {
                 memset(str, sizeof(str), 0);
                 fgets(str, sizeof(str) - 1, fp);

                 /* Search for comment and cut it off */
                 s = strchr(str, '#');
                 if (NULL != s) {
                     *s = '\0';
                 }

                 if (0 == strlen(str)) {
                     continue;
                 }

                 /* Skip leading spaces */
                 for (s = str; isspace(*s); s++);

                 /* Skip trailing spaces */
                 for (i = strlen(s) - 1; i > 0 && isspace(s[i]); i--);
                 s[i + 1] = '\0';

                 if (0 != strlen(s)) {
                     //printf("%d %s\n", line, s);
                     /*parse this string*/
                     if (!strcmp("Sended\0", s)) {
                         flag = 1;
                     }
                     else if (!strcmp("Queue\0", s)) {
                         flag = 2;
                     }
                     else if (!strcmp("Recved\0", s)) {
                         flag = 3;
                     }
                     else if (!strcmp("Deleted\0", s)) {
                         flag = 4;
                     }
                     else {
                         i = 0; j = 0;
                         while (s[i] != ' ') {
                             Read_sms_buf[j] = s[i];
                             i++; j++;
                         }
                         Read_sms_buf[j] = '\0';
                         j = 0; i++;
                         s_p = atoi(Read_sms_buf);
                         while (s[i] != ' ') {
                             Read_sms_buf[j] = s[i];
                             i++; j++;
                         }
                         Read_sms_buf[j] = '\0';
                         j = 0; i++;
                         //printf ("flag:%d\r\n", flag);
                         switch (flag) {
                         case 1:
                             snprintf(opts->sended[s_p].phone[opts->sended[s_p].j],13, Read_sms_buf);
                             break;
                         case 2:
                             snprintf(opts->queue[s_p].phone[opts->queue[s_p].j], 13, Read_sms_buf);
                             break;
                         case 3:
                             snprintf(opts->recved[s_p].phone[opts->recved[s_p].j], 13, Read_sms_buf);
                             break;
                         case 4:
                             snprintf(opts->deleted[s_p].phone[opts->deleted[s_p].j], 13, Read_sms_buf);
                             break;
                         }
                         while (s[i] != '\0') {
                             Read_sms_buf[j] = s[i];
                             i++; j++;
                         }
                         Read_sms_buf[j] = '\0';
                         switch (flag) {
                         case 1:
                             snprintf(opts->sended[s_p].text[opts->sended[s_p].j], 500, Read_sms_buf);
                             printf("Parse! %d\r\n%s\r\n%s\r\n\r\n", s_p, opts->sended[s_p].phone[opts->sended[s_p].j], opts->sended[s_p].text[opts->sended[s_p].j]);
                             opts->sended[s_p].j++;
                             break;
                         case 2:
                             snprintf(opts->queue[s_p].text[opts->queue[s_p].j], 500, Read_sms_buf);
                             printf("Parse! %d\r\n%s\r\n%s\r\n\r\n", s_p, opts->queue[s_p].phone[opts->queue[s_p].j], opts->queue[s_p].text[opts->queue[s_p].j]);
                             opts->queue[s_p].j++;
                             break;
                         case 3:
                             snprintf(opts->recved[s_p].text[opts->recved[s_p].j], 500, Read_sms_buf);
                             printf("Parse! %d\r\n%s\r\n%s\r\n\r\n", s_p, opts->recved[s_p].phone[opts->recved[s_p].j], opts->recved[s_p].text[opts->recved[s_p].j]);
                             opts->recved[s_p].j++;
                             break;
                         case 4:
                             snprintf(opts->deleted[s_p].text[opts->deleted[s_p].j], 500, Read_sms_buf);
                             printf("Parse! %d\r\n%s\r\n%s\r\n\r\n", s_p, opts->deleted[s_p].phone[opts->deleted[s_p].j], opts->deleted[s_p].text[opts->deleted[s_p].j]);
                             opts->deleted[s_p].j++;
                             break;
                         }
                         j = 0; i = 0;

                     }
                     line++;
                 }
             }
             printf("OK\n");
         }
         else {
             printf("ERROR\n");
         }
     fclose(fp);
     return 0;
 }
