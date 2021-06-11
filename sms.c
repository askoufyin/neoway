#include "uart.h"
#include "modem.h"
#include "thread_funcs.h"
#include "sms.h"

#include <sys/select.h>
#include <termios.h>
#include <stdbool.h> //

#include "nwy_error.h"
#include "nwy_common.h"
#include "nwy_sim.h"




 #define MAX_SMS_IN_PRIORITY 20 //Предельное количество смс в каждом приоритете. Приоритеты 1-7
 #define MAX_SMS_LENGTH 160     //Предельное количество символов в смс, годится только для отправки латиницы


 /*char phone_num[13], sms_text[500] = { '\0' }; //Переменные для работы с смс
 char recv_phone[13], recv_text[500] = { '\0' };
 bool recv_flag = false;*/
 // Stops at any null characters.
 static int decode_code_point( char **s )
 {
     // Count # of leading 1 bits.
     int k = **s ? __builtin_clz( ~( **s << 24) ) : 0;
     // All 1's with k leading 0's.
     int mask = ( 1 << ( 8 - k ) ) - 1;
     int value = **s & mask;
     // Note that k = #total bytes, or 0.
     for ( ++( *s ), --k; k > 0 && **s; --k, ++( *s ) ) {
         value <<= 6;
         value += ( **s & 0x3F );
     }
     return value;
 }

 /*  */
 void transliterate( char *in, uint32_t len,
                     char *out, uint32_t out_sizeof )
 {
     char *s = in;
     uint32_t l = strlen( in );

     char *table[] = {
         "A", "B", "V", "G", "D", "E", "ZH", "Z", "I", "Y", "K", "L", "M", "N", "O", "P", "R", "S", "T", "U", "F", "KH", "C", "CH", "SH", "JSH", "HH", "IH", "JH", "EH", "JU", "JA",
         "a", "b", "v", "g", "d", "e", "zh", "z", "i", "y", "k", "l", "m", "n", "o", "p", "r", "s", "t", "u", "f", "kh", "c", "ch", "sh", "jsh", "hh", "ih", "jh", "eh", "ju", "ja"};

     uint32_t i = 0;

     while( s < in + len ) {
         if( out_sizeof - 4 < i ) {
             return;
         }

         int d = decode_code_point( &s );
         if( d >= 1040 && d <= 1103 ) {
             /* А-Я а-я */
             out[ i++ ] = table[ d - 1040 ][ 0 ];
             if( table[ d - 1040 ][ 1 ] != '\0' ) {
                 out[ i++ ] = table[ d - 1040 ][ 1 ];
                 if(table[ d - 1040 ][ 2 ] != '\0')
                     out[ i++ ] = table[ d - 1040 ][ 2 ];
             }
         } else if( d == 1105 ) {
             /* ё */
             out[ i++ ] = 'e';
         } else if( d == 1025 ) {
             /* Ё */
             out[ i++ ] = 'E';
         } else {
             /* Остальное */
             out[ i++ ] = d;
         }
         out[ i ] = '\0';
     }
 }


 static int _sms_last_index = 0;

int is_sms_empty(SingleSMStoGet_t *sms)
 {
     if(sms->phone[0] == 0) {
         return 1;
     } else {
         return 0;
     }
 }

 static SingleSMStoGet_t *sms_get_by_index(int index, options_t *opts)
 {
     int i;
     for(i = 0; i < UPVS_SMS_LEN; i++)
     {
         if (index == opts->get_single_sms[i].index) {
             return &opts->get_single_sms[i];
         }
     }
     return NULL;
 }

int send_upvs_sms(char *phone, char *text, int length, int encoding)
{
    //options_t* opts = (options_t*)opts_sms;
    int result = 0;
    //int length;
    nwy_sms_info_type_t sms_data = { 0 };
    strcpy(sms_data.phone_num, phone);
    sms_data.msg_context_len = length;

    if (encoding == 1) {
        sms_data.msg_format = NWY_SMS_MSG_FORMAT_TEXT_UTF8;
    }
    else if (encoding == 2) {
        sms_data.msg_format = NWY_SMS_MSG_FORMAT_TEXT_GBK;
        //length = strlen(text);
    }
    else if (encoding == 0)
        sms_data.msg_format = NWY_SMS_MSG_FORMAT_TEXT_ASCII;
    else if (encoding == 3)
        sms_data.msg_format = NWY_SMS_MSG_FORMAT_BIN;

    uint32_t len = strlen(text);
    char oo[512] = "";
    transliterate( text, len, oo, 320 );
    printf( "out: %s\n", oo );
    memcpy(sms_data.msg_context, oo, strlen(oo) + 1);

    result = nwy_sms_send_message(&sms_data);
    return result;
}


int sms_add(char* phone, char* text, char* ttl, char* priority, options_t *opts, SingleSMS_status status)
 {
     int i = 0;
     if(_sms_last_index > 19)
     {
         int circle_index = _sms_last_index%UPVS_SMS_LEN;
         snprintf(opts->get_single_sms[circle_index].phone, sizeof(opts->get_single_sms[circle_index].phone), 0);
         snprintf(opts->get_single_sms[circle_index].text, sizeof(opts->get_single_sms[circle_index].text), 0);
         snprintf(opts->get_single_sms[circle_index].priority, sizeof(opts->get_single_sms[circle_index].priority), 0);
         snprintf(opts->get_single_sms[circle_index].ttl, sizeof(opts->get_single_sms[circle_index].ttl), 0);
         opts->get_single_sms[circle_index].status = SSMS_NAN;
         //return opts->get_single_sms[i].index;
     }
     for (i = 0; i < UPVS_SMS_LEN; i++)
     {
         if(is_sms_empty(&opts->get_single_sms[i]) == 1)
         {
             opts->get_single_sms[i].index = _sms_last_index++;
             snprintf(opts->get_single_sms[i].phone, sizeof(opts->get_single_sms[i].phone), phone);
             snprintf(opts->get_single_sms[i].text, sizeof(opts->get_single_sms[i].text), text);
             snprintf(opts->get_single_sms[i].priority, sizeof(opts->get_single_sms[i].priority), priority);
             snprintf(opts->get_single_sms[i].ttl, sizeof(opts->get_single_sms[i].ttl), ttl);
             opts->get_single_sms[i].status = status;
             save_sms_struct("/data/smsstruct.dat", opts->get_single_sms, UPVS_SMS_LEN);
             return opts->get_single_sms[i].index;
         }
     }
     return -1;
 }

 void clear_out_queue(SingleSMStoGet_t *sms)
 {
     memset(&sms[0], 0, sizeof(SingleSMStoGet_t)*UPVS_SMS_LEN);
     _sms_last_index = 0;
 }

void get_variable_queue(char *out, int len, options_t *opts)
 {
     int i = 0;
     int buflen = 0;
     for (i = 0; i < UPVS_SMS_LEN; i++)
     {
         if(opts->get_single_sms[i].status == SSMS_QUEUE)
         {
             buflen += snprintf(&out[buflen], len-buflen, "<index><value>%d</value></index>", opts->get_single_sms[i].index);
         }
     }
     if (buflen == 0)
     {
         buflen = snprintf(&out[buflen], len-buflen, "<index><value></value></index>");
     } else {
         //return buflen;
    }
 }

 void get_variable_deleted(char *out, int len, options_t *opts)
  {
      int i = 0;
      int buflen = 0;
      for (i = 0; i < UPVS_SMS_LEN; i++)
      {
          if(opts->get_single_sms[i].status == SSMS_DELETED)
          {
              buflen += snprintf(&out[buflen], len-buflen, "<index><value>%d</value></index>", opts->get_single_sms[i].index);
          }
      }
      if (buflen == 0)
      {
          buflen = snprintf(&out[buflen], len-buflen, "<index><value></value></index>");
      } else {
          //return buflen;
     }
  }

 void get_variable_sended(char *out, int len, options_t *opts)
 {
     int i = 0;
     int buflen = 0;
     for (i = 0; i < UPVS_SMS_LEN; i++)
     {
         if(opts->get_single_sms[i].status == SSMS_SENDED)
         {
             buflen += snprintf(&out[buflen], len-buflen, "<index><value>%d</value></index>", opts->get_single_sms[i].index);
         }
     }
     if (buflen == 0)
     {
         buflen = snprintf(&out[buflen], len, "<index><value></value></index>");
     } else {
         //return buflen;
    }
 }

void *out_from_queue(void* web_opts)
{
    int i;
    options_t* opts = (options_t*)web_opts;
    SingleSMStoGet_t *sms = opts->get_single_sms;
    while (1)
    {
        printf("Out_from_queue: \n");
        pthread_mutex_lock(&opts->mutex_sms);
        for(i = 0; i < UPVS_SMS_LEN; i++)
        {
            int res = 0;
            if(sms[i].status == SSMS_QUEUE)
            {
                //pthread_mutex_lock(&opts->mutex_modem);
                send_at_cmd("AT+CSQ\0", opts);
                //pthread_mutex_unlock(&opts->mutex_modem);
                printf("%d\n",opts->rssi_val);
                if (opts->rssi_val < 0 && opts->rssi_val > -85)
                {
                    printf("Try to send index: %d\nphone: %s\n text: %s\n\n", sms[i].index, sms[i].phone, sms[i].text);
                    res = send_upvs_sms(sms[i].phone, sms[i].text, strlen(sms[i].text), 0);
                    int sms_ind = -1;
                    SingleSMS_status status;
                    if (res == 0){
                        status = SSMS_SENDED;
                        sms[i].status = status;
                        printf("Sended!\n\n");
                    } else {
                        status = SSMS_QUEUE;
                        sms[i].status = status;
                        printf("Queue!\n\n");
                    }
                } else {
                    printf("Bad signal\n");
                }
            }

        }
        pthread_mutex_unlock(&opts->mutex_sms);
        sleep(10);
    }
}

void count_sms(SingleSMStoGet_t *sms)
{
    int i;
    for (i = 0; i < UPVS_SMS_LEN; i++)
    {
        printf("%s\n%d\n\n", sms[i].phone, sms[i].index);
        if (sms[i].phone[0] == 0)
        {

        } else {
            if(sms[i].index > _sms_last_index)
            _sms_last_index = sms[i].index;
        }
    }
    _sms_last_index++;
    printf("Count: %d\n", _sms_last_index);
}



 int get_sms_by_xml(int index_xml, void* web_opts, char *phone, char *text, char *ttl, char *priority)
 {
     options_t* opts = (options_t*)web_opts;
     if (index_xml >= _sms_last_index || index_xml < 0)
     {
         return -1;
     }
     int i = 0;
     opts->one_sms.index = index_xml;
     for (i = 0; i < UPVS_SMS_LEN; i++)
     {
          if(opts->get_single_sms[i].index == index_xml)
          {
              int res = is_sms_empty(&opts->get_single_sms[i]);
              if (res == 1) {
                  snprintf(phone, sizeof("+X(XXX)XXX-XX-XX"), "%s", "+X(XXX)XXX-XX-XX");
                  snprintf(text, sizeof(text), "%s", "");
                  snprintf(priority, sizeof(priority), "%s", "0");
                  snprintf(ttl, sizeof(ttl), "%s", "255");
              } else {
                  snprintf(phone, sizeof(opts->get_single_sms[i].phone), "%s", opts->get_single_sms[i].phone);
                  snprintf(text, sizeof(opts->get_single_sms[i].text), "%s", opts->get_single_sms[i].text);
                  snprintf(priority, sizeof(opts->get_single_sms[i].priority), "%s", opts->get_single_sms[i].priority);
                  snprintf(ttl, sizeof(opts->get_single_sms[i].ttl), "%s", opts->get_single_sms[i].ttl);
              }
              break;
          }
     }
     printf("Phone: %s\nText: %s\nPriority: %s\nTTL: %s\n\n", phone, text, priority, ttl);
     return 0;
 }

int save_sms_struct(char * filename, SingleSMStoGet_t * sms, int n)
{
    FILE * fp;
    char *c;
    if ((fp = fopen(filename, "wb+")) == NULL)
    {
        perror("Error occured while opening file");
        printf("SMS_WRITE_ERROR_wb+");
        fclose(fp);
        if ((fp = fopen(filename, "wb")) == NULL)
        {
            perror("Error occured while opening file");
            printf("SMS_WRITE_ERROR_wb");
            return 1;
        }
        //return 1;
    }
    fwrite(&sms[0], sizeof(SingleSMStoGet_t), n, fp);
    fclose(fp);
    printf("SMS_WRITE_OK");
    return 0;
}

int load_sms_struct(char * filename, SingleSMStoGet_t * sms, int n)
{
    FILE * fp;
    if ((fp = fopen(filename, "rb")) == NULL)
    {
        perror("Error occured while opening file");
        printf("SMS_READ_ERROR");
        return 1;
    }
    fread(&sms[0], sizeof(SingleSMStoGet_t), n, fp);
    fclose(fp);
    printf("SMS_READ_OK");
    return 0;
}

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
