#ifndef SMS_H
#define SMS_H


#include <stdint.h>
#include <pthread.h>
#include "utils.h"

#include "nwy_sms.h"


#ifdef __cplusplus
extern "C" {
#endif
int is_sms_empty(SingleSMStoGet_t *sms);
int sms_add(char* phone, char* text, char* ttl, char* priority, options_t *opts, SingleSMS_status status);
int save_sms_struct(char *filename, SingleSMStoGet_t *sms, int n);
int load_sms_struct(char *filename, SingleSMStoGet_t *sms, int n);
void count_sms(SingleSMStoGet_t *sms);
int get_sms_by_xml(int index_xml, void* web_opts, char *phone, char *text, char *ttl, char *priority);
int send_upvs_sms(char *phone, char *text, int length, int encoding);
void clear_out_queue(SingleSMStoGet_t *sms);
void *out_from_queue(void* web_opts);
void get_variable_queue(char *out, int len, options_t *opts);
void get_variable_sended(char *out, int len, options_t *opts);
void get_variable_deleted(char *out, int len, options_t *opts);
extern int Read_smsFrom_txt(const char* file, void* web_opts);
extern int Write_smsTo_txt(const char* file, void* web_opts);
#ifdef __cplusplus
}
#endif


#endif // SMS_H
