#ifndef AT_COMMANDS_H
#define AT_COMMANDS_H


typedef enum {
    AT_UNKNOWN = 0,
    AT_GET_RECORD,
    AT_PEEK_RECORD,
    AT_GET_RECORD_COUNT,
    AT_CLEAR_BUFFER
} atmsg_id;


typedef struct _at_msg {
    atmsg_id msgid;
    const char *msgbody;
}
at_msg_t;


#ifdef __cplusplus
extern "C" {
#endif
extern atmsg_id at_which_command(const char *);
#ifdef __cplusplus
}
#endif


#endif // AT_COMMANDS_H
