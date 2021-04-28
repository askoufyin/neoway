#ifndef __UPVS_H_INCLUDED__
#define __UPVS_H_INCLUDED__


#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#define MAX_XML_NESTING_DEPTH   10


enum {
    XML_INFO = 0,
    XML_SMS,
    XML_ROUTER,
    XML_QUERY_STATE_VARIABLE,
    XML_INFO2,
    XML_WEB,
    //
    XML_MAX
};


typedef struct _xml_tag_attr {
    char *name;
    char *value;
    struct _xml_tag_attr *next;
} xml_tag_attr_t;


typedef struct _xml_tag {
    char *name;
    char *content;
    xml_tag_attr_t *attrs;
    struct _xml_tag *parent;
    struct _xml_tag *next;
    struct _xml_tag *child;
} xml_tag_t;


typedef struct _buf {
    size_t alloc;
    size_t size;
    char *data;
} buf_t;


typedef struct _xml_context {
    int level;
    int tag_closed;
    buf_t *tags;
    buf_t *strings;
    xml_tag_t *root;
    xml_tag_t *last;
} xml_context_t;


#ifdef __cplusplus
extern "C" {
#endif
extern int buf_init(buf_t *, size_t);
extern void buf_reset(buf_t *);
extern char *buf_strdup(buf_t *, const char *);
extern char *buf_strndup(buf_t *, const char *, size_t);
extern char *buf_strncpy(buf_t *, const char *, size_t);
extern xml_tag_t *xml_tag(xml_context_t *, const char *, const char *);
extern char *xml_tag_string(xml_tag_t *);
extern xml_tag_t *xml_find_tag(xml_tag_t *, const char *, int);
extern int xml_is(xml_tag_t *, const char *);
extern void xml_context_init(xml_context_t *, buf_t *, buf_t *);
extern void xml_context_reset(xml_context_t *);
#ifdef __cplusplus
}
#endif

#endif
