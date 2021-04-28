#include "upvs.h"


int
buf_init(buf_t *buf, size_t initital_alloc)
{
    if(NULL == buf)
        return -1;

    buf->size = 0;
    buf->alloc = 0;
    buf->data = (char *)malloc(initital_alloc);
    if(NULL == buf->data)
        return -2;

    buf->alloc = initital_alloc;
    return 0;
}


void
buf_reset(buf_t *buf)
{
    buf->size = 0;
}


char
*buf_alloc(buf_t *buf, size_t cb)
{
    char *res, *temp;
    size_t newcb;

    newcb = buf->size + cb + 1;
    if(newcb > buf->alloc) {
        /* Caution! realloc() is no guarantee that the returned pointer will be the
         * same as that has been passed in, so in this case, all pointers to XML
         * tags and tag attributes will become invalid!
         */
        temp = (char *)realloc(buf->data, buf->alloc*2);
        if(NULL == temp)
            return NULL;
        buf->data = temp;
        buf->alloc = buf->alloc*2;
    }

    res = buf->data + buf->size;
    buf->size += cb + 1;
    return res;
}


char
*buf_strdup(buf_t *buf, const char *str)
{
    char *res;
    int len = strlen(str);
    res = buf_alloc(buf, len);
    if(NULL != res) {
        strncpy(res, str, len);
        res[len] = 0;
    }
    return res;
}


char
*buf_strndup(buf_t *buf, const char *str, size_t len)
{
    char *res;

    res = buf_alloc(buf, len);
    if(NULL != res) {
        strncpy(res, str, len);
        res[len] = 0;
    }

    return res;
}


void
xml_context_init(xml_context_t *ctx, buf_t *tags, buf_t *strings)
{
    ctx->tag_closed = 0;
    ctx->tags = tags;
    ctx->strings = strings;
    ctx->root = NULL;
    ctx->last = NULL;
    ctx->level = 0;
}


void
xml_context_reset(xml_context_t *ctx)
{
    ctx->tag_closed = 0;
    buf_reset(ctx->tags);
    buf_reset(ctx->strings);
    ctx->root = NULL;
    ctx->last = NULL;
    ctx->level = 0;
}


xml_tag_t *
xml_tag(xml_context_t *ctx, const char *name, const char *content)
{
    xml_tag_t *tag;

    tag = (xml_tag_t *)buf_alloc(ctx->tags, sizeof(*tag));
    if(NULL != tag) {
        tag->name = buf_strdup(ctx->strings, name);
        tag->content = buf_strdup(ctx->strings, content);
        tag->parent = NULL;
        tag->next = NULL;
        tag->child = NULL;
    }

    return tag;
}


int
xml_is(xml_tag_t *tag, const char *name)
{
    if(NULL == tag || NULL == name)
        return 0;
    return 0 == strcasecmp(tag->name, name);
}


char *
xml_tag_string(xml_tag_t *tag)
{
    static char res[512];
    int len;

    len = snprintf(res, sizeof(res)-1, "<%s>%s</%s>", 
        tag->name,
        (NULL==tag->content)? "": tag->content,
        tag->name
    );

    res[len] = 0;
    return res;
}


xml_tag_t *
xml_find_tag(xml_tag_t *tag, const char *name, int recursive)
{
    xml_tag_t *res;

    while(tag) {
        if(xml_is(tag, name))
            return tag;
        if(recursive && NULL != tag->child) {
            res = xml_find_tag(tag->child, name, recursive);
            if(NULL != res)
                return res;
        }
        tag = tag->next;
    }
}

