#ifndef __ADB_H_INCLUDED__
#define __ADB_H_INCLUDED__


#ifdef __cplusplus
extern "C" {
#endif
extern int adb_upload(const char*);
extern const char* adb_shell(const char*, const char*);
#ifdef __cplusplus
}
#endif


#endif

