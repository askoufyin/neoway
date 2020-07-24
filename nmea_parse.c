#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "nmea.h"


#define NMEA_MAX_FIELDS 32


const char *
nmea_system_name(nmea_sys_type_t type) {
    switch(type) {
    case NMEA_SYS_GLONASS:
        return "GLONASS";
    case NMEA_SYS_GPS:
        return "GPS";
    case NMEA_SYS_GLONASS_GPS:
        return "GLONASS+GPS";
    case NMEA_SYS_GALILEO:
        return "GALILEO";
    case NMEA_SYS_PROPRIETARY:
        return "PROPRIETARY";
    }
    return "UNKNOWN";
}


nmea_sys_type_t
nmea_system_type(const char *hdr)
{
    if('G'==hdr[0]) {
        if('L'==hdr[1])
            return NMEA_SYS_GLONASS;
        if('P'==hdr[1])
            return NMEA_SYS_GPS;
        if('N'==hdr[1])
            return NMEA_SYS_GLONASS_GPS;
        if('A'==hdr[1])
            return NMEA_SYS_GALILEO;
    }
    else if('P'==hdr[0]) {
        return NMEA_SYS_PROPRIETARY;
    }
    return NMEA_SYS_UNKNOWN;
}


/* This is local typedef, so we hide it from the outer world */
struct _sentence {
    char name[4];
    nmea_sentence_t id;
};


nmea_sentence_t
nmea_sentence_id(const char *hdr)
{
    static struct _sentence _s[] = {
        { "GLL", NMEA_GLL },
        { "GGA", NMEA_GGA },
        { "GNS", NMEA_GNS },
        { "GSA", NMEA_GSA },
        { "GSV", NMEA_GSV },
        { "RMC", NMEA_RMC },
        { "VTG", NMEA_VTG }
        /* Add new NMEA sentences here */
    };

    int i;

    for(i=0; i<sizeof(_s)/sizeof(_s[0]); ++i)
        if(hdr[2] == _s[i].name[0] && hdr[3] == _s[i].name[1] && hdr[4] == _s[i].name[2])
            return _s[i].id;

    return NMEA_UNKNOWN_SENTENCE;
}


static nmea_err_t
_parse_gsv(nmea_msg_t *out, char *fields[], int count)
{
    int i, n;
    int subgroup_number, group_size, num_sats;

    (void)count; // prevent compiler complains about unused parameter

    group_size = atoi(fields[1]);
    subgroup_number = atoi(fields[2]);
    num_sats = atoi(fields[3]);

    if(1 == subgroup_number) {
        out->gsv.count = 0;
        memset(out->gsv.satellites, 0, sizeof(out->gsv.satellites));
    }

    n = out->gsv.count;
    for(i=4; i+4<count; i+=4) {
        if('\0' != *fields[i])
            out->gsv.satellites[n].id = atoi(fields[i]);
        if('\0' != *fields[i+1])
            out->gsv.satellites[n].elevation = atoi(fields[i+1]);
        if('\0' != *fields[i+2])
            out->gsv.satellites[n].azimuth = (u_int16_t)atoi(fields[i+2]);
        if('\0' != *fields[i+3])
            out->gsv.satellites[n].snr = (unsigned char)atoi(fields[i+3]);
        if(0 != out->gsv.satellites[n].id)
            ++n;
    }

    out->gsv.count = n;

    /* set 'good to go' flag */
    out->gsv.complete = (subgroup_number == group_size);
    return NMEA_ERR_OK;
}


static nmea_err_t
_parse_gga(nmea_msg_t *out, char *fields[], int count)
{
    int i, n;
    int subgroup_number, group_size, num_sats;

    (void)count; // prevent compiler complains about unused parameter

    if('\0' == fields[1]) {
        out->gga.valid = 0;
        out->gga.latitude = 0;
        out->gga.lat_flag = 0;
        out->gga.longitude = 0;
        out->gga.long_flag = 0;
        out->gga.fix_type = NMEA_FIX_UNAVAILABLE;
        out->gga.sat_count = 0;
        out->gga.dilution = 0;
        out->gga.altitude = 0;
        out->gga.g_separation = 0;

        return NMEA_ERR_OK;
    }

    out->gga.valid = 1;
    out->gga.latitude = atof(fields[1]);
    out->gga.lat_flag = fields[2][0];
    out->gga.longitude = atof(fields[3]);
    out->gga.long_flag = fields[4][0];
    out->gga.fix_type = (nmea_fix_type_t)atoi(fields[5]);
    out->gga.sat_count = atoi(fields[6]);
    out->gga.dilution = atof(fields[7]);
    out->gga.altitude = atoi(fields[8]);
    out->gga.g_separation = 0;

    return NMEA_ERR_OK;
}


static nmea_err_t
_parse_gll(nmea_msg_t *out, char *fields[], int count)
{
    (void)count;

    out->gll.latitude = atof(fields[1]);
    out->gll.lat_flag = fields[2][0];
    out->gll.longitude = atof(fields[3]);
    out->gll.long_flag = fields[4][0];
    out->gll.valid = ('A' == fields[6][0]);

    return NMEA_ERR_OK;
}


nmea_err_t
nmea_parse(const char *msg, nmea_msg_t *out)
{
    int i, n, len;
    char buf[NMEA_MAX_MESSAGE_LENGTH+1];
    char *fields[NMEA_MAX_FIELDS];
    nmea_sys_type_t sys;
    nmea_sentence_t sent;

    if(NULL == out || NULL == msg)
        return NMEA_ERR_INVALID_PARAM; // Bad parameter passed

    len = strlen(msg);
    if(len > NMEA_MAX_MESSAGE_LENGTH)
        return NMEA_ERR_MESSAGE_TOO_LARGE; // Prevent buffer overruns

    if('$' != msg[0] /* || '\r' != msg[len-1] || '\n' != msg[len] */)
        return NMEA_ERR_NOT_A_MESSAGE; // Not a NMEA message

    strcpy(buf, msg+1); // skip '$'
    buf[len-2] = '\0';  // strip CR/LF


    fields[0] = buf;
    for(i=0, n=1; buf[i] != '\0'; ++i) {
        if(',' == buf[i] || '*' == buf[i]) { // Split string by field delimiters
            buf[i] = '\0'; // replace separator char with /0
            fields[n++] = &buf[i+1];
        }
    }

    fields[n] = NULL;

    sys = nmea_system_type(buf);
    if(NMEA_SYS_UNKNOWN == sys)
        return NMEA_ERR_UNKNOWN_SYSTEM;

    sent = nmea_sentence_id(buf);
    if(NMEA_UNKNOWN_SENTENCE == sent)
        return NMEA_ERR_UNKNOWN_SENTENCE;

    out->system = sys;
    out->type = sent;

    switch(sent) {
        case NMEA_GSV:
            return _parse_gsv(out, fields, n);
        case NMEA_GLL:
            return _parse_gll(out, fields, n);
        case NMEA_GGA:
            return _parse_gga(out, fields, n);
        case NMEA_GSA:
            return NMEA_ERR_OK;
        case NMEA_RMC:
            return NMEA_ERR_OK;
        case NMEA_VTG:
            return NMEA_ERR_OK;
        case NMEA_GNS:
            return NMEA_ERR_OK;
    }

    return NMEA_ERR_UNKNOWN_SENTENCE; // Something went wrong
}
