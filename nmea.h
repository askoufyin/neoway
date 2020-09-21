#ifndef NMEA_H
#define NMEA_H

// Stultōrum infinītus est numerus

/* We parse only limited set of NMEA messages and ignore others
 */
#define NMEA_MAX_MESSAGE_LENGTH         100 // 82 actually, including $ and CR/LF, but we give us some more space


typedef enum {
    NMEA_ERR_OK = 0,
    NMEA_ERR_INVALID_PARAM = -1,
    NMEA_ERR_BAD_CRC = -2,
    NMEA_ERR_NOT_A_MESSAGE = -3,
    NMEA_ERR_MESSAGE_TOO_LARGE = -4,
    NMEA_ERR_UNKNOWN_SYSTEM = -5,
    NMEA_ERR_UNKNOWN_SENTENCE = -6,
    NMEA_ERR_OTHER = -99
}
nmea_err_t;


typedef enum {
    NMEA_SYS_UNKNOWN = 0,
    NMEA_SYS_GPS,
    NMEA_SYS_GLONASS,
    NMEA_SYS_GLONASS_GPS,
    NMEA_SYS_GALILEO,
    NMEA_SYS_PROPRIETARY,
    POSITIONING_SYS_MAX
}
nmea_sys_type_t;


#define NMEA_MAX_SATELLITES_PER_SYSTEM  32


typedef struct _nmea_satellite_info {
    unsigned char id;           // ID of the satellite
    char elevation;             // -99 to 99 degrees
    unsigned short azimuth;     // 0 to 359 degrees
    unsigned char snr;          // Signal-to-Noise Ratio, dB (00-99)
}
nmea_satellite_info_t;


struct nmea_gll {
    char valid;                 // 1 = data valid, 0 = data is not valid
    float latitude;             // latitude, degrees
    char lat_flag;              // N = north, S = south
    float longitude;            // longitude, degrees
    char long_flag;             // E = east, W = west
};


struct nmea_gsv {
    char complete;              // 0 if more data expected (number of received GSV sentences less than batch size), 1 if all data available
    unsigned char count;        // Number of satellites currently visible
    nmea_satellite_info_t satellites[NMEA_MAX_SATELLITES_PER_SYSTEM]; // Satellite info
};


struct nmea_rmc {
    char valid;                 // 1 if message have 'A' in validity field, 0 otherwise
    float latitude;             // Lattitude, degrees
    char lat_flag;              // N = north, S = south
    float longitude;            // Longitude, degrees
    char long_flag;             // E = east, W = west
    float speed;                // Speed, knots
    float course;               // Course, degrees
    float variation;            // Magnetic variation, degrees
    char var_flag;              // E or W (E = east, W = west)
};


typedef enum {
    NMEA_FIX_UNAVAILABLE = 0,   // Fix not available
    NMEA_FIX_GPS = 1,           // GPS fix
    NMEA_FIX_GPS_DIFF = 2,      // Differential GPS fix (values above 2 are 2.3 features)
    NMEA_FIX_PPS = 3,           // PPS fix
    NMEA_FIX_KINEMATIC_RT = 4,  // Real Time Kinematic
    NMEA_FIX_FLOAT_RTK = 5,     // Float RTK
    NMEA_FIX_ESTIMATED = 6,     // Estimated (dead reckoning)
    NMEA_FIX_MANUAL_INPUT = 7,  // Manual input mode
    NMEA_FIX_SIMULATION = 8,    // Simulation mode
}
nmea_fix_type_t;


struct nmea_gga {
    char valid;                 // 1 if GGA sentence contains valid data
    float latitude;             // Latitude
    char lat_flag;              // [N]orth or [S]outh
    float longitude;            // Longitude
    char long_flag;             // [E]ast or [W]est
    nmea_fix_type_t fix_type;   // Fix type
    unsigned char sat_count;    // Number of satellites in use
    unsigned int dilution;      // Horizontal dilution (meters)
    float altitude;             // Antenna altitude above sea level (meters)
    float g_separation;         // Geoidal separation (meters)
};


typedef enum {
    NMEA_UNKNOWN_SENTENCE = 0,
    NMEA_GLL,                   // Geographic location: longitude and latitude
    NMEA_GSV,                   // Visible satellites info
    NMEA_GSA,
    NMEA_GGA,                   // Global Positioning System Fix Data
    NMEA_GNS,
    NMEA_RMC,                   // Recommended minimum specific GPS/Transit data
    NMEA_VTG
}
nmea_sentence_t;


typedef struct _nmea_msg {
    nmea_sentence_t type;
    nmea_sys_type_t system;
    union {
        struct nmea_gll gll;
        struct nmea_gsv gsv;
        struct nmea_gga gga;
        struct nmea_rmc rmc;
    };
} nmea_msg_t;


#ifdef __cplusplus
extern "C" {
#endif
extern const char *nmea_system_name(nmea_sys_type_t);
extern nmea_sys_type_t nmea_system_type(const char *);
extern nmea_sentence_t nmea_sentence_id(const char *);
extern nmea_err_t nmea_parse(const char *, nmea_msg_t *);
#ifdef __cplusplus
}
#endif


#endif // NMEA_H
