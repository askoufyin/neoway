#ifndef MAIN_H
#define MAIN_H


#define MAX_MESSAGE_LENGTH      8192

/* Number of records to keep in fixed circular buffer
 */
#define CIRCULAR_BUFFER_SIZE    100


typedef struct _location_rec {
    time_t timestamp;           // Unix time, when record arrived
    float latitude;             // GPS latitude, degrees
    float longitude;            // GPS longitude, degrees
    float altitude;             // GPS elevation above sea level, meters
    float speed;                // GPS speed
//    struct location_rec *next;  // Pointer to the next record in chain
}
location_rec_t;


#endif // MAIN_H
