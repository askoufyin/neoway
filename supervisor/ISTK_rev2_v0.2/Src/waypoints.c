#include <stdlib.h>
#include <math.h>


#define EQUATOR_LENGTH  40075.696   // Km
#define GROW_CONST      100         // Waypoint buffer grows each time on this count


typedef struct _waypoint {
    float lng;      // Longitude
    float lat;      // Lattitude
    float x;        // Distance from Greenwich meridian in Km
    float y;        // Distance from equator
} waypoint_t;


static waypoint_t *_waypoints;
static unsigned int _wpcount;
static unsigned int _wpalloc;
static float _distance;


void
wp_init()
{
    _waypoints = NULL;
    _wpcount = 0;
    _wpalloc = 0;
    _distance = 0.0;
}


void
wp_cleanup()
{
    free(_waypoints);
    wp_init();
}


int
wp_add(float lng, float lat)
{
    waypoint_t *temp;
    float eql, dist;
    float dx, dy;

    if(_wpcount == _wpalloc) {
        temp = (waypoint_t *)realloc(_waypoints, _wpalloc + GROW_CONST);
        if(NULL == temp) {
            return 0;
        }
        _waypoints = temp;
        _wpalloc += GROW_CONST;
    }

    _waypoints[_wpcount].lng = lng;
    _waypoints[_wpcount].lat = lat;

    /* Converting latitude to 0...1 range, because we need to calculate
     * equator length on this degree
     */
    eql = EQUATOR_LENGTH * cosf(fabsf(lng)/90.0);

    /* Now calculate distance from Greenwich meridian
     */
    dist = eql * (lat/180.0);

    _waypoints[_wpcount].x = dist;
    _waypoints[_wpcount].y = EQUATOR_LENGTH * (lng/90.0);

    if(_wpcount > 0) {
        /* Update total accumulated distance
         */
        dx = _waypoints[_wpcount].x - _waypoints[_wpcount-1].x;
        dy = _waypoints[_wpcount].y - _waypoints[_wpcount-1].y;
        _distance += sqrtf(dx*dx + dy*dy);
    }

    ++_wpcount;
}


float
wp_getdistance()
{
    return _distance;
}

