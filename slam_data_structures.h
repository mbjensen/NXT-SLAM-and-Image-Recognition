#ifndef SLAM_DATA_STRUCTURES_H
#define SLAM_DATA_STRUCTURES_H

/**** Used in slam.cpp ****/
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SWAP(x, y) (x ^= y ^= x ^= y)

#define KINECT_DOTS 30
#define MIN_DIST 500
#define ANGLE_MIN -30 // start angle for scan
#define ANGLE_MAX +30 // end angle for scan
#define OFFSET_LASER 0 // position of the laser wrt center of rotation

#define SAMPLE_RATE 200

#define HOLE_WIDTH 200
#define DISTANCE_NO_DETECTION 4500 // default value when the laser returns 0
/**** Used in slam.cpp ****/

#define TS_SCAN_SIZE 200 // number of points per scan
#define TS_MAP_SIZE 150 //2048
#define TS_MAP_SCALE 0.02
#define TS_NO_OBSTACLE 255
#define TS_OBSTACLE 1

typedef unsigned char ts_map_pixel_t;

typedef struct {
    ts_map_pixel_t map[TS_MAP_SIZE * TS_MAP_SIZE];
} ts_map_t;

typedef struct {
    double x[TS_SCAN_SIZE], y[TS_SCAN_SIZE];
    int value[TS_SCAN_SIZE];
    int nb_points;
} ts_scan_t;

typedef struct {
    double x, y;    // in mm
    double theta;   // in degrees
} ts_position_t;

typedef struct {
    unsigned int timestamp;
    ts_position_t position;
    int kinectOutput[TS_SCAN_SIZE];
    ts_scan_t scan;
} ts_sensor_data_t;

// Stochastic part
typedef struct {
    unsigned long jz;
    unsigned long jsr;
    long hz;
    unsigned long iz;
    unsigned long kn[128];
    double wnt[128];
    double wn[128];
    double fn[128];
} ts_randomizer_t;

typedef struct {
    ts_randomizer_t randomizer;
    ts_map_t *map;
    ts_position_t position;
    unsigned int timestamp;
    ts_scan_t scan;
    double sigma_xy;
    double sigma_theta;
} ts_state_t;

#endif // SLAM_DATA_STRUCTURES_H
