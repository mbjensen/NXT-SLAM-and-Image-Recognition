#ifndef SLAM
#define SLAM

#include "math.h"
#include <cstdlib>
#include "cfloat"
#ifdef _MSC_VER
   typedef __int64 int64_t; // Define it from MSVC's internal type
#else
   #include <stdint.h>      // Use the C99 official header
#endif

#include "slamDataStructures.h"
#include "communication.cpp"

namespace medina {
    using namespace ecrobot;

    class Slam {
        Communication*  comm;
        Clock* clock;

        // Struct "ts_position_t" has double x, y in mm and double theta in degrees as fields
        // (Math symbol: x where x_{t} is the robots position at time t. x = (x, y, theta))
        ts_position_t startpos, position2;

        bool firstReading;
        int robotLengthMoved;

        // Manage robot position.
        double thetarad;

        ts_state_t state;
        ts_map_t map;
        ts_sensor_data_t sensor_data;

    public:
        int tooClose;

        Slam(Communication* _comm, Clock* _clock) {
            comm    = _comm;
            clock   = _clock;

            // Sets the robots position in the map first time (x_{t} = (x, y, theta))
            startpos.x = 0.5 * TS_MAP_SIZE / TS_MAP_SCALE;
            startpos.y = 0.5 * TS_MAP_SIZE / TS_MAP_SCALE;
            startpos.theta = 0;

            firstReading = true;
            tooClose = 0;

            ts_map_init(&map);
            ts_state_init(&state, &map, &startpos, 10.0, 8.0);
        }

        void ts_map_init(ts_map_t *map) {
            int x, y, initval;
            ts_map_pixel_t *ptr;
            initval = (TS_OBSTACLE + TS_NO_OBSTACLE) / 2;

            for (ptr = map->map, y = 0; y < TS_MAP_SIZE; y++) {
                for (x = 0; x < TS_MAP_SIZE; x++, ptr++) {
                    *ptr = initval;
                }
            }
        }

        void ts_state_init(ts_state_t *state, ts_map_t *map, ts_position_t *position,
            double sigma_xy, double sigma_theta) {

            ts_random_init(&state->randomizer, 0xdead);
            state->map = map;
            state->position = *position;
            state->timestamp = 0; // Indicating start
            state->sigma_xy = sigma_xy;
            state->sigma_theta = sigma_theta;
        }

        void begin(unsigned short *data, int compassData, int centimetersMoved) {

            sensor_data.timestamp = clock->now();

            for (int i = 0; i < KINECT_DOTS; i++) {
                sensor_data.kinectOutput[i] = data[i];
            }

	    sensor_data.position.theta = (double)compassData;
            robotLengthMoved = centimetersMoved;

            newSlamStart();
        }

        void newSlamStart() {
            ts_scan_t scanData;

            // Manage robot position
            if (!firstReading) {
                thetarad = (sensor_data.position.theta * M_PI) / 180;

                position2 = state.position;

                // lengthToObject multiplied with 10 to get mm instead of cm
                position2.x += (robotLengthMoved * 10) * cos(thetarad);
                position2.y += (robotLengthMoved * 10) * sin(thetarad);

				position2.theta = sensor_data.position.theta;

                comm->sendLocation(position2.x, position2.y, position2.theta);
            } else {
                position2 = state.position;
				position2.theta = sensor_data.position.theta;
                thetarad = state.position.theta * M_PI / 180;
                firstReading = false;
            }

            build_scan(&sensor_data, &scanData, &state, 3);
			build_scan(&sensor_data, &state.scan, &state, 1);

            // Monte Carlo search
            position2.x += OFFSET_LASER * cos(thetarad);
            position2.y += OFFSET_LASER * sin(thetarad);
            sensor_data.position = position2 = monte_carlo_search(&state.randomizer, &state.scan, state.map, &position2, state.sigma_xy, state.sigma_theta, SAMPLE_RATE);

            sensor_data.position.x -= OFFSET_LASER * cos(position2.theta * M_PI / 180);
            sensor_data.position.y -= OFFSET_LASER * sin(position2.theta * M_PI / 180);

            // Map update
            ts_map_update(&scanData, state.map, &position2, 50);

            int mapLength = TS_MAP_SIZE * TS_MAP_SIZE;
            comm->sendBytes(mapLength, state.map->map);

            // Gets the length to the nearest object infront of MEDINA
            tooClose = 0;
            int pointsTooClose = 0;

            for (int p = 0; p < KINECT_DOTS; p++){
                if (sensor_data.kinectOutput[p] < 1600){
                    pointsTooClose++;
                }
            }

            if (pointsTooClose >= 15){
                tooClose = 1;
            }

            // Prepare next step
            state.position = sensor_data.position;
            state.timestamp = sensor_data.timestamp;
        }

        void build_scan(ts_sensor_data_t *sd, ts_scan_t *scan, ts_state_t *state, int span) {
            int i, j;
            double angle_rad, angle_deg;

            scan->nb_points = 0;

            // Span the laser scans to better cover the space
            for (i = 0; i < KINECT_DOTS; i++) {
                for (j = 0; j != span; j++) {
                    angle_deg = ANGLE_MIN + ((double)(i * span + j)) * (ANGLE_MAX - ANGLE_MIN) / (KINECT_DOTS * span - 1);

                    angle_rad = angle_deg * M_PI / 180;

                    if (sd->kinectOutput[i] > HOLE_WIDTH / 2 && sd->kinectOutput[i] <= DISTANCE_NO_DETECTION) {
                        scan->x[scan->nb_points] = sd->kinectOutput[i] * cos(angle_rad);
                        scan->y[scan->nb_points] = sd->kinectOutput[i] * sin(angle_rad);
                        scan->value[scan->nb_points] = TS_OBSTACLE;
                        scan->x[scan->nb_points] += OFFSET_LASER;
                        scan->nb_points++;
                    }
                }
            }
        }

        ts_position_t monte_carlo_search(ts_randomizer_t *randomizer, ts_scan_t *scan, ts_map_t *map,
                ts_position_t *start_pos, double sigma_xy, double sigma_theta, int stop) {

            ts_position_t currentpos, bestpos, lastbestpos;

            int currentdist;
            int bestdist, lastbestdist;
            int counter = 0;

            currentpos = bestpos = lastbestpos = *start_pos;
            currentdist = ts_distance_scan_to_map(scan, map, &currentpos);

            bestdist = lastbestdist = currentdist;

            do {
                currentpos = lastbestpos;
                currentpos.x = ts_random_normal(randomizer, currentpos.x, sigma_xy);
                currentpos.y = ts_random_normal(randomizer, currentpos.y, sigma_xy);
                currentpos.theta = ts_random_normal(randomizer, currentpos.theta, sigma_theta);

                currentdist = ts_distance_scan_to_map(scan, map, &currentpos);

                if (currentdist < bestdist) {
                    bestdist = currentdist;
                    bestpos = currentpos;
                } else {
                    counter++;
                }

                if (counter > stop / 3) {
                    if (bestdist < lastbestdist) {
                        lastbestpos = bestpos;
                        lastbestdist = bestdist;
                        counter = 0;
                        sigma_xy *= 0.5;
                        sigma_theta *= 0.5;
                    }
                }

            } while (counter < stop);

            return bestpos;
        }

        int ts_distance_scan_to_map(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos) {
            double c, s;
            int i, x, y, nb_points = 0;
            int64_t sum;

            c = cos(pos->theta * M_PI / 180);
            s = sin(pos->theta * M_PI / 180);

            // Translate and rotate scan to robot position
            // and compute the distance
            for (i = 0, sum = 0; i != scan->nb_points; i++) {
                if (scan->value[i] != TS_NO_OBSTACLE) {
                    x = (int)floor((pos->x + c * scan->x[i] - s * scan->y[i]) * TS_MAP_SCALE + 0.5);
                    y = (int)floor((pos->y + s * scan->x[i] + c * scan->y[i]) * TS_MAP_SCALE + 0.5);

                    // Check boundaries
                    if (x >= 0 && x < TS_MAP_SIZE && y >= 0 && y < TS_MAP_SIZE) {
                        sum += map->map[y * TS_MAP_SIZE + x];
                        nb_points++;
                    }
                }
            }

            if (nb_points) {
                sum = sum * 1024 / nb_points;
            } else {
                sum = 2000000000;
            }

            return (int)sum;
        }

        void updateMapWithObject(unsigned short *data) {
            int depth, angleB, objectX1, objectY1, radius, whichSideOfTheRobot;
            ts_map_pixel_t *ptr;

            // To get the x1 and y1 coordinate in mm instead of cm
            angleB = data[1];
            depth = data[0] * 10;

            // data[2] gives the radius of the object in cm. Converts it to map resolution
            radius = data[2] * (TS_MAP_SCALE * 10);
            whichSideOfTheRobot = data[3];

            // determines if the object is to the left or right side of the robot
            if (whichSideOfTheRobot == 0) {
                angleB = -angleB;
            }

            angleB = (angleB * M_PI) / 180;

            thetarad = (state.position.theta * M_PI) / 180;
            objectX1 = (int)(state.position.x + (depth * (cos(thetarad + angleB))));
            objectY1 = (int)(state.position.y + (depth * (sin(thetarad + angleB))));

            int xIndex = coordToIndex(objectX1);
            int yIndex = coordToIndex(objectY1);

            state.map->map[mapTo1D(xIndex, yIndex)] = 0;

            for (int i = 0; i < radius; i++) {
                state.map->map[mapTo1D(xIndex + 1, yIndex)] = 0;
                state.map->map[mapTo1D(xIndex - 1, yIndex)] = 0;
            }
        }

        int mapTo1D(int x, int y) {
            return y * TS_MAP_SIZE + x;
        }

        int coordToIndex(double coord) {
            double max = TS_MAP_SIZE / TS_MAP_SCALE;

            return (int)(TS_MAP_SIZE * coord / max);
        }

        void ts_map_update(ts_scan_t *scan, ts_map_t *map, ts_position_t *pos,
                int quality) {

            double c, s;
            double x2p, y2p;
            int i, x1, y1, x2, y2, xp, yp, value, q;
            double add, dist;

            c = cos(pos->theta * M_PI / 180);
            s = sin(pos->theta * M_PI / 180);
            x1 = (int)floor(pos->x * TS_MAP_SCALE + 0.5);
            y1 = (int)floor(pos->y * TS_MAP_SCALE + 0.5);

            // Translate and rotate scan to robot position
            for (i = 0; i != scan->nb_points; i++) {
                x2p = c * scan->x[i] - s * scan->y[i];
                y2p = s * scan->x[i] + c * scan->y[i];
                xp = (int)floor((pos->x + x2p) * TS_MAP_SCALE + 0.5);
                yp = (int)floor((pos->y + y2p) * TS_MAP_SCALE + 0.5);
                dist = sqrt(x2p * x2p + y2p * y2p);
                add = HOLE_WIDTH / 2 / dist;
                x2p *= TS_MAP_SCALE * (1 + add);
                y2p *= TS_MAP_SCALE * (1 + add);
                x2 = (int)floor(pos->x * TS_MAP_SCALE + x2p + 0.5);
                y2 = (int)floor(pos->y * TS_MAP_SCALE + y2p + 0.5);

                if (scan->value[i] == TS_NO_OBSTACLE) {
                    q = quality / 4;
                    value = TS_NO_OBSTACLE;
                } else {
                    q = quality;
                    value = TS_OBSTACLE;
                }

                ts_map_laser_ray(map, x1, y1, x2, y2, xp, yp, value, q);
            }
        }

        void ts_map_laser_ray(ts_map_t *map, int x1, int y1, int x2, int y2,
                int xp, int yp, int value, int alpha) {

            int x2c, y2c, dx, dy, dxc, dyc, error, errorv, derrorv, x;
            int incv, sincv, incerrorv, incptrx, incptry, pixval, horiz, diago;
            ts_map_pixel_t *ptr;

            if (x1 < 0 || x1 >= TS_MAP_SIZE || y1 < 0 || y1 >= TS_MAP_SIZE)
                return; // Robot is out of map

            x2c = x2; y2c = y2;
            // Clipping
            if (x2c < 0) {
                if (x2c == x1) return;
                y2c += (y2c - y1) * (-x2c) / (x2c - x1);
                x2c = 0;
            }
            if (x2c >= TS_MAP_SIZE) {
                if (x1 == x2c) return;
                y2c += (y2c - y1) * (TS_MAP_SIZE - 1 - x2c) / (x2c - x1);
                x2c = TS_MAP_SIZE - 1;
            }
            if (y2c < 0) {
                if (y1 == y2c) return;
                x2c += (x1 - x2c) * (-y2c) / (y1 - y2c);
                y2c = 0;
            }
            if (y2c >= TS_MAP_SIZE) {
                if (y1 == y2c) return;
                x2c += (x1 - x2c) * (TS_MAP_SIZE - 1 - y2c) / (y1 - y2c);
                y2c = TS_MAP_SIZE - 1;
            }

            dx = abs(x2 - x1); dy = abs(y2 - y1);
            dxc = abs(x2c - x1); dyc = abs(y2c - y1);
            incptrx = (x2 > x1) ? 1 : -1;
            incptry = (y2 > y1) ? TS_MAP_SIZE : -TS_MAP_SIZE;
            sincv = (value > TS_NO_OBSTACLE) ? 1 : -1;

            if (dx > dy) {
                derrorv = abs(xp - x2);
            } else {
                SWAP(dx, dy); SWAP(dxc, dyc); SWAP(incptrx, incptry);
                derrorv = abs(yp - y2);
            }

            error = 2 * dyc - dxc;
            horiz = 2 * dyc;
            diago = 2 * (dyc - dxc);
            errorv = derrorv / 2;
            incv = (value - TS_NO_OBSTACLE) / derrorv;
            incerrorv = value - TS_NO_OBSTACLE - derrorv * incv;
            ptr = map->map + y1 * TS_MAP_SIZE + x1;

            pixval = TS_NO_OBSTACLE;

            for (x = 0; x <= dxc; x++, ptr += incptrx) {
                if (x > dx - 2 * derrorv) {
                    if (x <= dx - derrorv) {
                        pixval += incv;
                        errorv += incerrorv;
                        if (errorv > derrorv) {
                            pixval += sincv;
                            errorv -= derrorv;
                        }
                    } else {
                        pixval -= incv;
                        errorv -= incerrorv;
                        if (errorv < 0) {
                            pixval -= sincv;
                            errorv += derrorv;
                        }
                    }
                }
                // Integration into the map
                if (*ptr != 0){ // is a circle pinned?
                    *ptr = ((256 - alpha) * (*ptr) + alpha * pixval) >> 8;
                }

                if (error > 0) {
                    ptr += incptry;
                    error += diago;
                } else error += horiz;
            }
        }

    private:
        static unsigned long SHR3(ts_randomizer_t *d) {
            d->jz   = d->jsr;
            d->jsr ^= (d->jsr << 13);
            d->jsr ^= (d->jsr >> 17);
            d->jsr ^= (d->jsr << 5);

            return d->jz + d->jsr;
        }

        static double UNI(ts_randomizer_t *d) {
            return .5 + (signed)SHR3(d) * .2328306e-9;
        }

        void ts_random_init(ts_randomizer_t *d, unsigned long jsrseed) {
            const double m1 = 2147483648.0;

            double dn = 3.442619855899, tn = dn, vn = 9.91256303526217e-3, q;
            int i;
            d->jsr = jsrseed;

            // Set up tables for Normal
            q           = vn / exp(-.5 * dn * dn);
            d->kn[0]    = (int)((dn / q) * m1);
            d->kn[1]    = 0;
            d->wn[0]    = q / m1; d->wnt[0] = q;
            d->wn[127]  = dn / m1; d->wnt[127] = dn;
            d->fn[0]    = 1.;
            d->fn[127]  = exp(-.5 * dn * dn);

            for(i = 126; i >= 1; i--) {
                dn              = sqrt(-2. * log(vn / dn + exp(-.5 * dn * dn)));
                d->kn[i + 1]    = (int)((dn / tn) * m1); tn=dn;
                d->fn[i]        = exp(-.5 * dn * dn);
                d->wn[i]        = dn / m1; d->wnt[i] = dn;
            }
        }

        double ts_random_normal_fix(ts_randomizer_t *d) {
            const double r = 3.442620;  // The starting of the right tail
            static double x, y;

            for(;;) {
                x = d->hz * d->wn[d->iz];
                if(d->iz == 0) { // iz==0, handle the base strip
                    do {
                        x = -log(UNI(d)) * 0.2904764;
                        // .2904764 is 1/r
                        y = -log(UNI(d));
                    } while(y + y < x * x);

                    return (d->hz > 0) ? r + x : -r - x;
                }

                // iz>0, handle the wedges of other strips
                if(d->fn[d->iz] + UNI(d) * (d->fn[d->iz - 1] - d->fn[d->iz]) < exp(-.5 * x * x))
                    return x;
                // Start all over
                d->hz = SHR3(d);
                d->iz = d->hz&127;

                if((unsigned long)abs(d->hz) < d->kn[d->iz])
                    return (d->hz*d->wn[d->iz]);
            }
        }

        double ts_random_normal(ts_randomizer_t *d, double m, double s) {
            double x;
            d->hz = SHR3(d);
            d->iz = d->hz & 127;
            x= ((unsigned long)abs(d->hz) < d->kn[d->iz])? d->hz * d->wn[d->iz] : ts_random_normal_fix(d); // Generic version

            return x * s + m ;
        }
    };
}
#endif
