#pragma once
#ifndef DEFINES_HPP
#define DEFINES_HPP

/* Debug prints uncomment to print debug information
 */
// #define DEBUG_IMU
// #define DEBUG_TOF
#define DEBUG_OBS

/* Sensors
*/
enum SENSOR_SIDE { LEFT, RIGHT };

/* Shutdown pins for TOF sensor Left
 */
#define XSHUTL 8

/* Shutdown pins for TOF sensor Right
 */
#define XSHUTR 7

/* X-offset of TOF sensor Left w.r.t. middle
 */
#define OFFL -0.1

/* X-offset of TOF sensor Right w.r.t. middle
 */
#define OFFR 0.1

#endif // DEFINES_HPP

