#pragma once
#ifndef DEFINES_HPP
#define DEFINES_HPP

/* Debug prints uncomment to print debug information
 */
#define DEBUG

#define c_len  10
#define v_len   3
#define b_len 255

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
#define OFFL -100 

/* X-offset of TOF sensor Right w.r.t. middle
 */
#define OFFR 100

#endif // DEFINES_HPP

