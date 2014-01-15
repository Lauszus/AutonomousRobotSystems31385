#ifndef _constants_h_
#define _constants_h_

#define IR_LIN_DIST 0.06 /* Distance between IR sensors and line sensor */
#define START_CROSS_DIST 1.225 /* Distance from start to the first cross */

#define IR_GATE_DIST 0.40 /* Distance from robot to gate on the loose */
#define GATE_DIST 0.55 /* Distance from robot to gate on the loose */

#define DIST_CAL ((300.0/285.0)) /* *(300/298.5)) */ /* Absolute distance constant */
#define kV 2.0 /* Constant in angular controller */
#define kTurn 10.0 /* Turning limiting constant */
#define Kfl 0.314245 /* Conversion from ticks to m for left encoder */
#define Kfr 0.314141 /* Conversion from ticks to m for right encoder */

/* PD values for line following algorithm */
#define kPfollow 0.28 /* 0.4 - 2.4 */
#define kDfollow 0.6 /* 0.05 */

/* IR distance sensor constants */
#define kA(i) (i == 0 ? 15.7642 : \
	       i == 1 ? 14.5691 : \
	       i == 2 ? 14.6813 : \
	       i == 3 ? 13.5049 : \
	       i == 4 ? 15.5932 : \
	       0 )

#define kB(i) (i == 0 ? 71.9962 : \
	       i == 1 ? 88.3791 : \
	       i == 2 ? 82.9057 : \
	       i == 3 ? 73.8117 : \
	       i == 4 ? 53.2018 : \
	       0 )

#define IR_MIN_VALUE(i) (i == 0 ? 95 : \
                         i == 1 ? 109 : \
                         i == 2 ? 104 : \
                         i == 3 ? 94 : \
                         i == 4 ? 75 : \
                         0 )

/* Constants for the line sensor */
#define BLACK_VALUE 53.067500
#define WHITE_VALUE 91.021250

#define BLACK_LINE_FOUND_VALUE 0.2 /* If all the line sensors are below this value, then the robot must have crossed a line */

/*****************************************
* odometry
*/
#define WHEEL_DIAMETER   0.067 /* m */
#define ED 1.0037 /* Ed */
#define ED2 1.0014 /* Seconds calibration */
#define WHEEL_DIAMETER_L ((WHEEL_DIAMETER * (1 + (1 - ED) / 2))) /* * (1 + (1 - ED2) / 2)) */
#define WHEEL_DIAMETER_R ((WHEEL_DIAMETER * (1 - (1 - ED) / 2))) /* * (1 - (1 - ED2) / 2)) */
#define WHEEL_SEPARATION 0.2761 /* 0.2765 m */
#define DELTA_M_L (M_PI * WHEEL_DIAMETER_L / 2000)
#define DELTA_M_R (M_PI * WHEEL_DIAMETER_R / 2000)
#define ROBOTPORT 24902

#endif