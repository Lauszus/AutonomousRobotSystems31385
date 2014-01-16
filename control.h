#ifndef _control_h_
#define _control_h_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "rhd.h"

#include "constants.h"

enum { mot_stop = 1, mot_move, mot_move_bwd, mot_follow_black, mot_follow_white, mot_follow_wall_left, mot_turn };
enum { ms_init, ms_fwd, ms_fwd_time, ms_bwd, ms_fwd_fixed, ms_fwd_cross_black, ms_fwd_cross_white, ms_fwd_ir_left, ms_fwd_ir_wall_left, ms_fwd_ir_wall_right, ms_follow_white, ms_follow_white_cross_black, ms_follow_black, ms_follow_black_gate_left, ms_follow_black_gate_left_right, ms_follow_black_box, ms_follow_black_cross, ms_follow_wall_left, ms_turn_left, ms_turn_right, ms_turn_around, ms_center_line_black, ms_center_line_black_left, ms_center_line_black_right, ms_ir_dist, ms_gate_left, ms_gate_left_right, ms_center_angle, ms_reset_state, ms_wait, ms_stop, ms_end };

typedef struct { // Input signals
  int left_enc, right_enc; // encoderticks
  // parameters
  double w; // wheel separation
  double cr,cl; // meters per encodertick
  // Output signals
  double right_pos, left_pos;
  // internal variables
  int left_enc_old, right_enc_old;
  // Difference between current pos and new measurement
  int left_delta_pos, right_delta_pos;
} odotype;

typedef struct { // Input
  int cmd;
  int curcmd;
  double speedcmd;
  double dist;
  double angle, startAngle;
  double left_pos, right_pos;
  // parameters
  double w;
  // output
  double motorspeed_l, motorspeed_r;
  int finished;
  // internal variables
  double startpos;
  double x0, y0; // Start coordinates
  double wall_dist; // Distance to wall
} motiontype;

typedef struct {
  int state, oldstate;
  int programState[100];
  double dist[100];
  double speed[100];
  double angle[100];
  int time;
} smtype;

typedef struct {
  int32_t value_raw[8];
  double value[8];
  uint8_t length;
  double lowest_val, lowest_val_left, lowest_val_right, highest_val, highest_val_left, highest_val_right;
  uint8_t lowest_pos, highest_pos;
  uint8_t black_line_found, white_line_found;
  double center_mass[2], center_mass_neighbors[2]; // First value is for black line and second value is for white line
} linesensortype;

typedef struct {
  int32_t value_raw[5];
  double value[5];
  double kA[5], kB[5]; // Calibration values
  uint8_t length;
  uint8_t ignoreObs; // Ignore obstacles
} irsensortype;

double getX();
double getXRaw();
double getY();
double getYRaw();
double getPhi();

void printState(int state);

void updateIRSensor(symTableElement *irsensor, irsensortype *p);
void printIRSensor(irsensortype *p);
#if CALIBRATE_IR_SENSOR_FRONT
uint8_t calibrateIRSensorFront(irsensortype *p);
#endif
#if CALIBRATE_IR_SENSOR_LEFT
uint8_t calibrateIRSensorLeft(irsensortype *p);
#endif
#if CALIBRATE_IR_SENSOR_RIGHT
uint8_t calibrateIRSensorRight(irsensortype *p);
#endif
void updateLineSensor(symTableElement *linesensor, linesensortype *p);
void printLineSensor(linesensortype *p);
#if CALIBRATE_LINE_SENSOR
uint8_t calibrateLineSensor(linesensortype *p);
#endif
void update_pos(odotype *p);
void reset_odo(odotype *p);
void update_odo(odotype *p);
void update_motcon(motiontype *p, linesensortype *line, irsensortype *ir);
int followBlackLine(motiontype *mot, double dist, double speed, int time);
int followWhiteLine(motiontype *mot, double dist, double speed, int time);
int followWallLeft(motiontype *mot, double dist, double speed, double wall_dist, int time);
int fwd(motiontype *mot, double dist, double speed, int time);
int bwd(motiontype *mot, double dist, double speed, int time);
int turn(motiontype *mot, double angle, double speed, int time);
void sm_update(smtype *p);

#endif
