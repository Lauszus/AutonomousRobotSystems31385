/*
 * An example SMR program.
 *
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <sys/ioctl.h>
#include "rhd.h"

#include "constants.h"
#include "control.h"

#define LIMIT_ACC 1 /* Used to limit acceleration and velocity of the robot */
#define CALIBRATE_LINE_SENSOR 0 /* Set to 1 to start calibration of the linesensor */
#define CALIBRATE_IR_SENSOR_FRONT 0 /* Set to 1 to start calibration of the ir front sensors */
#define CALIBRATE_IR_SENSOR_LEFT 0 /* Set to 1 to start calibration of the ir left sensor */
#define CALIBRATE_IR_SENSOR_RIGHT 0 /* Set to 1 to start calibration of the ir right sensor */

#define AVOID_OBSTACLES_DIST 20 /* If any obstacle gets this close, then it will stop and wait until it is removed (in cm) - set to 0 to deactivate */

symTableElement *getinputref(const char *sym_name, symTableElement * tab) {
	int i;
	for (i=0; i< getSymbolTableSize('r'); i++)
		if (strcmp (tab[i].name, sym_name) == 0)
			return &tab[i];
	return 0;
}

symTableElement *getoutputref(const char *sym_name, symTableElement * tab) {
	int i;
	for (i=0; i< getSymbolTableSize('w'); i++)
		if (strcmp (tab[i].name, sym_name) == 0)
			return &tab[i];
	return 0;
}

/********************************************
* Motion control
*/

// SMR input/output data
symTableElement *inputtable, *outputtable;
symTableElement *encl,*encr, *linesensor, *irsensor, *speedl, *speedr, *resetmotorr, *resetmotorl, *tick;

linesensortype line;
irsensortype ir;
odotype odo;
smtype mission;
motiontype mot;

int32_t timer;
double V_old, omega_old;

int main() {
  int running;

  /* Establish connection to robot sensors and actuators. */
  if (rhdConnect('w',"localhost",ROBOTPORT)!='w'){
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }

  printf("Connected to robot\n");
  if ((inputtable = getSymbolTable('r')) == NULL){
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }
  if ((outputtable = getSymbolTable('w')) == NULL){
    printf("Can't connect to rhd \n");
    exit(EXIT_FAILURE);
  }

  // Connect to robot I/O variables
  encl = getinputref("encl", inputtable);
  encr = getinputref("encr", inputtable);
  linesensor = getinputref("linesensor", inputtable);
  irsensor = getinputref("irsensor", inputtable);
  tick = getinputref("tick", inputtable);

  speedl = getoutputref("speedl", outputtable);
  speedr = getoutputref("speedr", outputtable);
  resetmotorr = getoutputref("resetmotorr", outputtable);
  resetmotorl = getoutputref("resetmotorl", outputtable);

  *resetmotorr->data = *resetmotorl->data = 1;
  resetmotorr->updated = resetmotorl->updated = 1;

  /* Read sensors and zero our position. */
  rhdSync();

  odo.w = WHEEL_SEPARATION;
  odo.cl = DELTA_M_L;
  odo.cr = DELTA_M_R;
  reset_odo(&odo);
  //printf("position: %f, %f\n", odo.left_pos, odo.right_pos);

  line.length = linesensor->length;
  ir.length = irsensor->length - 1; // The last one is not used
  uint8_t i;
  for (i = 0; i < ir.length; i++) {
    ir.kA[i] = kA(i);
    ir.kB[i] = kB(i);
  }

  mot.w = odo.w;
  running = 1;
  mission.state = ms_init;
  mission.oldstate = -1;

  V_old = omega_old = 0;
  timer = *tick->data;

  *resetmotorr->data = *resetmotorl->data = 0;
  resetmotorr->updated = resetmotorl->updated = 1;

  /* Setup program states */
  for (i = 0; i < sizeof(mission.speed) / sizeof(mission.speed[0]); i++)
    mission.speed[i] = 0.3; // Set default speed to 0.3 m/s
  for (i = 0; i < sizeof(mission.dist) / sizeof(mission.dist[0]); i++)
    mission.dist[i] = 10; // The default value is just 10m, so we don't have to set it every time
  for (i = 0; i < sizeof(mission.angle) / sizeof(mission.angle[0]); i++)
    mission.angle[i] = M_PI/2.0; // Defaults to ±90 degress

  ir.ignoreObs = 1; // Ignore all obstacles by default
  uint8_t stateIndex = 0; // Reset state index

/************************** Program state **************************/
goto Start;

  /********* Distance measurement *********/
  mission.dist[stateIndex] = 1.0;
  mission.programState[stateIndex++] = ms_follow_black;
  mission.speed[stateIndex] = 0.15;
  mission.programState[stateIndex++] = ms_follow_black_cross;
  mission.programState[stateIndex++] = ms_stop;
  mission.programState[stateIndex++] = ms_wait;
  mission.speed[stateIndex] = 0.15;
  mission.programState[stateIndex++] = ms_follow_black_box;
  mission.programState[stateIndex++] = ms_stop;
  mission.programState[stateIndex++] = ms_wait;
  mission.programState[stateIndex++] = ms_ir_dist;

  /********* Box gate **********/
  /* Go to gate */
  mission.programState[stateIndex++] = ms_turn_around;
  mission.programState[stateIndex++] = ms_follow_black_cross;
  mission.programState[stateIndex++] = ms_fwd_fixed;
  mission.programState[stateIndex++] = ms_turn_right;

Start:
  mission.speed[stateIndex] = 0.25;
  mission.programState[stateIndex++] = ms_follow_black_cross;
  mission.speed[stateIndex] = 0.25;
  mission.programState[stateIndex++] = ms_fwd_fixed;
  mission.speed[stateIndex] = 0.25;
  mission.programState[stateIndex++] = ms_follow_black_cross;
  mission.programState[stateIndex++] = ms_fwd_fixed;
  mission.programState[stateIndex++] = ms_turn_right;

  /* Push box */
  mission.programState[stateIndex++] = ms_follow_black_cross;
  mission.dist[stateIndex] = 0.15;
  mission.programState[stateIndex++] = ms_fwd_time;
  mission.dist[stateIndex] = 1.0;
  mission.programState[stateIndex++] = ms_bwd;

  /* Go to gate */
  mission.programState[stateIndex++] = ms_turn_right;
  mission.programState[stateIndex++] = ms_fwd_cross_black;
  mission.programState[stateIndex++] = ms_fwd_fixed;
  mission.programState[stateIndex++] = ms_turn_left;
  mission.programState[stateIndex++] = ms_follow_black_cross;

  /* Go through gate */
  mission.programState[stateIndex++] = ms_fwd_fixed;
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_turn_left;
  mission.speed[stateIndex] = 0.20;

  mission.programState[stateIndex++] = ms_center_line_black;
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_follow_black_cross;
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_fwd_fixed;

  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_follow_black_cross;

  /********* Gate on the loose *********/
#if 1
  mission.dist[stateIndex] = 0.30;
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_fwd;

  mission.angle[stateIndex] = 40.0 * M_PI/180;
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_turn_right;
#else
  mission.speed[stateIndex] = 0.20;
  mission.dist[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_follow_black;
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_center_line_black;
#endif


  /* Look for gate */
#if 1
  /*mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_follow_black_cross;*/
  mission.speed[stateIndex] = 0.15;
  mission.programState[stateIndex++] = ms_follow_black_gate_left;
#else
  mission.dist[stateIndex] = 0.5;
  mission.speed[stateIndex] = 0.15;
  mission.programState[stateIndex++] = ms_fwd;
  mission.dist[stateIndex] = 2.0;
  mission.speed[stateIndex] = 0.15;
  mission.programState[stateIndex++] = ms_gate_left;
#endif
  mission.dist[stateIndex] = 0.10;
  mission.speed[stateIndex] = 0.15;
  mission.programState[stateIndex++] = ms_fwd;

  mission.dist[stateIndex] = GATE_DIST - 0.10;
  mission.speed[stateIndex] = 0.15;
#if 1
  mission.programState[stateIndex++] = ms_follow_black_gate_left;
#else
  mission.programState[stateIndex++] = ms_gate_left;
#endif

  /* Go through gate */
  mission.dist[stateIndex] = 0.02;
  mission.speed[stateIndex] = 0.15;
  mission.programState[stateIndex++] = ms_bwd;
  mission.speed[stateIndex] = 0.15;
  mission.programState[stateIndex++] = ms_turn_left;
  mission.speed[stateIndex] = 0.15;
  mission.programState[stateIndex++] = ms_gate_left_right;
  mission.dist[stateIndex] = 0.50;
  mission.speed[stateIndex] = 0.15;
  mission.programState[stateIndex++] = ms_fwd;

  /********* Wall *********/
  mission.programState[stateIndex++] = ms_turn_right;
  mission.programState[stateIndex++] = ms_fwd_cross_black;
  mission.programState[stateIndex++] = ms_fwd_fixed;
  mission.programState[stateIndex++] = ms_turn_left;

  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_follow_black_gate_left_right;
  mission.speed[stateIndex] = 0.20;
  mission.dist[stateIndex] = 0.50;
  mission.programState[stateIndex++] = ms_fwd;
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_turn_left;

  /* Follow wall */
  mission.dist[stateIndex] = 0.50;
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_fwd;
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_fwd_ir_wall_left;

  mission.programState[stateIndex++] = ms_stop;
  mission.programState[stateIndex++] = ms_wait;

  /* Gate found */
  mission.dist[stateIndex] = 0.40;
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_fwd;
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_turn_left;
  mission.dist[stateIndex] = 0.10;
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_bwd;
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_gate_left_right;
  mission.speed[stateIndex] = 0.20;
  mission.dist[stateIndex] = 0.80;
  mission.programState[stateIndex++] = ms_fwd;
  mission.speed[stateIndex] = 0.20;

  /* Go back to track */
  mission.programState[stateIndex++] = ms_turn_left;
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_fwd_cross_black;
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_fwd_fixed;
  mission.programState[stateIndex++] = ms_turn_left;
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_center_line_black;

  /* Goto white line */
#if 1
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_follow_black_cross;
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_fwd_fixed;
#endif
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_follow_black_cross;
  mission.dist[stateIndex] = 0.45;
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_fwd;
  mission.speed[stateIndex] = 0.20;
  mission.programState[stateIndex++] = ms_turn_left;

  /* Follow white line */
  mission.speed[stateIndex] = 0.25;
  mission.programState[stateIndex++] = ms_follow_white_cross_black;
  mission.dist[stateIndex] = 0.10;
  mission.speed[stateIndex] = 0.25;
  mission.programState[stateIndex++] = ms_fwd;
  mission.speed[stateIndex] = 0.25;
  mission.programState[stateIndex++] = ms_follow_black_cross;
  mission.speed[stateIndex] = 0.25;
  mission.programState[stateIndex++] = ms_fwd_fixed;
  mission.speed[stateIndex] = 0.25;
  mission.programState[stateIndex++] = ms_turn_right;
  mission.speed[stateIndex] = 0.25;
  mission.programState[stateIndex++] = ms_follow_black_box;

  /********* The End *********/
  mission.programState[stateIndex++] = ms_end;

/*******************************************************************/

  if (stateIndex >= sizeof(mission.programState) / sizeof(mission.programState[0])) {
    printf("Please increase the smtype struct array\n");
    exit(EXIT_FAILURE);
  }


  while (running) {
    rhdSync();
    odo.left_enc = *encl->data;
    odo.right_enc = *encr->data;
    update_odo(&odo);
    update_pos(&odo);

    updateLineSensor(linesensor, &line);
    //printLineSensor(&line);

    updateIRSensor(irsensor, &ir);
    //printIRSensor(&ir);

    /****************************************/
    /*        Mission statemachine          */
    /****************************************/
    sm_update(&mission);
    switch (mission.state) {
      case ms_init:
#if CALIBRATE_LINE_SENSOR
	if (calibrateLineSensor(&line)) running = 0;
#elif CALIBRATE_IR_SENSOR_FRONT
	if (calibrateIRSensorFront(&ir)) running = 0;
#elif CALIBRATE_IR_SENSOR_LEFT
	if (calibrateIRSensorLeft(&ir)) running = 0;
#elif CALIBRATE_IR_SENSOR_RIGHT
	if (calibrateIRSensorRight(&ir)) running = 0;
#else
      stateIndex = 0;
      mission.state = mission.programState[stateIndex];
#endif
      break;

      case ms_fwd:
        if (fwd(&mot, mission.dist[stateIndex], mission.speed[stateIndex], mission.time))
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_fwd_time:;
        static int32_t timer;
        if (mission.time == 0)
          timer = *tick->data;
        if (fwd(&mot, mission.dist[stateIndex], mission.speed[stateIndex], mission.time) || (*tick->data - timer > (mission.dist[stateIndex]/mission.speed[stateIndex]) * 300.0)) {
          if (*tick->data - timer > (mission.dist[stateIndex]/mission.speed[stateIndex]) * 300.0)
            printf("Aborting %d %f\n", *tick->data - timer, (mission.dist[stateIndex]/mission.speed[stateIndex]) * 300.0);
          mission.state = mission.programState[++stateIndex];
        }
        break;

      case ms_bwd:
        if (bwd(&mot, mission.dist[stateIndex], mission.speed[stateIndex], mission.time))
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_fwd_fixed: // This is used over and over again after we have found a cross
        if (fwd(&mot, 0.2, mission.speed[stateIndex], mission.time))
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_fwd_cross_white:
        if (fwd(&mot, mission.dist[stateIndex], mission.speed[stateIndex], mission.time)  || line.white_line_found)
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_fwd_cross_black:
        if (fwd(&mot, mission.dist[stateIndex], mission.speed[stateIndex], mission.time)  || line.black_line_found)
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_fwd_ir_wall_left:
        if (fwd(&mot, mission.dist[stateIndex], mission.speed[stateIndex], mission.time) || ir.value[0] > 0.50)
          mission.state = mission.programState[++stateIndex];
        break;

       case ms_follow_white:
        if (followWhiteLine(&mot, mission.dist[stateIndex], mission.speed[stateIndex], mission.time))
          mission.state = mission.programState[++stateIndex];
        break;

       case ms_follow_white_cross_black:
        if (followWhiteLine(&mot, mission.dist[stateIndex], mission.speed[stateIndex], mission.time) || line.black_line_found)
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_follow_black:
        if (followBlackLine(&mot, mission.dist[stateIndex], mission.speed[stateIndex], mission.time))
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_follow_black_box:
        if (followBlackLine(&mot, mission.dist[stateIndex], mission.speed[stateIndex], mission.time) || ir.value[2] < 0.20)
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_follow_black_cross:
        if (followBlackLine(&mot, mission.dist[stateIndex], mission.speed[stateIndex], mission.time) || line.black_line_found)
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_turn_left:
        if (turn(&mot, mission.angle[stateIndex], mission.speed[stateIndex], mission.time))
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_turn_right:
        if (turn(&mot, -mission.angle[stateIndex], mission.speed[stateIndex], mission.time))
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_turn_around:
        if (turn(&mot, M_PI, mission.speed[stateIndex], mission.time))
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_gate_left:
        if (fwd(&mot, mission.dist[stateIndex], mission.speed[stateIndex], mission.time))
          mission.programState[stateIndex] = ms_stop;
        else if (ir.value[0] < IR_GATE_DIST)
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_gate_left_right:
        if (fwd(&mot, mission.dist[stateIndex], mission.speed[stateIndex], mission.time))
          mission.programState[stateIndex] = ms_stop;
        else if (ir.value[0] < IR_GATE_DIST || ir.value[4] < IR_GATE_DIST) // Check if the left or right sensor is below the threshold
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_follow_black_gate_left:
        if (followBlackLine(&mot, mission.dist[stateIndex], mission.speed[stateIndex], mission.time))
          mission.programState[stateIndex] = ms_stop;
        else if (ir.value[0] < IR_GATE_DIST)
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_follow_black_gate_left_right:
        if (followBlackLine(&mot, mission.dist[stateIndex], mission.speed[stateIndex], mission.time))
          mission.programState[stateIndex] = ms_stop;
        else if (ir.value[0] < IR_GATE_DIST || ir.value[4] < IR_GATE_DIST) // Check if the left or right sensor is below the threshold
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_reset_state: // This is just used so the same state can be called twice
        mission.state = mission.programState[++stateIndex];
        break;

      case ms_center_line_black:
        if (line.center_mass_neighbors[0] < 4)
          mission.state = ms_center_line_black_right;
        else if (line.center_mass_neighbors[0] > 5)
          mission.state = ms_center_line_black_left;
        else // The line is already in the center
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_center_line_black_right:
        if (turn(&mot, -M_PI/2, mission.speed[stateIndex], mission.time))
          mission.state = ms_turn_left;
        else if (line.center_mass_neighbors[0] >= 4.5)
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_center_line_black_left:
        if (turn(&mot, M_PI/2, mission.speed[stateIndex], mission.time))
          mission.state = ms_turn_right;
        else if (line.center_mass_neighbors[0] <= 4.5)
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_ir_dist: {
        static double avg = 0;
        static uint8_t counter = 0, use_left = 0, use_right = 0;
        if (counter == 0 && ir.value[1] < 0.25)
          use_left = 1;
        if (counter == 0 && ir.value[3] < 0.25)
          use_right = 1;

        if (use_left == 1)
          avg += ir.value[1]; // Left sensor
        if (use_right == 1)
          avg += ir.value[3]; // Right sensor
        avg += ir.value[2]; // Center sensor

        if (++counter == 100) {
          if (use_left == 1 && use_right == 1)
            avg /= 300;
          else if (use_left == 1 || use_right == 1)
            avg /= 200;
          else
            avg /= 100;
          double x_new = (getXRaw() - mot.x0) / DIST_CAL;
          double y_new = (getYRaw() - mot.y0) / DIST_CAL;
          double dist = sqrt(x_new*x_new + y_new*y_new);
          printf("Avg: %f %d %d\n", avg, use_left, use_right);
          printf("Dist: %f %f %f\n", x_new, y_new, dist);
          printf("Total: %f\n", dist + avg + IR_LIN_DIST + START_CROSS_DIST);
          mission.state = mission.programState[++stateIndex];
        }
        break;
      }

      case ms_wall_left:
	break;

      case ms_wait:;
        static int32_t wait_timer;
        if (mission.time == 0)
          wait_timer = *tick->data;
        else if (*tick->data - wait_timer >= 100) // Wait 1s
          mission.state = mission.programState[++stateIndex];
        break;

      case ms_stop:
        mot.cmd = mot_stop;
        mission.state = mission.programState[++stateIndex];
        break;

      case ms_end:
        mot.cmd = mot_stop;
        running = 0;
        break;
    }
    /* End of mission  */
    mot.left_pos = odo.left_pos;
    mot.right_pos = odo.right_pos;
    update_motcon(&mot, &line);

#if LIMIT_ACC
    double omegal = Kfl * 100.0 * mot.motorspeed_l;
    double omegar = Kfr * 100.0 * mot.motorspeed_r;

    if (mot.curcmd == mot_move || mot.curcmd == mot_move_bwd) {
      double V = (omegar * WHEEL_DIAMETER_R + omegal * WHEEL_DIAMETER_L) / 4.0;
      double dtime = (double)(*tick->data - timer) / 100.0; // Tick increment every 10ms
      double acc = (V - V_old) / dtime;

      if (acc > 0.5) {
        acc = 0.5; // Limit acceleration
        V = acc * dtime + V_old; // acc = (V - V_old) / dtime
      } else if (acc < -0.5) {
        acc = -0.5; // Limit acceleration
        V = acc * dtime + V_old; // acc = (V - V_old) / dtime
      }

      double dx = getXRaw() - mot.x0;
      double dy = getYRaw() - mot.y0;
      double diff = mot.dist - sqrt(dx*dx + dy*dy);
      double Vmax = sqrt(fabs(2 * 0.5 * diff));
      if (V > Vmax)
        V = Vmax;
      else if (V < -Vmax)
        V = -Vmax;

      double dV = kV * (mot.startAngle - getPhi());

      omegal = (V - dV) / (WHEEL_DIAMETER_L / 2);
      omegar = (V + dV) / (WHEEL_DIAMETER_R / 2);

      V_old = V;
      omega_old = 0;
      //printf("Dist: %f, %f\n", mot.dist, diff);
      //printf("X: %f, Y: %f Phi: %f Acc: %f V: %f Vmax: %f dV: %f Diff: %f Omega: %f, %f\n", getX(), getY(), getPhi(), acc, V, Vmax, dV, diff, omegal, omegar);
   } else if (mot.curcmd == mot_turn) {
      double omega = (omegar * WHEEL_DIAMETER_R - omegal * WHEEL_DIAMETER_L) / (2 * WHEEL_SEPARATION); // This is actually angular velocity
      double dtime = (double)(*tick->data - timer) / 100.0; // Tick increment every 10ms
      double omegaAcc = (omega - omega_old) / dtime;

      if (omegaAcc > 0.5) {
        omegaAcc = 0.5; // Limit acceleration
        omega = omegaAcc * dtime + omega_old; // omegaAcc = (omega - omega_old) / dtime
      } else if (omegaAcc < -0.5) {
        omegaAcc = -0.5; // Limit acceleration
        omega = omegaAcc * dtime + omega_old; // omegaAcc = (omega - omega_old) / dtime
      }

      double diff = mot.angle - (getPhi() - mot.startAngle);
      double omegaMax = sqrt(fabs(kTurn * 0.5 * diff));
      if (omega > omegaMax)
        omega = omegaMax;
      else if (omega < -omegaMax)
        omega = -omegaMax;

      omegal = - omega * WHEEL_SEPARATION / WHEEL_DIAMETER_L;
      omegar = omega * WHEEL_SEPARATION / WHEEL_DIAMETER_R;

      omega_old = omega;
      V_old = 0;
      //printf("X: %f, Y: %f Phi: %f OmegaAcc: %f Omega: %f, %f, %f Diff: %f, omegaMax: %f\n", getX(), getY(), getPhi(), omegaAcc, omegal, omegar, omega, diff, omegaMax);
    }
    double outputl = omegal / Kfl;
    double outputr = omegar / Kfr;

    if (outputl < -128) outputl = -128;
    else if (outputl > 127) outputl = 127;
    if (outputr < -128) outputr = -128;
    else if (outputr > 127) outputr = 127;

    //printf("Output: %f, %f\n", outputl, outputr);

    uint8_t stop = 0;
#if AVOID_OBSTACLES_DIST
    if (ir.ignoreObs == 0) {
      uint8_t i;
      for (i = 0; i < ir.length - 2; i++) {
        if (ir.value[i + 1] < (double)AVOID_OBSTACLES_DIST / 100) {
	  printf("STOP %f\n", ir.value[i + 1]);
	  stop = 1;
	  break;
        }
      }
    }
#endif
    if (stop)
      *speedl->data = *speedr->data = 0;
    else {
      *speedl->data = outputl;
      *speedr->data = outputr;
    }
    speedl->updated = speedr->updated = 1;

    timer = *tick->data;
#else
    *speedl->data = 100.0 * mot.motorspeed_l;
    *speedr->data = 100.0 * mot.motorspeed_r;
    speedl->updated = speedr->updated = 1;
#endif

    //printf("X: %f, Y: %f Phi: %f\n", getX(), getY(), getPhi());

    /* Stop if keyboard is activated */
    int arg;
    ioctl(0, FIONREAD, &arg);
    if (arg != 0) running = 0;
  }/* End of main control loop */
  //writeOutput();

  *speedl->data = *speedr->data = 0;
  speedl->updated = speedr->updated = 1;

  rhdSync();
  rhdDisconnect();
  exit(0);
}
