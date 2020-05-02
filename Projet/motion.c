/*
 * motion.c
 *
 *  Created on: 2 Apr 2020
 *      Author: Leonardo Panattoni / Demar Künzle
 */
#include <stdlib.h>
#include <motors.h>
#include <sensors/imu.h>
#include <detection.h>
#include <arm_math.h>
//#include <chprintf.h>
#include <leds.h>
#include <audio/audio_thread.h>
#include <audio/play_melody.h>
#include <melodies.h>

#define NB_AXIS 3
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define THRESHOLD_ACC 500
#define THRESHOLD_STOP 1000
#define MAX_INCLINATION 2600
#define FACTOR 10000
#define MAX_ROTATION_SPEED 500
#define MAX_ADVANCE_SPEED 750
#define CYCLES_TO_DESTUCK_180 10
#define MAX_ANGLE_PROPORTIONAL_R_SPEED 0.68
#define ROTATION_CONST 735

//motion thread declaration
static THD_WORKING_AREA(waMotion, 256);
static THD_FUNCTION(Motion, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int16_t acc_offsets[NB_AXIS];
    int16_t acc_true[NB_AXIS];
    int16_t advance_speed = 0;
    int16_t rot_speedL = 0;
    int16_t rot_speedR = 0;
    int16_t let_rotate = 0;
    float inclination = 0;
    float angle;

    //calibration for accelerations offsets, 20 samples
    acc_offsets[X_AXIS] = get_acc_filtered(X_AXIS, 20);
    acc_offsets[Y_AXIS] = get_acc_filtered(Y_AXIS, 20);
    acc_offsets[Z_AXIS] = get_acc_filtered(Z_AXIS, 20);

    //dac_start();
    //playMelody(MARIO_START,0,NULL);
    playMelody(EXTERNAL_SONG,ML_SIMPLE_PLAY,get_xp_boot());
    //dac_stop();

    while(1){

    		time = chVTGetSystemTime();

    		//actual accelerations, filtered for abrupt acceleration changes and noise
    		acc_true[X_AXIS] = get_acc_filtered(X_AXIS, 10) - acc_offsets[X_AXIS];
    		acc_true[Y_AXIS] = get_acc_filtered(Y_AXIS, 10) - acc_offsets[Y_AXIS];


    		//angle formed between X and Y axis, used for alignment.
    		angle = atan(((float) acc_true[X_AXIS])/acc_true[Y_AXIS]);

    		//rotation speed attribution
    		if(fabs(angle)<MAX_ANGLE_PROPORTIONAL_R_SPEED && acc_true[Y_AXIS] < -THRESHOLD_ACC){
    			rot_speedR = -ROTATION_CONST*angle;
    			rot_speedL = ROTATION_CONST*angle;
    		}else if(acc_true[X_AXIS]< 0){
    			rot_speedR = -MAX_ROTATION_SPEED;
    			rot_speedL = MAX_ROTATION_SPEED;
    		}else{
    			rot_speedR = MAX_ROTATION_SPEED;
    			rot_speedL = -MAX_ROTATION_SPEED;
    		}

    		//Destucking when new direction is changed to 180°.
    		//Solves a problem that made the robot sometimes "shake" because of X axis value that changed sign very often near 0 value (180°).
    		//We let the robot rotate anti-clockwise for 10 cycles to get it out of that problematic zone and let the other conditions handle back when done.
    		if(acc_true[Y_AXIS] >THRESHOLD_ACC && acc_true[X_AXIS]>-THRESHOLD_ACC && acc_true[X_AXIS]<THRESHOLD_ACC){
    			let_rotate = CYCLES_TO_DESTUCK_180;
    		}
    		if(let_rotate>0){
    			let_rotate--;
    			rot_speedR = MAX_ROTATION_SPEED;
    			rot_speedL = -MAX_ROTATION_SPEED;
    		}

    		//STOP condition, when on leveled surface.
    		if(acc_true[X_AXIS]< THRESHOLD_STOP && acc_true[X_AXIS]> -THRESHOLD_STOP && acc_true[Y_AXIS]< THRESHOLD_STOP && acc_true[Y_AXIS]> -THRESHOLD_STOP){
    			rot_speedR = 0;
    			rot_speedL = 0;
    			advance_speed = 0;

    		//move control when no obstacles in front
    		}else if(get_canAdvance()){

    			//inclination calculation: Y axis over Z axis
    			inclination = (FACTOR*abs(acc_true[Y_AXIS]))/abs(acc_offsets[Z_AXIS]);
    			if(inclination > MAX_INCLINATION) inclination = MAX_INCLINATION;

    			//determination of advance_speed as function of inclination and alignment (angle)
    			//advance_speed higher for higher inclination and higher when more aligned (smaller angle)
    			if(acc_true[Y_AXIS] > THRESHOLD_ACC){
    				advance_speed=0;
    			}
    			else if(fabs(angle)<MAX_ANGLE_PROPORTIONAL_R_SPEED && abs(acc_true[X_AXIS]) > THRESHOLD_ACC){
    				advance_speed = MAX_ADVANCE_SPEED*((MAX_ANGLE_PROPORTIONAL_R_SPEED - fabs(angle))/MAX_ANGLE_PROPORTIONAL_R_SPEED)*inclination/MAX_INCLINATION;
    			}else if(abs(acc_true[X_AXIS])< THRESHOLD_ACC){
    				advance_speed = MAX_ADVANCE_SPEED*((MAX_ANGLE_PROPORTIONAL_R_SPEED - fabs(angle))/MAX_ANGLE_PROPORTIONAL_R_SPEED)*inclination/MAX_INCLINATION;

    				//rotation do not occur when aligned
    				rot_speedR = 0;
    				rot_speedL = 0;
    			}

    			//only rotation when angle is too big
    			if(fabs(angle)>MAX_ANGLE_PROPORTIONAL_R_SPEED){
    				advance_speed = 0;
    			}
    		}
    		//check if there's an obstacle in front of robot. Only rotation allowed if so.
    		if(!get_canAdvance()){
    			advance_speed = 0;
    		}
    		//set motors speed
    		right_motor_set_speed(advance_speed + rot_speedR);
    		left_motor_set_speed(advance_speed + rot_speedL);

    		//Waits 10ms (100Hz)
    		chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void motion_start(void){
	chThdCreateStatic(waMotion, sizeof(waMotion), NORMALPRIO, Motion, NULL);
}
