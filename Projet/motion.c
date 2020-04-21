/*
 * motion.c
 *
 *  Created on: 2 Apr 2020
 *      Author: leonardopanattoni
 */
#include <stdlib.h>
#include <motors.h>
#include <sensors/proximity.h>
#include <sensors/imu.h>
#include <detection.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <i2c_bus.h>
#include <leds.h>

#define NB_AXIS 3
#define X_AXIS 0
#define Y_AXIS 1
#define Z_AXIS 2
#define THRESHOLD_ROTATION 500
#define MAX_INCLINATION 5236
#define FACTOR 10000
#define ROTATION_SPEED 300

static THD_WORKING_AREA(waMotion, 256);
static THD_FUNCTION(Motion, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    systime_t time;
    int16_t accelerations[NB_AXIS];
    int16_t acc_offsets[NB_AXIS];
    int16_t acc_true[NB_AXIS];
    int16_t acc_prec = 0;
    int16_t advance_speed = 0;
    int16_t inclination = 0;
    int16_t rot_speedL = 0;
    int16_t rot_speedR = 0;
    int16_t alined =0;

    acc_offsets[X_AXIS] = get_acc_filtered(X_AXIS, 20);
    acc_offsets[Y_AXIS] = get_acc_filtered(Y_AXIS, 20);
    acc_offsets[Z_AXIS] = get_acc_filtered(Z_AXIS, 20);

    while(1){

    	time = chVTGetSystemTime();
    	//get_acc_all(accelerations);
    	acc_true[X_AXIS] = get_acc_filtered(X_AXIS, 3) - acc_offsets[X_AXIS];
    	acc_true[Y_AXIS] = get_acc_filtered(Y_AXIS, 3) - acc_offsets[Y_AXIS];
    	acc_true[Z_AXIS] = get_acc_filtered(Z_AXIS, 3) - acc_offsets[Z_AXIS];

    //chprintf((BaseSequentialStream *)&SD3, "%Ax=%d Ay=%d Az=%d \r\n\n",
    //               acc_true[0], acc_true[1], acc_offsets[2]);


    if(acc_true[X_AXIS]> THRESHOLD_ROTATION && ((alined>0) || !get_canAdvance()) ){
    		alined--;
    		//rotate anticlockwise
    		rot_speedR = ROTATION_SPEED;
    		rot_speedL = -ROTATION_SPEED;
    		advance_speed = 0;

    } else if(acc_true[X_AXIS]< - THRESHOLD_ROTATION && ((alined>0) || !get_canAdvance()) ){
    		alined--;
		//rotate anticlockwise
		rot_speedR = -ROTATION_SPEED;
		rot_speedL = ROTATION_SPEED;
		advance_speed = 0;
    }

    else if(acc_true[X_AXIS]< THRESHOLD_ROTATION && acc_true[X_AXIS]> -THRESHOLD_ROTATION && acc_true[Y_AXIS]< THRESHOLD_ROTATION && acc_true[Y_AXIS]> -THRESHOLD_ROTATION){
    		rot_speedR = 0;
    	    rot_speedL = 0;
    	    advance_speed = 0;

    } else if(acc_true[X_AXIS]< acc_prec + THRESHOLD_ROTATION && acc_true[X_AXIS]> acc_prec -THRESHOLD_ROTATION && acc_true[Y_AXIS]< 0 && get_canAdvance()){

    		//inclination = (FACTOR*abs(acc_true[Y_AXIS]))/abs(acc_offsets[Z_AXIS]);
    		//chprintf((BaseSequentialStream *)&SD3, "Inclination = %d \r\n\n", inclination);
    	    	if(inclination > MAX_INCLINATION) inclination = MAX_INCLINATION;
    	    	alined =0;
    	    //advance_speed = MOTOR_SPEED_LIMIT*inclination/MAX_INCLINATION;
    	    	advance_speed = MOTOR_SPEED_LIMIT/2;
    	    	rot_speedR = 0;
    	    	rot_speedL = 0;
    	    	acc_prec = acc_true[X_AXIS];
    }
    if(!(acc_true[X_AXIS]< acc_prec +THRESHOLD_ROTATION && acc_true[X_AXIS]> acc_prec -THRESHOLD_ROTATION && acc_true[Y_AXIS]< 0)){
    	alined = 4;
    	acc_prec = 0;
    }

    if(!get_canAdvance()){
    	advance_speed = 0;
    }

    //set_led(LED5,alined);
    //set motors speed
    right_motor_set_speed(advance_speed + rot_speedR);
    left_motor_set_speed(advance_speed + rot_speedL);

    //chprintf((BaseSequentialStream *)&SD3, "Alined = %d \r\n\n", alined);

    //100Hz
    chThdSleepUntilWindowed(time, time + MS2ST(5));
    }
}


void motion_start(void){
	chThdCreateStatic(waMotion, sizeof(waMotion), NORMALPRIO+1, Motion, NULL);
}
