/*
 * detection.c
 *
 *  Created on: 3 Apr 2020
 *      Author: leonardopanattoni
 */

#include <stdlib.h>
#include <motors.h>
#include <sensors/proximity.h>
#include <sensors/imu.h>
#include <usbcfg.h>
#include <leds.h>
#include <chprintf.h>
#include <i2c_bus.h>

#define IR_TRESHOLD 200
#define NB_LEDS 8

static bool canAdvance = true;

void led_play(int ir_val[]);

static THD_WORKING_AREA(waDetection, 256);
static THD_FUNCTION(Detection, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int ir_values[PROXIMITY_NB_CHANNELS];

    calibrate_ir();

    while(1){

    	time = chVTGetSystemTime();

    	for(int i=0;i<PROXIMITY_NB_CHANNELS;i++){
    		ir_values[i] = get_calibrated_prox(i);
    	}
    	chprintf((BaseSequentialStream *)&SD3, "IR1=%d IR2=%d IR3=%d IR4=%d IR5=%d IR6=%d IR7=%d IR8=%d \r\n\n",
    			ir_values[0],ir_values[1],ir_values[2],ir_values[3],ir_values[4],ir_values[5],ir_values[6],ir_values[7]);

    	obstacle_in_front(ir_values);
    	led_play(ir_values);

    	chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void detection_start(void){
	chThdCreateStatic(waDetection, sizeof(waDetection), NORMALPRIO, Detection, NULL);
}

void led_play(int ir_val[]){

	if(ir_val[0]>= IR_TRESHOLD || ir_val[7]>= IR_TRESHOLD || ir_val[1]>= IR_TRESHOLD || ir_val[6]>= IR_TRESHOLD){
		set_led(LED1,1);
	} else set_led(LED1,0);

	if(ir_val[1]>= IR_TRESHOLD || ir_val[2]>= IR_TRESHOLD || ir_val[3]>= IR_TRESHOLD){
			set_led(LED3,1);
	} else set_led(LED3,0);

	if(ir_val[3]>= IR_TRESHOLD || ir_val[4]>= IR_TRESHOLD){
			set_led(LED5,1);
	} else set_led(LED5,0);

	if(ir_val[4]>= IR_TRESHOLD || ir_val[5]>= IR_TRESHOLD || ir_val[6]>= IR_TRESHOLD){
			set_led(LED7,1);
	} else set_led(LED7,0);

}

void obstacle_in_front(int ir_val[]){
	if(ir_val[0]>IR_TRESHOLD || ir_val[7]>IR_TRESHOLD ||ir_val[1]>IR_TRESHOLD ||ir_val[6]>IR_TRESHOLD){
		canAdvance = false;
	} else canAdvance = true;
}

bool get_canAdvance(void){
	return canAdvance;
}
