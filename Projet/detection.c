/*
 * detection.c
 *
 *  Created on: 3 Apr 2020
 *      Author: Leonardo Panattoni / Demar Künzle
 */

#include <stdlib.h>
#include <motors.h>
#include <sensors/proximity.h>
#include <leds.h>
#include <chprintf.h>
#include <audio/audio_thread.h>
#include <audio/play_melody.h>

#define IR_TRESHOLD 200
static bool canAdvance = true;
enum senors {IR_1, IR_2, IR_3, IR_4, IR_5, IR_6, IR_7, IR_8};

void led_play(int ir_val[]);
void obstacle_in_front(int ir_val[]);

//detection thread declaration
static THD_WORKING_AREA(waDetection, 256);
static THD_FUNCTION(Detection, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;
    int ir_values[PROXIMITY_NB_CHANNELS];

    //performs a calibration of the IR sensors
    calibrate_ir();

    while(1){

    		time = chVTGetSystemTime();

    		//get the values of the sensors and place them in ir_values[]
    		for(int i=0;i<PROXIMITY_NB_CHANNELS;i++){
    			ir_values[i] = get_calibrated_prox(i);
    		}

    		obstacle_in_front(ir_values);
    		led_play(ir_values);

    		chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void detection_start(void){
	chThdCreateStatic(waDetection, sizeof(waDetection), NORMALPRIO, Detection, NULL);
}

//switch on desired LEDs for each IR sensor when robot is too close to obstacle:
//IR1: L1 | IR2: L1,L3 | IR3: L3 | IR4: L3,L5 | IR5: L5,L7 | IR6: L7 | IR7: L7,L1 | IR8: L1
void led_play(int ir_val[]){

	if(ir_val[IR_1]>= IR_TRESHOLD || ir_val[IR_8]>= IR_TRESHOLD || ir_val[IR_2]>= IR_TRESHOLD || ir_val[IR_7]>= IR_TRESHOLD){
		set_led(LED1,1);
	}else set_led(LED1,0);

	if(ir_val[IR_2]>= IR_TRESHOLD || ir_val[IR_3]>= IR_TRESHOLD || ir_val[IR_4]>= IR_TRESHOLD){
		set_led(LED3,1);
	}else set_led(LED3,0);

	if(ir_val[IR_4]>= IR_TRESHOLD || ir_val[IR_5]>= IR_TRESHOLD){
		set_led(LED5,1);
	}else set_led(LED5,0);

	if(ir_val[IR_5]>= IR_TRESHOLD || ir_val[IR_6]>= IR_TRESHOLD || ir_val[IR_7]>= IR_TRESHOLD){
		set_led(LED7,1);
	}else set_led(LED7,0);

}
//check if there's an obstacle in front of the robot. canAdvance is set adequately.
void obstacle_in_front(int ir_val[]){

	if(ir_val[IR_1]>IR_TRESHOLD || ir_val[IR_8]>IR_TRESHOLD ||ir_val[IR_2]>IR_TRESHOLD ||ir_val[IR_7]>IR_TRESHOLD){
		canAdvance = false;
	} else canAdvance = true;

	if(ir_val[IR_1]>IR_TRESHOLD && ir_val[IR_2]>IR_TRESHOLD && ir_val[IR_3]>IR_TRESHOLD && ir_val[IR_4]>IR_TRESHOLD
		&& ir_val[IR_5]>IR_TRESHOLD && ir_val[IR_6]>IR_TRESHOLD && ir_val[IR_7]>IR_TRESHOLD && ir_val[IR_8]>IR_TRESHOLD){

		 playMelody(5,0,NULL);
	}
}

bool get_canAdvance(void){
	return canAdvance;
}
