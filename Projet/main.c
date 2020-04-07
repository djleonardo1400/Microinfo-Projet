#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include <msgbus/messagebus.h>
#include <motors.h>
#include <sensors/proximity.h>
#include <sensors/imu.h>
#include <audio/microphone.h>
#include <i2c_bus.h>
#include <arm_math.h>
#include <motion.h>
#include <detection.h>

#define NB_SAMPLES_OFFSET     200

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

static void timer11_start(void){
    //General Purpose Timer configuration
    //timer 11 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt11cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD11, &gpt11cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD11, 0xFFFF);
}


int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();

    //starts the USB communication
    usb_start();

    //starts timer 11
    timer11_start();

    //inits the motors
    motors_init();

    imu_start();

    messagebus_init(&bus, &bus_lock, &bus_condvar);
    //messagebus_init(&bus, &prox_topic_lock, &prox_topic_condvar);

    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;

    proximity_start();

    motion_start();
    detection_start();

    while (1) {
    	//waits 1 second
    	chThdSleepMilliseconds(1000);
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
