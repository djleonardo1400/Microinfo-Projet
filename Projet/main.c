/*
 * main.c
 *
 *  Created on: 2 Apr 2020
 *      Author: Leonardo Panattoni / Demar Künzle
 */
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
#include <arm_math.h>
#include <motion.h>
#include <detection.h>

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

// MAIN FUNCTION
int main(void)
{
    halInit();
    chSysInit();
    mpu_init();

    //starts the serial communication
    serial_start();

    //starts the USB communication
    usb_start();

    //init the motors
    motors_init();

    //init the IMU
    imu_start();

    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //init the IR sensors
    proximity_start();

    //init motion and detection processes
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
