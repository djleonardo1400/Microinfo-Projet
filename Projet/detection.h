/*
 * detection.h
 *
 *  Created on: 3 Apr 2020
 *      Author: Leonardo Panattoni / Demar Künzle
 */

#ifndef DETECTION_H_
#define DETECTION_H_

//creates detection thread
void detection_start(void);

//returns canAdvance which tells the robot if he can advance or not
//depending if there's an obstacle in front of it
bool get_canAdvance(void);

#endif /* DETECTION_H_ */
