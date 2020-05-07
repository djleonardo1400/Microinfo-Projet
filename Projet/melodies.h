/*
 * melodies.h
 *
 *  Created on: 2 May 2020
 *      Author: Leonardo Panattoni / Demar Künzle
 */

#ifndef MELODIES_H_
#define MELODIES_H_

//returns the necessary data (melody,tempo and length) of FFVII melody
melody_t* get_ffv_win(void);

//returns the necessary data (melody,tempo and length) of Windows XP boot sound
melody_t* get_xp_boot(void);

#endif /* MELODIES_H_ */
