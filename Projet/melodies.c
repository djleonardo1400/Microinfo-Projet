/*
 * melodies.c
 *
 *  Created on: 2 May 2020
 *      Author: Leonardo Panattoni / Demar Künzle
 */

#include <audio/play_melody.h>

//FINAL FANTASY VII - WIN FANFARE melody
static const uint16_t ffv_win_melody[] = {
	NOTE_C5, NOTE_C5, NOTE_C5, NOTE_C5, NOTE_GS4, NOTE_AS4, NOTE_C5, 0, NOTE_AS4, NOTE_C5
};

static const float ffv_win_tempo[] = {
	15, 15, 15, 5, 5, 5, 15, 15, 15, 2
};

static const melody_t ffv_win = {
    .notes = ffv_win_melody,
    .tempo = ffv_win_tempo,
    .length = sizeof(ffv_win_melody)/sizeof(uint16_t),
  };

//WINDOWS XP BOOT SOUND
static const uint16_t xp_boot_melody[] = {
	NOTE_DS5, 0, NOTE_DS4, NOTE_AS4, 0, 0, NOTE_GS4, 0, 0, 0, NOTE_DS5, 0, NOTE_AS4
};

static const float xp_boot_tempo[] = {
	17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 4
};

static const melody_t xp_boot = {
    .notes = xp_boot_melody,
    .tempo = xp_boot_tempo,
    .length = sizeof(xp_boot_melody)/sizeof(uint16_t),
};

//returns the necessary data (melody,tempo and length) of FFVII melody
melody_t* get_ffv_win(void){
	melody_t* p = &ffv_win;
	return p;
}

//returns the necessary data (melody,tempo and length) of Windows XP boot sound
melody_t* get_xp_boot(void){
	melody_t* p = &xp_boot;
	return p;
}
