/*
 * audioplay.h
 *
 *  Created on: Feb 5, 2021
 *      Author: HOME
 */

#ifndef INC_AUDIOPLAY_H_
#define INC_AUDIOPLAY_H_

#include "stm32h7xx_hal.h"
#include "usbd_audio_if.h"

void Audio_Player_Play(uint8_t* pBuffer, uint32_t Size);
void Audio_Player_Stop(void);

#endif /* INC_AUDIOPLAY_H_ */
