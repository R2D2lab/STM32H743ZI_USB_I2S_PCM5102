/*
 * audioplay.c
 *
 *  Created on: Feb 5, 2021
 *      Author: HOME
 */

#include "audioplay.h"
#include "usbd_audio_if.h"

//extern I2S_HandleTypeDef hi2s2;
//extern DMA_HandleTypeDef hdma_spi2_tx;

extern DMA_HandleTypeDef hdma_sai1_a;
extern SAI_HandleTypeDef hsai_BlockA1;

#define DMA_MAX_SZE                     0xFFFF
#define DMA_MAX(_X_)                (((_X_) <= DMA_MAX_SZE)? (_X_):DMA_MAX_SZE)
#define AUDIODATA_SIZE                  2   /* 16-bits audio data size */

void Audio_Player_Play(uint8_t* pBuffer, uint32_t Size)
{
	if(Size > 0xFFFF) {
		Size = 0xFFFF;
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
	HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*)pBuffer, Size);
//	HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*)pBuffer, DMA_MAX(Size/AUDIODATA_SIZE));
}

void Audio_Player_Pause(void)
{
//	HAL_I2S_DMAPause(&hi2s2);
	HAL_SAI_DMAPause(&hsai_BlockA1);
}

void Audio_Player_Resume(void)
{
//	HAL_I2S_DMAResume(&hi2s2);
	HAL_SAI_DMAResume(&hsai_BlockA1);
}

void Audio_Player_Stop(void)
{
//	HAL_I2S_DMAStop(&hi2s2);
	HAL_SAI_DMAStop(&hsai_BlockA1);
}

void Audio_Player_VolumeCtl(uint8_t vol)
{
//	WM8978_VolumeCtl(vol);
}

//void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s)
//{
//	HalfTransfer_CallBack_FS();
//}
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai_BlockA1)
{
	if(hsai_BlockA1->Instance==SAI1_Block_A)
	{
//		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		HalfTransfer_CallBack_FS();
	}
}

//void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
//{
//	TransferComplete_CallBack_FS();
//}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai_BlockA1)
{
	if(hsai_BlockA1->Instance==SAI1_Block_A)
	{
//		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		TransferComplete_CallBack_FS();
	}
}
