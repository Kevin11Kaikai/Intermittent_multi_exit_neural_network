/*
 * cam_main.c
 *
 *  Created on: Feb 1, 2023
 *      Author: wuyaw
 */
#include "stdio.h"

void mp_hal_delay_ms(uint32_t Delay)
{
    HAL_Delay(Delay);
}

uint32_t mp_hal_ticks_ms(void)
{
    return HAL_GetTick();
}
