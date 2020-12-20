/*
 * interrupt_callbacks.h
 *
 *  Created on: 2020. dec. 20.
 *      Author: Norbert
 */

#ifndef INC_INTERRUPT_CALLBACKS_H_
#define INC_INTERRUPT_CALLBACKS_H_

extern short flag_10ms, flag_200ms, flag_1s, flag_10s;

void TIM3_callback();
void TIM4_callback();
void TIM6_callback();
void TIM7_callback();

#endif /* INC_INTERRUPT_CALLBACKS_H_ */
