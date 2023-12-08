/*
 * mecanum.h
 *
 *  Created on: Sep 12, 2023
 *      Author: Tommaso
 */

#ifndef INC_MECANUM_H_
#define INC_MECANUM_H_

#include <stm32f4xx_hal.h>

typedef struct {
	GPIO_TypeDef *dir_pin_1_port;
	uint16_t dir_pin_1;
	GPIO_TypeDef *dir_pin_2_port;
	uint16_t dir_pin_2;
	TIM_HandleTypeDef *timer;
	uint16_t channel;
	uint32_t timer_reload;
} motor_t;

/**
 * @brief Update the timer_reload value to the current one
 * @retval None
 */
void motor_init(motor_t* motor);

/**
 * @brief Stops the motor
 * @retval None
 */
void motor_stop(motor_t* motor);

/**
 * @brief Runs the motor at a power between -1.0 and 1.0
 * @param power: power to run the motor at
 * @retval None
 */
void motor_run(motor_t* motor, float power);

typedef struct {
	motor_t* fl_motor;
	motor_t* fr_motor;
	motor_t* bl_motor;
	motor_t* br_motor;
} four_wheeled_robot_t;

void mecanum_robot_init(four_wheeled_robot_t *mecanum_robot);

void mecanum_robot_stop(four_wheeled_robot_t *mecanum_robot);

/**
 * @brief Move the mecanum robot at given parameters
 */
void mecanum_robot_move(four_wheeled_robot_t *mecanum_robot, float power, float angle, float angular_speed);

#endif /* INC_MECANUM_H_ */
