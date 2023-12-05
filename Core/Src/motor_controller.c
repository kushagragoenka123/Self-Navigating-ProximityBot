/*
 * motor_controller.c
 *
 *  Created on: Nov 8, 2023
 *      Author: kgoenka
 */
#include "main.h"

//based on pixy input, center the target by rotating wheels so that the object is at [127.5, y]
//assume that the robot is already moving
void center_robot(TIM_HandleTypeDef htim3, TIM_HandleTypeDef htim4, uint16_t x) {
	//to prevent the robot from speeding up constantly as it centers on the object
	extern int speed;
	static char turning_left = 0;
	static char turning_right = 0;

	if (x > 192 && !turning_right) { //turn right

		turning_right = 1;
		turning_left = 0;
		//int ccr = __HAL_TIM_GET_COMPARE(&htim3, TIM_CHANNEL_3);
		//set_throttle(htim3, htim4, 0);
		//HAL_Delay(1000);

		turn(htim3, htim4, 650, 550, 'R');
		//HAL_Delay(200);
		//turn(htim3, htim4, 600, 600, 'R');
	}

	else if (x < 112 && !turning_left) { //turn left

		turning_right = 0;
		turning_left = 1;
		//set_throttle(htim3, htim4, 0);
		//HAL_Delay(1000);
		//int ccr = __HAL_TIM_GET_COMPARE(&htim4, TIM_CHANNEL_2);

		turn(htim3, htim4, 550, 650, 'L');
		//HAL_Delay(200);
		//turn(htim3, htim4, 600, 600, 'L');

	}

	else if (x >= 112 && x <= 192 ){

		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1, GPIO_PIN_SET); //L
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);

		turning_right = 0;
		turning_left = 0;
		set_throttle(htim3, htim4, speed / 100);
	}

}

void turn(TIM_HandleTypeDef htim3, TIM_HandleTypeDef htim4, uint16_t l_speed, uint16_t r_speed, char direction) {
	if (direction == 'L') {
		 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); //R
		  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_0, GPIO_PIN_SET); //L

		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1, GPIO_PIN_SET); //L

		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, r_speed); //TODO: check if 50 increment is enough
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, l_speed);
	}

	else {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);

		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, l_speed); //TODO: check if 50 increment is enough
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, r_speed); //TODO: check if 50 increment is enough
	}
}

//ARR=1000; set CCR based on input from 1-10
void set_throttle(TIM_HandleTypeDef htim3, TIM_HandleTypeDef htim4, int throttle) {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, throttle*100); //L
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, throttle*100); //R

}

void collision_avoidance(TIM_HandleTypeDef htim3, TIM_HandleTypeDef htim4, float avg, int pixy_coord){
	//rotate 90 degrees
	extern int ePulse;
	extern int ePulse1;
	extern int ePulse2;
	extern int ePulse3;

		//move forward a few feet
		//rotate -90 degrees
		//repeat if obstacle is still present
		//return on success here
		turn(htim3, htim4, 800, 800, 'L');
		HAL_Delay(1300); // CHECK VAL

		set_throttle(htim3, htim4, 0);
		HAL_Delay(500);
		const float factor = 1/144.0f;
		const float limit = 30.0f;

		float a = (ePulse * factor > limit) ? limit : ePulse * factor;
		float a1 = (ePulse1 * factor > limit) ? limit : ePulse1 * factor;
		float a2 = (ePulse2 * factor > limit) ? limit : ePulse2 * factor;
		float a3 = (ePulse3 * factor > limit) ? limit : ePulse3 * factor;

		float avgLHS = (a2 * a) * 0.5f;
		float avgCenter = (a + a3) * 0.5f;
		float avgRHS = (a1+ a3) * 0.5f;
		float avg = (a1+ a3 + a + a2) * 0.25f;

		if ((avgLHS < 15) || (avgCenter < 15) ||(avgRHS < 15) || (avg < 15)) {
			HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1, GPIO_PIN_SET); //L
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);

			set_throttle(htim3, htim4, 7);
			HAL_Delay(850); //test striaght line

			set_throttle(htim3, htim4, 0);
			HAL_Delay(200);

			turn(htim3, htim4, 700, 700, 'R');
			HAL_Delay(1200); // CHECK VAL

			set_throttle(htim3, htim4, 0);
			HAL_Delay(500);

			a = (ePulse * factor > limit) ? limit : ePulse * factor;
			a1 = (ePulse1 * factor > limit) ? limit : ePulse1 * factor;
			a2 = (ePulse2 * factor > limit) ? limit : ePulse2 * factor;
			a3 = (ePulse3 * factor > limit) ? limit : ePulse3 * factor;

			avgLHS = (a2 * a) * 0.5f;
					  avgCenter = (a + a3) * 0.5f;
					  avgRHS = (a1+ a3) * 0.5f;
					  avg = (a1+ a3 + a + a2) * 0.25f;

			if ((avgLHS < 15) || (avgCenter < 15) ||(avgRHS < 15) || (avg < 15)) {
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1, GPIO_PIN_SET); //L
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);

				set_throttle(htim3, htim4, 7);
				HAL_Delay(1000); //check

				set_throttle(htim3, htim4, 0);
				HAL_Delay(500);

				find_target(pixy_coord, htim3, htim4);
			}

			else {
				find_target(pixy_coord, htim3, htim4);
			}

		}

		else {
			turn(htim3, htim4, 800, 800, 'L');
			HAL_Delay(2600); // CHECK VAL

			set_throttle(htim3, htim4, 0);
			HAL_Delay(500);


			a = (ePulse * factor > limit) ? limit : ePulse * factor;
			a1 = (ePulse1 * factor > limit) ? limit : ePulse1 * factor;
			a2 = (ePulse2 * factor > limit) ? limit : ePulse2 * factor;
			a3 = (ePulse3 * factor > limit) ? limit : ePulse3 * factor;

			avgLHS = (a2 * a) * 0.5f;
					  avgCenter = (a + a3) * 0.5f;
					  avgRHS = (a1+ a3) * 0.5f;
					  avg = (a1+ a3 + a + a2) * 0.25f;


			if ((avgLHS < 15) || (avgCenter < 15) ||(avgRHS < 15) || (avg < 15)) {
				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1, GPIO_PIN_SET); //L
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);

				set_throttle(htim3, htim4, 7);
				HAL_Delay(850); //check

				set_throttle(htim3, htim4, 0);
				HAL_Delay(200);

				turn(htim3, htim4, 800, 800, 'L');
				HAL_Delay(1300); // CHECK VAL

				set_throttle(htim3, htim4, 0);
				HAL_Delay(500);


				a = (ePulse * factor > limit) ? limit : ePulse * factor;
				a1 = (ePulse1 * factor > limit) ? limit : ePulse1 * factor;
				a2 = (ePulse2 * factor > limit) ? limit : ePulse2 * factor;
				a3 = (ePulse3 * factor > limit) ? limit : ePulse3 * factor;

				avgLHS = (a2 * a) * 0.5f;
						  avgCenter = (a + a3) * 0.5f;
						  avgRHS = (a1+ a3) * 0.5f;
						  avg = (a1+ a3 + a + a2) * 0.25f;

				if ((avgLHS < 15) || (avgCenter < 15) ||(avgRHS < 15) || (avg < 15)) {
					HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1, GPIO_PIN_SET); //L
					HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);

					set_throttle(htim3, htim4, 7);
					HAL_Delay(1000);

					set_throttle(htim3, htim4, 0);
					HAL_Delay(500);

					find_target(pixy_coord, htim3, htim4);
				}

				else {
					find_target(pixy_coord, htim3, htim4);
				}

			}

			else {
				turn(htim3, htim4, 750, 750, 'L');
				HAL_Delay(1300); // CHECK VAL

				HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1, GPIO_PIN_SET); //L
				HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);
				set_throttle(htim3, htim4, 0);
				return;

			}



		}

		return;

		//rotate -180 degrees otherwise and do while loop

}

void find_target(int x, TIM_HandleTypeDef htim3, TIM_HandleTypeDef htim4){
	extern UART_HandleTypeDef huart3;
	extern uint8_t recvBuf[32];
	extern uint8_t getBlocks[6];
	int pixy_coord = x;
	int x_val = x;
	while (pixy_coord == x_val){
		turn(htim3, htim4, 700, 700, 'R');

		HAL_UART_Transmit(&huart3, getBlocks, 6, 1000);
		HAL_UART_Receive(&huart3, recvBuf, 20,1000);

		 x_val = (((uint16_t)(recvBuf[9]) << 8) + recvBuf[8]);
	}

	HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1, GPIO_PIN_SET); //L
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET);

	set_throttle(htim3, htim4, 0);
	return;

}


// set threshold for ultrasonic sensor to cause interrupt
void set_distance_from_user(int distance) {}


// if object is out of frame rotate clockwise until its at [127.5, y]
void recalibrate_frame() {}
