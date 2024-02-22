#include "core.h"
#include "FreeRTOS.h"
#include "task.h"

void task( void *args ){

	while (1){
		//printf("1000\r\n");
		servo_position_set(move_servo_get(SERVO_LEFT), 5.0f, 500.0f, 0.6f);
		servo_position_set(move_servo_get(SERVO_RIGHT), 5.0f, 500.0f, 0.6f);
		//servo_rpm_set(move_servo_get(SERVO_LEFT), 700.0f, 1.0f);
		//servo_rpm_set(move_servo_get(SERVO_RIGHT), 700.0f, 1.0f);
		delay(3000);
		//..delay(3000);
		servo_position_set(move_servo_get(SERVO_LEFT), 0.0f, 500.0f, 0.6f);
		servo_position_set(move_servo_get(SERVO_RIGHT), 0.0f, 500.0f, 0.6f);
		//servo_rpm_set(move_servo_get(SERVO_LEFT), -700.0f, 1.0f);
		//servo_rpm_set(move_servo_get(SERVO_RIGHT), -700.0f, 1.0f);
		//move_servos_permit(false);
		delay(3000);
		//move_servos_permit(true);
	}
}

void core_init( void ){
	move_servos_permit(true);
	//move_set_power(0.0, 0.0);
	shell_log("[move] init ok");
	//collision_attach(collision_event);
	for(uint16_t i = 0; i < 5; i++){
		collision_sensor_set_comparator(i, 150, 100);
	}
	//servo_mode_set(move_servo_get(SERVO_LEFT), SERVO_MODE_SPEED_FEEDBACK);
	//servo_mode_set(move_servo_get(SERVO_RIGHT), SERVO_MODE_SPEED_FEEDBACK);
	servo_mode_set(move_servo_get(SERVO_LEFT), SERVO_MODE_POSITION_FEEDBACK);
	servo_mode_set(move_servo_get(SERVO_RIGHT), SERVO_MODE_POSITION_FEEDBACK);
	xTaskCreate(task, "fsdafsd", 500, NULL, 4400, NULL);
}



void core_loop( void ){
	
	//servo_mode_set(move_servo_get(SERVO_LEFT), SERVO_MODE_NO_FEEDBACK);
	//servo_power_set(move_servo_get(SERVO_LEFT), 0.5);

	//printf("%.3f\r\n", servo_position_get(move_servo_get(SERVO_LEFT)));
	//printf("%.3f\r\n", servo_rpm_get(move_servo_get(SERVO_LEFT)));
	//int32_t pos_l = move_encoders_get(ENCODER_BACK_LEFT);
	//int32_t pos_r = move_encoder_get_steps(ENCODER_BACK_RIGHT);
	//float speed = move_encoder_get_rpm(ENCODER_BACK_RIGHT);
	//move_set_power(i, i);
	//printf("%d\t%f", pos_r, speed);
	//printf("123\r\n");
	delay(5);
	
	/*
	while(collision_sensor_get_state(4) == COLLISION_EVENT_DISENGAGE){
		move_set_speed(0.4, 0.4);
	}
	move_set_speed(-0.4, -0.4);
	delay(500);
	move_set_speed(0.4, -0.4);
	delay(500);
*/

	/*
	for(uint32_t i = 0; i < 5; i++){
		printf("%d\t", collision_sensor_get_value(i));
	}
	printf("\r\n");
	delay(1000);
	*/
}


