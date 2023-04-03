/*
 * encoders.c
 *
 * Created: 2/1/2023 5:29:08 PM
 *  Author: tonat
 */ 

#include "encoders.h"
int back_left_counter = 0;
int back_right_counter = 0;
int front_left_counter = 0;
int front_right_counter = 0;

void encoders_init()
{
 if(gpio_get_pin_level(encoder1_B) == true){
	 
 }
 
}

void interrupt_front_left()
{
if(gpio_get_pin_level(encoder1_B))
{
	front_left_counter++;
}else{
	front_left_counter--;
}
}

void interrupt_front_right()
{
	if(gpio_get_pin_level(encoder2_B))
	{
		front_right_counter++;
		}else{
		front_right_counter--;
	}
}

void interrupt_back_left()
{
	if(gpio_get_pin_level(encoder3_B))
	{
		back_left_counter++;
		}else{
		back_left_counter--;
	}
}

void interrupt_back_right()
{
	if(gpio_get_pin_level(encoder4_B))
	{
		back_right_counter++;
		}else{
		back_right_counter--;
	}
}


int front_left_counts(){
	return front_left_counter;
}
int front_right_counts()
{
return front_right_counter;
}
int back_left_counts()
{
	return back_left_counter;
}
int back_right_counts()
{
	return back_right_counter;
}