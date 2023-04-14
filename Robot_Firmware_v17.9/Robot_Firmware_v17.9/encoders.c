/*
 * encoders.c
 *
 * Created: 2/1/2023 5:29:08 PM
 *  Author: tonat
 */ 

#include "encoders.h"
long double back_left_counter = 0;
long double back_right_counter = 0;
long double front_left_counter = 0;
long double front_right_counter = 0;

//attach interrupts to pin (rising edge)
void encoders_init()
{
 ext_irq_register(encoder1_A, interrupt_back_left);
 ext_irq_register(encoder2_A, interrupt_front_left);
 ext_irq_register(Encoder3_A, interrupt_back_right);
 ext_irq_register(Encoder4_A, interrupt_front_right);
}

void interrupt_front_left()
{
	if(gpio_get_pin_level(encoder2_B))
	{
		front_left_counter++;
	}
    else front_left_counter--;
}

void interrupt_front_right()
{
    if(gpio_get_pin_level(encoder4_B))
    {
        front_right_counter++;
        }
        else{
        front_right_counter--;
    }
}

void interrupt_back_left()
{
    if(gpio_get_pin_level(encoder1_B))
    {
        back_left_counter++;
        }
        else{
        back_left_counter--;
    }
}

void interrupt_back_right()
{
    if(gpio_get_pin_level(encoder3_B))
    {
        back_right_counter++;
        }
        else{
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