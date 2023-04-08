/*
 * gpio.h
 *
 *  Created on: Sep 7, 2021
 *      Author: Nick
 */

#ifndef SRC_HEADER_FILES_GPIO_H_
#define SRC_HEADER_FILES_GPIO_H_

//***********************************************************************************
// Include files
//***********************************************************************************
#include "em_gpio.h"
#include "bspconfig.h"

//***********************************************************************************
// defined files
//***********************************************************************************

// LED 0 pin is
#define LED0_port   gpioPortF
#define LED0_pin    4u
#define LED0_default  false   // Default false (0) = off, true (1) = on
// LED 1 pin is
#define LED1_port   gpioPortF
#define LED1_pin    5u
#define LED1_default  false // Default false (0) = off, true (1) = on
// BUTTON 0 is
#define BUTTON0_port gpioPortF
#define BUTTON0_pin  6u
#define BUTTON0_default false // Default false (0) = not pressed, true (1) = pressed
// BUTTON 1 is
#define BUTTON1_port gpioPortF
#define BUTTON1_pin  7u
#define BUTTON1_default false // Default false (0) = not pressed, true (1) = pressed
// CAPSENSE Channel 0 is
#define CSEN0_port gpioPortC
#define CSEN0_pin  0u
#define CSEN0_default false
// CAPSENSE Channel 1 is
#define CSEN1_port gpioPortC
#define CSEN1_pin  1u
#define CSEN1_default false
// CAPSENSE Channel 2 is
#define CSEN2_port gpioPortC
#define CSEN2_pin  2u
#define CSEN2_default false
// CAPSENSE Channel 3 is
#define CSEN3_port gpioPortC
#define CSEN3_pin  3u
#define CSEN3_default false


//***********************************************************************************
// global variables
//***********************************************************************************


//***********************************************************************************
// function prototypes
//***********************************************************************************
void gpio_open(void);


#endif /* SRC_HEADER_FILES_GPIO_H_ */
