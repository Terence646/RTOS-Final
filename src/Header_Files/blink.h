/***************************************************************************//**
 * @file
 * @brief Blink examples functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/


#include  <kernel/include/os.h>
#include "app.h"
#include  <bsp_os.h>
#include  "bsp.h"
#include <stdint.h>
#include <stdbool.h>
#include "em_emu.h"
#include  <common/include/common.h>
#include <stdlib.h>
#include "em_device.h"
#include "em_cmu.h"
#include "em_chip.h"
#include "em_core.h"
#include <stdio.h>
#include "glib.h"
#include "dmd.h"
#include <string.h>
#include "capsense.h"

#ifndef BLINK_H
#define BLINK_H

#include "gpio.h"
#include <string.h>

//Task Defines
#define TASK_STACK_SIZE         512
#define LCD_TASK_PRIO             20
#define PHYS_TASK_PRIO        21
#define  BUTTON_TASK_PRIO            22  /*   Button Task Priority.                 */
#define CAP_TASK_PRIO       23

//Game Defines
#define REFRESHRATE         60
#define LEFTBOUND           25
#define RIGHTBOUND          124
#define TOPBOUND          -50
#define BOTTOMBOUND         115

#define MAXFORCE          64
#define PLATFORMMASS        8


// Structs


typedef struct
{
  uint8_t center;
  uint8_t length;
  int x_velocity;
  int x_acceleration;
  uint8_t mass;
  /* data */
} platform_obj;


typedef struct
{
  /* data */
  int x_position;
  float y_position;
  double time_of_flight;
  double max_height;
  double max_distance;

} satchel_obj;

typedef struct
{
  /* data */
  int x_position;
  int y_position;
  int power;

} railgun_obj;


void LCD_init(void);
void game_init(void);
void loadingScreen(void);
void read_button1(void);
void read_button0(void);
void gameLoss(void);
void gameWin(void);
void railGunCalc(float power);


#define MAX_STR_LEN             48

/////////


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
//////////

#endif  // BLINK_H

