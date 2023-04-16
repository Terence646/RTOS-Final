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

//#include "sl_simple_led.h"
//#include "sl_simple_led_instances.h"
#include "os.h"
#include "blink.h"
#include "glib.h"
#include "dmd.h"

/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/
static OS_TCB LCD_TCB;
static OS_TCB PHY_TCB;
static OS_TCB   Button_InputTaskTCB;                            /*   Task Control Block.   */


static CPU_STK LCD_stack[TASK_STACK_SIZE];
static CPU_STK PHY_stack[TASK_STACK_SIZE];
static CPU_STK  ButtonTaskStk[TASK_STACK_SIZE];             /*   Stack. */




static GLIB_Context_t glibContext;
static GLIB_Rectangle_t platform_context;

// Health Stats
static GLIB_Rectangle_t castle; // Outline
static GLIB_Rectangle_t caslteHealth; // Moveable Health bar

static GLIB_Rectangle_t player;
static GLIB_Rectangle_t playerHealth;

static GLIB_Rectangle_t energy;
static GLIB_Rectangle_t energyTotal;


platform_obj platform;

bool left_sense_hard, left_sense_soft, right_sense_hard,right_sense_soft;
bool      game_won      = false;
bool      game_loss     = false;
bool      fireRailgun   = false;
bool      shield        = false;
bool B0_Pressed = false;
bool B1_Pressed = false;
bool newGame = true;

int       active_Satchel = 0;

OS_TMR       PERIODIC_TMR;

OS_Q         App_MessageQ;
OS_FLAG_GRP  Flags;
OS_FLAG_GRP  LED_Flags;
OS_SEM       Semaphore;
OS_MUTEX     VS_Mutex;
OS_MUTEX     VD_Mutex;
OS_Q         Button_Q;

/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/
static void LCD_task(void *arg);
static void Platform_Physics_Task();
static void Button_InputTask (void  *p_arg);

/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/
/***************************************************************************//**
// * @brief Unified GPIO Interrupt handler (pushbuttons)
// *        PB0 Switches causes a fifo_write upon being pressed
// *        PB1 Switches causes a fifo_write upon being pressed
// *        Posts a flag depending on the pushbutton pressed
// *        posts a semaphore for Vehicle Speed task
// *****************************************************************************/
//void GPIO_Unified_IRQ(void)
//{
//  /* Get and clear all pending GPIO interrupts */
//  // CORE_DECLARE_IRQ_STATE;
//  // CORE_ENTER_ATOMIC();
//
//  uint32_t interruptMask = GPIO_IntGet();
//  GPIO_IntClear(interruptMask);
//  /* Act on interrupts */
//  //Checks for interrupt on the push buttons
//
//  if ((interruptMask & (1 << BSP_GPIO_PB1_PIN)) && !(interruptMask & (1 << BSP_GPIO_PB0_PIN))){
//      if(!shield) shield = true;
//      else shield = false;
//    // Shield Task
//  }
//  if ((interruptMask & (1 << BSP_GPIO_PB0_PIN)) && !(interruptMask & (1 << BSP_GPIO_PB1_PIN))) {
//   // Fire Railgun
//
//  }
//
//}
///***************************************************************************//**
// * @brief GPIO Interrupt handler for even pins
// ******************************************************************************/
//void GPIO_EVEN_IRQHandler(void)
//{
//  GPIO_Unified_IRQ();
//}
///***************************************************************************//**
// * @brief GPIO Interrupt handler for odd pins
// ******************************************************************************/
//void GPIO_ODD_IRQHandler(void)
//{
//  GPIO_Unified_IRQ();
//}

static void Button_InputTask(void  *p_arg){
    /* Use argument. */
   (void)&p_arg;
   RTOS_ERR error;
    while (DEF_TRUE) {
        /* All tasks should be written as infinite loops. */
        read_button0();
        read_button1();
        OSTimeDly(100,OS_OPT_TIME_DLY,&error);
    }
}
void read_button0(void){
  B0_Pressed = !GPIO_PinInGet(BUTTON0_port,BUTTON0_pin);
}

void read_button1(void){
  B1_Pressed = !GPIO_PinInGet(BUTTON1_port,BUTTON1_pin);
}

void loadingScreen(void)
{
  uint32_t status;
  /* Enable the memory lcd */
  status = sl_board_enable_display();
  EFM_ASSERT(status == SL_STATUS_OK);

  /* Initialize the DMD support for memory lcd display */
  status = DMD_init(0);
  EFM_ASSERT(status == DMD_OK);

  /* Initialize the glib context */
  status = GLIB_contextInit(&glibContext);
  EFM_ASSERT(status == GLIB_OK);

  glibContext.backgroundColor = White;
  glibContext.foregroundColor = Black;

  /* Fill lcd with background color */
  GLIB_clear(&glibContext);

  /* Use Normal font */
  GLIB_setFont(&glibContext, (GLIB_Font_t *) &GLIB_FontNormal8x8);

  /* Draw text on the memory lcd display*/
  GLIB_drawString(&glibContext,
                        "Player Ready??..",
                        16,
                        3,
                        40,
                        true);
  GLIB_drawString(&glibContext,
                          "Crack open the",
                          15,
                          3,
                          60,
                          true);
  GLIB_drawString(&glibContext,
                             "fortress, Free",
                            15,
                            3,
                            80,
                            true);
  GLIB_drawString(&glibContext,
                  "the prisoners!",
                              15,
                              3,
                              100,
                              true);


  /* Post updates to display */
  DMD_updateDisplay();
}

void LCD_init(void){
  RTOS_ERR err;
  uint32_t status;

//  /* Enable the memory lcd */
  status = sl_board_enable_display();
  EFM_ASSERT(status == SL_STATUS_OK);

  /* Initialize the DMD support for memory lcd display */
  status = DMD_init(0);
  EFM_ASSERT(status == DMD_OK);

  /* Initialize the glib context */
  status = GLIB_contextInit(&glibContext);
  EFM_ASSERT(status == GLIB_OK);

  glibContext.backgroundColor = White;
  glibContext.foregroundColor = Black;

  /* Fill lcd with background color */
  GLIB_clear(&glibContext);

  /* Use Narrow font */
  GLIB_setFont(&glibContext, (GLIB_Font_t *) &GLIB_FontNarrow6x8);

  // Create Blink Task
  OSTaskCreate(&LCD_TCB,
               "LCD task",
               LCD_task,
               DEF_NULL,
               LCD_TASK_PRIO,
               &LCD_stack[0],
               (TASK_STACK_SIZE / 10u),
               TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}
void game_init(void){
  RTOS_ERR err;
  // Platform
  platform.center = 64;
  platform.length = 25;
  platform.x_velocity = 0;

  platform_context.xMin = platform.center - (platform.length * 0.5);
  platform_context.yMin = BOTTOMBOUND + 5;
  platform_context.xMax = platform.center + (platform.length * 0.5);
  platform_context.yMax = BOTTOMBOUND;

  // Create Health Stats Bar

  OSTaskCreate(&PHY_TCB,                          /* Create the Platform Physics Task.                               */
             "Platform Physics Task",
              &Platform_Physics_Task,
              DEF_NULL,
              PHYS_TASK_PRIO,
             &PHY_stack[0],
             (TASK_STACK_SIZE / 10u),
              TASK_STACK_SIZE,
              0u,
              0u,
              DEF_NULL,
             (OS_OPT_TASK_STK_CLR),
             &err);
  // Button Task
  OSTaskCreate(&Button_InputTaskTCB,                /* Pointer to the task's TCB.  */
               "Button Task.",                    /* Name to help debugging.     */
               &Button_InputTask,                   /* Pointer to the task's code. */
                DEF_NULL,                          /* Pointer to task's argument. */
                BUTTON_TASK_PRIO,                  /* Task's priority.            */
               &ButtonTaskStk[0],                  /* Pointer to base of stack.   */
               (TASK_STACK_SIZE / 10u),             /* Stack limit, from base.     */
               TASK_STACK_SIZE,                       /* Stack size, in CPU_STK.     */
                0u,                               /* Messages in task queue.     */
                0u,                                /* Round-Robin time quanta.    */
                DEF_NULL,                          /* External TCB data.          */
                OS_OPT_TASK_STK_CLR,               /* Task options.               */
               &err);
  // Shield Task

}

/***************************************************************************//**
 * LCD task.
 ******************************************************************************/
static void LCD_task(void *arg){
  PP_UNUSED_PARAM(arg);

  char plat_speed[16];
  // castle vertecies
  int32_t castleIndex[24] = {40,25,40,5,50,5,50,10,60,10,60,5,70,5,70,10,80,10,80,5,90,5,90,25};
  uint32_t numPoints = 12;


    RTOS_ERR err;

    while (1)
    {
        OSTimeDly(50, OS_OPT_TIME_DLY, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
        GLIB_clear(&glibContext);
    if(newGame)
      {
        loadingScreen();
        OSTimeDly(5000, OS_OPT_TIME_DLY, &err);
        newGame = false;
      }


    if(game_won){
      // Game Won, Wrap up
    }
    else if(game_loss){
      // Game Lost, you suck

    }
    else{
        // Activate Shield
        if(B0_Pressed)
          {
            GLIB_drawCircle(&glibContext, platform.center, 124, platform.length);
          }
        // Activate Railgun
        if(B1_Pressed)
        {

        }
        // Health Stats
        GLIB_drawString(&glibContext, "Castle:", 7, 3,30,1);
//        GLIB_drawRectFilled(&glibContext, &platform_context);
//        GLIB_drawRect(&glibContext, &platform_context);

        GLIB_drawString(&glibContext, "Player:", 7, 70,30,1);
//        GLIB_drawRectFilled(&glibContext, &platform_context);
//        GLIB_drawRect(&glibContext, &platform_context);

        GLIB_drawString(&glibContext, "E:", 2, 70,60,1);
//        GLIB_drawRectFilled(&glibContext, &platform_context);
//        GLIB_drawRect(&glibContext, &platform_context);

       // Draw Castle
       GLIB_drawPolygonFilled(&glibContext,numPoints, &castleIndex);

      //Draw Platform
      GLIB_drawRectFilled(&glibContext, &platform_context);

      //Display Stats
      sprintf(plat_speed, "%d", abs(platform.x_velocity));

      //Map Drawings
        // Debug
//      GLIB_drawString(&glibContext, "Plat:", 5, 5,120,1);
//      GLIB_drawString(&glibContext, plat_speed, 16, 35,120,1);
      GLIB_drawLine(&glibContext, 2,0, 2, 128);
      GLIB_drawLine(&glibContext, LEFTBOUND,0, LEFTBOUND, 128);
      GLIB_drawLine(&glibContext, RIGHTBOUND, 0, RIGHTBOUND, 128);
      GLIB_drawLine(&glibContext, 125, 0, 125, 128);

    }

        DMD_updateDisplay();
    }
}
/***************************************************************************//**
 * Platform Physics task.
 ******************************************************************************/
static void Platform_Physics_Task(){
  RTOS_ERR err;
  int platform_right, platform_left;

  while(1){


    OSTimeDly(REFRESHRATE, OS_OPT_TIME_DLY, &err);
    // create platform bounds
    platform_right = platform.center + (platform.length * 0.5);
    platform_left  = platform.center - (platform.length * 0.5);

    // Check map bounds
    if (platform_right >= RIGHTBOUND) platform.x_velocity = -platform.x_velocity + 5;
    if (platform_left <= LEFTBOUND)  platform.x_velocity = -platform.x_velocity - 5;


    CAPSENSE_Sense();
    left_sense_hard  = CAPSENSE_getPressed(CSEN0_pin);
    left_sense_soft  = CAPSENSE_getPressed(CSEN1_pin);
    right_sense_soft = CAPSENSE_getPressed(CSEN2_pin);
    right_sense_hard = CAPSENSE_getPressed(CSEN3_pin);

    if(left_sense_hard) platform.x_velocity = (platform.x_velocity - 2) % 20;
    if(left_sense_soft) platform.x_velocity = (platform.x_velocity - 1) % 20;
    if(right_sense_hard) platform.x_velocity = (platform.x_velocity + 2) % 20;
    if(right_sense_soft) platform.x_velocity = (platform.x_velocity + 1) % 20;

    platform.center += platform.x_velocity;

    // Set Boundary Locations
    platform_right = platform.center + (platform.length * 0.5);
    platform_left  = platform.center - (platform.length * 0.5);

    if (platform_right >= RIGHTBOUND) platform.center = RIGHTBOUND - ((platform.length * 0.5)+.2);
    if (platform_left  <= LEFTBOUND)  platform.center = LEFTBOUND + ((platform.length * 0.5)+.2);


    //update platform GLIB variable
    platform_context.xMin = platform.center - (platform.length * 0.5);
    platform_context.xMax = platform.center + (platform.length * 0.5);
  }
}
/***************************************************************************//**
 * Satchel task.
 ******************************************************************************/
