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
#include <stdlib.h>
#include <math.h>

#define GRAVITY 9.81   // m/s^2
#define PI 3.14159265358979323846
#define LAUNCH_ANGLE 45

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
static GLIB_Rectangle_t drawSatchel;

// Health Stats
static GLIB_Rectangle_t castle; // Outline
static GLIB_Rectangle_t caslteHealth; // Moveable Health bar

static GLIB_Rectangle_t player;
static GLIB_Rectangle_t playerHealth;

static GLIB_Rectangle_t energy;
static GLIB_Rectangle_t energyTotal;

volatile float capCharge;

platform_obj platform;

bool left_sense_hard, left_sense_soft, right_sense_hard,right_sense_soft;
bool      game_won      = false;
bool      game_loss     = false;
bool      fireRailgun   = false;
bool      shield        = false;
bool B0_Pressed = false;
bool B1_Pressed = false;
bool newGame = true;
float railGun;
bool launchCannon = false;
bool satchelActive = false;
bool descending = false;

OS_TMR       PERIODIC_TMR;

OS_Q         App_MessageQ;
OS_FLAG_GRP  Flags;
OS_FLAG_GRP  LED_Flags;
OS_SEM       Semaphore;
OS_MUTEX     VS_Mutex;
OS_MUTEX     VD_Mutex;
OS_Q         Button_Q;
int32_t RxIndexStartPos[16] = {62,120,62,116,50,104,56,107,76,117,73,120,64,116,64,120};
int32_t RGIndex[16] = {62,120,62,116,50,104,56,107,76,117,73,120,64,116,64,120};
uint8_t RGPower = 0;
railgun_obj RGShot;
uint8_t flag = 0;
uint8_t launchVelocity = 0;
satchel_obj satchel;
int numSatchels = 0;
bool bounce = false;


/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/
static void LCD_task(void *arg);
static void Platform_Physics_Task();
static void Button_InputTask (void  *p_arg);
static void satchel_init(satchel_obj *satchel);



/*******************************************************************************
 **************************   GLOBAL FUNCTIONS   *******************************
 ******************************************************************************/

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
                    "the Prisoners!",
                                15,
                                3,
                                100,
                                true);

  /* Post updates to display */
  DMD_updateDisplay();
}
void gameWin(void)
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
                        "YOU WIN!",
                        8,
                        3,
                        40,
                        true);

  /* Post updates to display */
  DMD_updateDisplay();
}

void gameLoss(void)
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
                        "GAME OVER!",
                        10,
                        3,
                        40,
                        true);


  /* Post updates to display */
  DMD_updateDisplay();
}

static void satchel_init(satchel_obj *satchel){
  satchel->x_position = 14;
  satchel->y_position= 25;
  return;
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

  // satchel Starting point
  drawSatchel.xMin = 10;
  drawSatchel.yMin = 21;
  drawSatchel.xMax = 14;
  drawSatchel.yMax = 25;

  // Create Satchel
  satchel_init(&satchel);

  // Create Health Stats Bar
  castle.xMin = 60; // Outline
  castle.xMax = 120;
  castle.yMin = 0;
  castle.yMax = 6;

 // Moveable Health bar
  caslteHealth.xMin = 60; // Outline
  caslteHealth.xMax =120;
  caslteHealth.yMin = 0;
  caslteHealth.yMax = 6;

  player.xMin = 60; // Outline
  player.xMax = 120;
  player.yMin = 9;
  player.yMax = 15;

 // Moveable Health bar
  playerHealth.xMin = 60; // Outline
  playerHealth.xMax = 120;
  playerHealth.yMin = 9;
  playerHealth.yMax = 15;

  energy.xMin = 60; // Outline
  energy.xMax = 120;
  energy.yMin = 18;
  energy.yMax = 24;

// Moveable Energy bar
  energyTotal.xMin = 60; // Outline
  energyTotal.xMax = 120;
  energyTotal.yMin = 18;
  energyTotal.yMax = 24;


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
}

/***************************************************************************//**
 * LCD task.
 ******************************************************************************/
static void LCD_task(void *arg){
  PP_UNUSED_PARAM(arg);

  char plat_speed[16];
  // castle vertecies
  int32_t castleIndex[46] = {0,128,0,65,0,25,2,25,2,30,4,30,4,25,6,25,6,30,8,30,8,25,10,25,10,30,12,30,12,25,14,25,14,65,20,65,20,70,20,75,30,75,30,128,0,128};
  uint32_t numPoints = 23;


  // Draw Raigun
  uint32_t numPointsRG = 8;

    RTOS_ERR err;

    while (1)
    {
        OSTimeDly(50, OS_OPT_TIME_DLY, &err);
        EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
        GLIB_clear(&glibContext);
    if(newGame)
      {
        loadingScreen();
        OSTimeDly(2000, OS_OPT_TIME_DLY, &err);
        newGame = false;
      }

    if(game_won){
      // Game Won, Wrap up
        gameWin();

    }
    else if(game_loss){
      // Game Lost, you suck
        gameLoss();

    }
    else {
        // Activate Shield
        if(B0_Pressed)
          {
            if(energyTotal.xMax > energyTotal.xMin)
            {
            GLIB_drawCircle(&glibContext, platform.center, 124, platform.length);
            energyTotal.xMax = energyTotal.xMax - 4;
            }
          }
        // Draw RailgunShot
        // Need to check Available Energy
        if(launchCannon && (energyTotal.xMax >= energyTotal.xMin) && !B0_Pressed)
          {
            if(RGShot.power == 1)
              {
              energyTotal.xMax = energyTotal.xMax - 20;
              flag = 1;
              RGShot.power = 0;
              }
            if(RGShot.power == 2)
              {
              energyTotal.xMax = energyTotal.xMax - 30;
              flag = 2;
              RGShot.power = 0;
              }
            if(RGShot.power == 3)
              {
              energyTotal.xMax = energyTotal.xMax - 40;
              flag = 3;
              RGShot.power = 0;
              }

            if(RGShot.x_position > 30 || RGShot.y_position < 65)
              {
            GLIB_drawCircleFilled(&glibContext, RGShot.x_position, RGShot.y_position, 4);
              }
            RGShot.x_position = RGShot.x_position - 5;
            RGShot.y_position = RGShot.y_position - 5;
          }

        // Capacitor Recharge
        if(energyTotal.xMax < 120 && !B0_Pressed)
          {
            energyTotal.xMax = energyTotal.xMax + ((energyTotal.xMax + 1) % 5);
          }
//        bound check

        if(energyTotal.xMax > 120)
          {
            energyTotal.xMax = 120;
          }

        // Shot is in Range of Castle
        if(RGShot.x_position <= 20 && RGShot.y_position <= 70)
          {

            if(flag == 1)caslteHealth.xMax = caslteHealth.xMax - 10;
            if(flag == 2)caslteHealth.xMax = caslteHealth.xMax - 15;
            if(flag == 3)caslteHealth.xMax = caslteHealth.xMax - 20;
            flag = 0;
          }
        // Castle Damage Attack
        if(!B0_Pressed && (satchel.y_position >= 115) && satchelActive)
          {
            // satchel on the right of platform in range
            if(drawSatchel.xMax >= platform_context.xMax && drawSatchel.xMin <= platform_context.xMax)
              {
                playerHealth.xMax = playerHealth.xMax - 30;
                GLIB_drawCircle(&glibContext, satchel.x_position, 115, 12);
                OSTimeDly(150, OS_OPT_TIME_DLY, &err);
                satchelActive = false;
                numSatchels = 0;
                descending = false;
                bounce = false;
              }
            if(drawSatchel.xMin <= platform_context.xMin && drawSatchel.xMax >= platform_context.xMin)
            {
              playerHealth.xMax = playerHealth.xMax - 30;
              GLIB_drawCircle(&glibContext, satchel.x_position, 115, 12);
              OSTimeDly(150, OS_OPT_TIME_DLY, &err);
              satchelActive = false;
              numSatchels = 0;
              descending = false;
              bounce = false;
            }
          }

        // Health Stats
        GLIB_drawString(&glibContext, "Castle:", 7, 0,0,1);

        // Castle Health Status Bar
        GLIB_drawRect(&glibContext, &castle);
        GLIB_drawRectFilled(&glibContext, &caslteHealth);

        // Player Health Status Bar
        GLIB_drawString(&glibContext, "Player:", 7, 0,10,1);
        GLIB_drawRect(&glibContext, &player);
        GLIB_drawRectFilled(&glibContext, &playerHealth);

        GLIB_drawString(&glibContext, "E:", 2, 20,20,1);
        GLIB_drawRect(&glibContext, &energy);
        GLIB_drawRectFilled(&glibContext, &energyTotal);


       // Draw Castle
       GLIB_drawPolygonFilled(&glibContext,numPoints, &castleIndex);

       // Draw Railgun
       GLIB_drawPolygonFilled(&glibContext,numPointsRG,RGIndex);

       // Blink Castle Health
       if(caslteHealth.xMax <= 90) GPIO_PinOutToggle(LED1_port, LED1_pin);
       else GPIO_PinOutClear(LED1_port, LED1_pin);


       // Draw Satchel
       if(satchelActive)
       {
       GLIB_drawRectFilled(&glibContext, &drawSatchel);
       }

      //Draw Platform
      GLIB_drawRectFilled(&glibContext, &platform_context);

      //Map Drawings

      GLIB_drawLine(&glibContext, RIGHTBOUND, 0, RIGHTBOUND, 128);
      GLIB_drawLine(&glibContext, 125, 0, 125, 128);

    }

        DMD_updateDisplay();
    }
}


// Calculates the max height and Distance of the Satchel Obj
satchel_obj launch_projectile(double angle_deg, double velocity) {
    satchel_obj satchel;
    double angle_rad = angle_deg * PI / 180.0;  // convert degrees to radians
    satchel.time_of_flight = (2 * velocity * sin(angle_rad)) / GRAVITY;
    satchel.max_height = pow(velocity, 2) * pow(sin(angle_rad), 2) / (2 * GRAVITY);
    satchel.max_distance = pow(velocity, 2) * sin(2 * angle_rad) / GRAVITY;
    return satchel;
}


/***************************************************************************//**
 * Platform Physics task.
 ******************************************************************************/
static void Platform_Physics_Task(){
  RTOS_ERR err;
  int platform_right, platform_left;

  while(1){


    OSTimeDly(REFRESHRATE, OS_OPT_TIME_DLY, &err);
    if(B1_Pressed)
      {
        GPIO_PinOutToggle(LED0_port, LED0_pin);
        railGun += 1;
      }
    if(!B1_Pressed)
    {
        GPIO_PinOutClear(LED0_port, LED0_pin);
        railGunCalc(railGun);
        if(RGPower != 0)
          {
            launchCannon = true;
            // Set Starting Spot
            RGShot.x_position = RGIndex[6];
            RGShot.y_position = 107;
            RGShot.power = RGPower;
          }
        railGun = 0;
    }
    // Satchel Launch
    if(numSatchels < 1)
      {
        float xinterval = 0,yinterval = 0;
        double velocity = (rand() % 11) + 20; // random velocity between 10 and 50m/s
        satchel = launch_projectile(LAUNCH_ANGLE, velocity);
        satchel.x_position = 12;
        satchel.y_position = 25;
        if(satchel.max_height > 25) satchel.max_height = 25; // Y Upper limit on LCD
        if(satchel.max_distance > 114) satchel.max_height = 114; // X Upper limit on LCD
        numSatchels++;
        satchelActive = true;
      }
    if(satchelActive)
      {
        if((satchel.x_position < satchel.max_distance) && !bounce) satchel.x_position = satchel.x_position + 3;
        else if((satchel.x_position < satchel.max_distance) && !bounce)
          {
          satchel.x_position = satchel.x_position + 1;
          }
        if(satchel.x_position >= 128)
          {
            bounce = true;
          }
        if(bounce) satchel.x_position = satchel.x_position - 3;

        if(!descending && (satchel.y_position > 25 - satchel.max_height))
          {
            satchel.y_position = satchel.y_position - 2;

          }
        else{
            // descending arch
            descending = true;
            if(satchel.y_position < 40) satchel.y_position = satchel.y_position + 4;
            if(satchel.y_position >= 40) satchel.y_position = satchel.y_position + 6;
        }
        // Hit Bottom
        if(satchel.y_position >= 124)
          {
            // bottom hit, explode
            RTOS_ERR error;
            satchelActive = false;
            numSatchels = 0;
            descending = false;
            bounce = false;
          }
        drawSatchel.xMin = satchel.x_position - 6;
        drawSatchel.xMax = satchel.x_position + 6;
        drawSatchel.yMin = satchel.y_position - 6;
        drawSatchel.yMax = satchel.y_position + 6;
      }

    // End Game Condiitons
    if(playerHealth.xMax <= playerHealth.xMin) game_loss = 1;
    if(caslteHealth.xMax <= caslteHealth.xMin) game_won = 1;

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
    if(platform.x_velocity < -15 && platform_left >=  LEFTBOUND)
      {
        game_loss = true;
      }

    platform.center += platform.x_velocity;

    // Set Boundary Locations
    platform_right = platform.center + (platform.length * 0.5);
    platform_left  = platform.center - (platform.length * 0.5);

    // check boundary conditions, platform never leaves map
    if (platform_right >= RIGHTBOUND) platform.center = RIGHTBOUND - ((platform.length * 0.5)+.2);
    if (platform_left  <= LEFTBOUND)  platform.center = LEFTBOUND + ((platform.length * 0.5)+.2);


    //update platform GLIB variable
    platform_context.xMin = platform.center - (platform.length * 0.5);
    platform_context.xMax = platform.center + (platform.length * 0.5);
    int8_t offset = 0;

    // Check map bounds again
    offset = platform.center - 64;

    // update railgun mount location
    for(int i = 0; i < 16; i++)
      {
        if(i % 2 == 0 || i == 0)
          {
            // is even index == x valyes
            RGIndex[i] = RxIndexStartPos[i] + offset;
          }
      }

  }
}

void railGunCalc(float power)
{
// Set 3 ranges of power depending on input power
  if(power == 0) RGPower = 0;
  if(power > 0 && power < 5)
    {
      RGPower = 1;
    }
  else if(power > 10 && power < 15)
    {
      RGPower = 2;
    }
  else if(power > 15)
      {
        RGPower = 3;
      }
  else RGPower = 0;
}








