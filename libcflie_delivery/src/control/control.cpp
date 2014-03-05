// Copyright (c) 2013, Jan Winkler <winkler@cs.uni-bremen.de>
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Universität Bremen nor the names of its
//       contributors may be used to endorse or promote products derived from
//       this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <cflie/CCrazyflie.h>
#include <stdio.h>
#include <unistd.h>
#include "../leap/leap_c.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <pthread.h>

using namespace std;

//CS50_TODO:  define your own states
//These are all the states the copter will have 
#define FLY_STATE 1
#define HOVER_STATE 2
#define LAND_STATE 3
#define PRE_HOVER_STATE 4
#define PRE_FLY_STATE 5
/*EXTENSION*/
#define GESTURE_STATE 6
/*EXTENSION*/

//#define *other states*
//#define the states you will use here

//CS50_TODO:  define your own signals
//Define All the signals
#define NO_SIG 11//Each time when our state machine process the current signal
//A good practice is that when the current signal is processd, set the current signal variable back to "no signal"
#define CHANGE_HOVER_SIG 12 //Use to transit state between Normal and Hover
#define LAND_SIG 13
#define TIME_OUT_SIG 14

/*EXTENSION*/
#define GESTURE_SIG 15
/*EXTENSION*/

//The "time out signal" should be created every several seconds
//#define *other signals*
//#define the signals you will use here

/*EXTENSION*/
#define BREAK 0
#define UP cflieCopter, 40001
#define DOWN cflieCopter, 27001
#define FORWARD cflieCopter, 10
#define BACKWARDS cflieCopter, -10
#define RIGHT cflieCopter, 10
#define LEFT cflieCopter, -10
#define NOTHING 7

#define SUPERUP cflieCopter, 50001
#define SUPERDOWN cflieCopter, 17001
#define SUPERFORWARDS cflieCopter, 30
#define SUPERBACKWARDS cflieCopter, -30
#define SUPERRIGHT cflieCopter, 30
#define SUPERLEFT cflieCopter, -30
/*EXTENSION*/

//Define the trim value
//Some copters are not very balanced
//If it constantly drift to one direction
//You may want to define a "trim"
#define TRIM_ROLL 0
#define TRIM_PITCH 0

//CS50_TODO:  define other variables here
//Such as: current states, current thrust
//variable for state and signals
#define ABS_PITCH_VALUE 8.5 // constant for pitch value if pitch is activated
#define ABS_ROLL_VALUE 8.5 // constant for roll value if roll is activated
#define POS_PITCH_THRESHOLD .35 // threshold for leap direction to set positive pitch
#define NEG_PITCH_THRESHOLD -.35 // threshold for leap direction to set negative pitch
#define POS_ROLL_THRESHOLD .35 // threshold for leap direction to set positive roll
#define NEG_ROLL_THRESHOLD -.35 // threshold for leap direction to set negative roll
#define HOVER_SWIPE_THRESHOLD 1000 // threshold for the velocity to interpret hover swipe gesture
#define THRUST_CONSTANT 35700 // constant for starting thrust level
#define FINGER_COUNT_THRESHOLD 2 // if we have less than this amount of fingers detected, we will land
#define HOVER_THRUST_CONST 32767 // hover thrust constant
#define LANDING_REDUCTION_CONSTANT 20 // landing reduction constant
#define THRUST_MULTIPLIER 52.0 // multiply hand position by this number to get thrust
#define BATT_MULTIPLIER_CONST 4.0 // constant for the battery multiplier
#define TIME_GAP 350 // gap for time break

#define 

int current_signal = NO_SIG; // default signal is no signal
int current_state = FLY_STATE; //default state is fly state
float current_thrust;
float current_roll;
float current_pitch;
double dTimeNow;
double dTimePrevious;

//The pointer to the crazy flie data structure
CCrazyflie *cflieCopter=NULL;

//CS50_TODO:  define other helper function here
//In normal state, the flyNormal function will be called, set different parameter
//In hover state, different function should be called, because at that time, we should set the thrust as a const value(32767), see reference 
//(http://forum.bitcraze.se/viewtopic.php?f=6&t=523&start=20)

//The helper functions 
void flyNormal(CCrazyflie *cflieCopter){ 
  if (current_thrust != -1){
    setThrust( cflieCopter, THRUST_CONSTANT + ( current_thrust * ( THRUST_MULTIPLIER - ( BATT_MULTIPLIER_CONST * batteryLevel(cflieCopter) ) ) ) );}
    else{setThrust(cflieCopter, 0);}
    setPitch( cflieCopter, current_pitch );
    setRoll( cflieCopter, current_roll );
  }

  void flyHover(CCrazyflie *cflieCopter){
    setThrust( cflieCopter, HOVER_THRUST_CONST );
    setPitch( cflieCopter, current_pitch );
    setRoll( cflieCopter, current_roll );
  }

  void land( CCrazyflie *cflieCopter ) {
    current_thrust = current_thrust - LANDING_REDUCTION_CONSTANT;
    if( (current_thrust - THRUST_CONSTANT) < 0){current_thrust = -1;}
    current_roll = 0;
    current_pitch = 0;
    flyNormal( cflieCopter );
  }

//The leap motion call back functions
//Leap motion functions
  void on_init(leap_controller_ref controller, void *user_info)
  {
    printf("init\n");
  }

  void on_connect(leap_controller_ref controller, void *user_info)
  {
    printf("connect\n");
  }

  void on_disconnect(leap_controller_ref controller, void *user_info)
  {
    printf("disconnect\n");
  }

  void on_exit(leap_controller_ref controller, void *user_info)
  {
    printf("exit\n");
  }

//This function will be called when leapmotion detect hand gesture, 
  void on_frame(leap_controller_ref controller, void *user_info)
  {
    leap_frame_ref frame = leap_controller_copy_frame(controller, 0);
    leap_vector velocity;
    leap_vector position;
    leap_vector direction;
  
/*

if(velo.x>500){
 double dTimeNow = currentTime();
 if(dTimeNow - cflieCopter->m_lastModeSet >500){
 printf("%f %f %f\n\n", velo.x,dTimeNow,cflieCopter->m_lastModeSet);
 if(current_mode==NORMAL){
 printf("%s\n", "current_mode change to Hover");
 current_mode=HOVER;
 cflieCopter->m_setHoverPoint=2;
 cflieCopter->m_lastModeSet=dTimeNow;
 }
 else if(current_mode==HOVER){
 printf("%s\n", "current_mode change to Pre-Normal");
 current_mode=PRE_NORMAL;
 cflieCopter->m_lastModeSet=dTimeNow;
 }
 else if(current_mode==PRE_NORMAL){
 printf("%s\n", "current_mode change to Normal");
 if(cflieCopter->m_lastModeSet-dTimeNow>3000)
 current_mode=HOVER;
 else{
 current_mode=NORMAL;
 cflieCopter->m_setHoverPoint=-2;
 cflieCopter->m_lastModeSet=dTimeNow;
 }
 }
 
 }
}
*/


if ( current_signal == NO_SIG ) {

  if (current_state == PRE_HOVER_STATE || current_state == PRE_FLY_STATE){
    dTimeNow = currentTime();
    if(dTimeNow - dTimePrevious >TIME_GAP){
      current_signal = TIME_OUT_SIG;
      dTimePrevious=dTimeNow;
      leap_frame_release(frame);
      return;
    }
  }

  for (int i = 0; i < leap_frame_hands_count(frame); i++) {

    leap_hand_ref hand = leap_frame_hand_at_index( frame, i );

      // Grab the hand velocity, direction, and position vectors
    leap_hand_palm_velocity(hand, &velocity);
    leap_hand_direction(hand, &direction);
    leap_hand_palm_position(hand, &position);
    
    /*EXTENSION*/
    if (leap_frame_hands_count(frame) == 1 && leap_hand_fingers_count( hand ) == FINGER_COUNT_THRESHOLD + 1 ) {
      current_signal = GESTURE_SIG;
      leap_frame_release(frame);
      return;
    }
    /*EXTENSION*/


      // If we detect a swipe gesture (high velocity), enter or exit hover mode
    if (leap_frame_hands_count(frame) > 1 && velocity.x > HOVER_SWIPE_THRESHOLD ) {
          //dTimeNow = currentTime();
     //  printf( "gesture detected" );
     current_signal = CHANGE_HOVER_SIG;
     leap_frame_release(frame);
     return;
   }  

    // printf("numhands: %d\n", leap_frame_hands_count(frame));
    // If we have less than 2 fingers detected, set signal to land
   if (leap_frame_hands_count(frame) == 1 && leap_hand_fingers_count( hand ) < FINGER_COUNT_THRESHOLD ) {
    current_signal = LAND_SIG;
    leap_frame_release(frame);

    return;
  }

      // Set the thrust value
  current_thrust = position.y;

      // Set the roll value
  if ( direction.x > POS_ROLL_THRESHOLD ) {
   current_roll = ABS_ROLL_VALUE;
 }
 else if ( direction.x < NEG_ROLL_THRESHOLD ) {
   current_roll = -ABS_ROLL_VALUE;
 }
 else {
   current_roll = 0;
 }

      // Set the pitch value
 if ( direction.y > POS_PITCH_THRESHOLD ) {
   current_pitch = ABS_PITCH_VALUE;
 }
 else if ( direction.y < NEG_PITCH_THRESHOLD ) {
   current_pitch = -ABS_PITCH_VALUE;
 }
 else {
   current_pitch = 0;
 }     
}
leap_frame_release(frame);
return;
}

  // Wait for the current signal to be consumed before doing anything else
else {
  leap_frame_release(frame);
  return;
}
}



//This the leap motion control callback function
//You don't have to modifiy this
void* leap_thread(void * param){
  struct leap_controller_callbacks callbacks;
  callbacks.on_init = on_init;
  callbacks.on_connect = on_connect;
  callbacks.on_disconnect = on_disconnect;
  callbacks.on_exit = on_exit;
  callbacks.on_frame = on_frame;
  leap_listener_ref listener = leap_listener_new(&callbacks, NULL);
  leap_controller_ref controller = leap_controller_new();
  leap_controller_add_listener(controller, listener);
  while(1);
}

/*EXTENSION*/
void gestureMacro(CCrazyflie *cflieCopter){
 int i = 0;

 while (i < 400005){

  if (i < 100000)current_roll = -10; 
  else if (i < 200000)current_roll = 10;
  else if (i < 200050)current_roll = 0;

  else if (i < 300000)current_pitch = -10;
  else if (i < 400000)current_pitch = 10;
  else if (i < 400005)current_pitch = 0;

  flyNormal(cflieCopter);

  i++;
  printf("GESTURE URMA GURD\n");
}       
}
/*EXTENSION*/


///MOVE BETWEEN STATES

//This thread will check the current state and send corrsponding command to the copter
void* main_control(void * param){
  CCrazyflie *cflieCopter=(CCrazyflie *)param;

  while(cycle(cflieCopter)) {

    switch ( current_signal ) {

      case NO_SIG:
        break;

      case CHANGE_HOVER_SIG:
if (current_state == HOVER_STATE ) {
  current_state = PRE_FLY_STATE;
}
else if ( current_state == FLY_STATE ) {
  current_state = PRE_HOVER_STATE;
}

break;

  case LAND_SIG:
    current_state = LAND_STATE;
break;

case TIME_OUT_SIG:
  if (current_state == PRE_FLY_STATE) {
    current_state = FLY_STATE;
  }
  else if (current_state == PRE_HOVER_STATE) {
    current_state = HOVER_STATE;
  }
  break;

}    

    // Consume the current signal
    current_signal = NO_SIG;

    //SWITCH CASE II : ELECTRIC BOOGALOO

  switch( current_state ) {

    case PRE_FLY_STATE:
    flyHover( cflieCopter );
    break;

    case FLY_STATE:
    flyNormal( cflieCopter );
    break;

    case LAND_STATE:
       // If sig is land, land
    land( cflieCopter );
    break;

    case HOVER_STATE:
    flyHover( cflieCopter );
    break;

    case PRE_HOVER_STATE:
    flyNormal( cflieCopter );
    break;

    /*EXTENSION*/
    case GESTURE_STATE:
    //gestureMacro( cflieCopter );
    break;
    /*EXTENSION*/

    default:
    break;
  }

}

printf("%s\n", "exit");
return 0;
}



//This this the main function, use to set up the radio and init the copter
int main(int argc, char **argv) {
  CCrazyRadio *crRadio = new CCrazyRadio;
  //CS50_TODO
  //The second number is channel ID
  //The default channel ID is 10
  //Each group will have a unique ID in the demo day 
  CCrazyRadioConstructor(crRadio,"radio://0/19/250K");
  
  if(startRadio(crRadio)) {
    cflieCopter=new CCrazyflie;
    CCrazyflieConstructor(crRadio,cflieCopter);

    //Initialize the set thrust value to 30001
    setThrust( cflieCopter, 36001 );    
    
    // Enable sending the setpoints. This can be used to temporarily
    // stop updating the internal controller setpoints and instead
    // sending dummy packets (to keep the connection alive).
    setSendSetpoints(cflieCopter,true);

    // Set up the leap and main copter control threads
    pthread_t leapThread;
    pthread_t mainThread;
    pthread_create(&leapThread, NULL, leap_thread, NULL); 
    pthread_create(&mainThread, NULL, main_control, cflieCopter);

    // Loop until we exit
    while (1) {}

    // Failure to find dongle
  } else {
    printf("%s\n", "Could not connect to dongle. Did you plug it in?");
  }
  return 0;
}
