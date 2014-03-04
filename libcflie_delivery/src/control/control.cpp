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
#include "../leap/leap_gesture.h"
#include "Leap.h"
#include <stdio.h>
#include <unistd.h>
#include "../leap/leap_c.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdarg.h>

using namespace std;

#define CHECKFIRST(x)  if (x[0] != 0){printf("macro already exists\n");return 0;}

#define BREAK 0
#define UP 1
#define DOWN 2
#define FORWARD 3
#define BACKWARDS 4
#define RIGHT 5
#define LEFT 6
#define NOTHING 7

#define SUPERUP 8
#define SUPERDOWN 9
#define SUPERFORWARDS 10
#define SUPERBACKWARDS 11
#define SUPERRIGHT 12
#define SUPERLEFT 13

#define TYPE_INVALID -1
#define TYPE_CIRCLE 4
#define TYPE_SCREEN_TAP 5
#define TYPE_KEY_TAP 6

//CS50_TODO:  define your own states
//These are all the states the copter will have 
#define FLY_STATE 1
#define HOVER_STATE 2
#define LAND_STATE 3
#define PRE_HOVER_STATE 4
#define PRE_FLY_STATE 5
#define GESTURE_STATE 6

//CS50_TODO:  define your own signals
//Define All the signals
#define NO_SIG 11//Each time when our state machine process the current signal
//A good practice is that when the current signal is processd, set the current signal variable back to "no signal"
#define CHANGE_HOVER_SIG 12 //Use to transit state between Normal and Hover
#define NORMAL_SIG 13
#define LAND_SIG 14
#define GESTURE_SIG 15
//The "time out signal" should be created every several seconds
//#define *other signals*
//#define the signals you will use here

//Define the trim value
//Some copters are not very balanced
//If it constantly drift to one direction
//You may want to define a "trim"
#define TRIM_ROLL 0
#define TRIM_PITCH 0

//CS50_TODO:  define other variables here
//Such as: current states, current thrust
//variable for state and signals
#define ABS_PITCH_VALUE 10 // constant for pitch value if pitch is activated
#define ABS_ROLL_VALUE 10 // constant for roll value if roll is activated
#define POS_PITCH_THRESHOLD .35 // threshold for leap direction to set positive pitch
#define NEG_PITCH_THRESHOLD -.35 // threshold for leap direction to set negative pitch
#define POS_ROLL_THRESHOLD .35 // threshold for leap direction to set positive roll
#define NEG_ROLL_THRESHOLD -.35 // threshold for leap direction to set negative roll
#define HOVER_SWIPE_THRESHOLD 500 // threshold for the velocity to interpret hover swipe gesture
#define THRUST_CONSTANT 38500 // constant for starting thrust level
#define FINGER_COUNT_THRESHOLD 2 // if we have less than this amount of fingers detected, we will land
#define HOVER_THRUST_CONST 32767 // hover thrust constant
#define LANDING_REDUCTION_CONSTANT 2 // landing reduction constant
#define THRUST_MULTIPLIER_CONST 12.0 // multiply hand position by this number to get thrust
#define BATT_MULTIPLIER_CONST 5.0 // constant for the battery multiplier
#define SLEEP_COUNT 10000 // sleep count
#define MAX_HAND_COUNT 2 // max hands permitted on the leap control

int current_signal = NO_SIG; // default signal is no signal
int current_state = FLY_STATE; //default state is fly state
int current_gesture = -1;
int macros[6][1000];
float current_thrust;
float current_roll;
float current_pitch;


//The pointer to the crazy flie data structure
CCrazyflie *cflieCopter=NULL;

//CS50_TODO:  define other helper function here
//In normal state, the flyNormal function will be called, set different parameter
//In hover state, different function should be called, because at that time, we should set the thrust as a const value(32767), see reference 
//(http://forum.bitcraze.se/viewtopic.php?f=6&t=523&start=20)

//The helper functions 
void flyNormal(CCrazyflie *cflieCopter){ 
  //printf( "%f\n", THRUST_CONSTANT + ( current_thrust * THRUST_MULTIPLIER_CONST - BATT_MULTIPLIER_CONST - batteryLevel(cflieCopter) ) ) );
  setThrust( cflieCopter, THRUST_CONSTANT + ( current_thrust * THRUST_MULTIPLIER_CONST * ( BATT_MULTIPLIER_CONST - batteryLevel(cflieCopter) ) ) );
  setPitch( cflieCopter, current_pitch );
  setRoll( cflieCopter, current_roll );
  //printf( "flying normal\n");
}

void flyHover(CCrazyflie *cflieCopter){
  setThrust( cflieCopter, HOVER_THRUST_CONST );
  setPitch( cflieCopter, current_pitch );
  setRoll( cflieCopter, current_roll );
  //printf( "flying hover\n" );
}

void land( CCrazyflie *cflieCopter ) {
  current_thrust -= LANDING_REDUCTION_CONSTANT;
  current_roll = 0;
  current_pitch = 0;
  flyNormal( cflieCopter );
  //printf( "landing\n");
}

void countSleep( CCrazyflie *cflieCopter ) {
  int count = 0;
  while ( count < SLEEP_COUNT ) {
    if ( current_state == FLY_STATE ) {
      flyNormal( cflieCopter );
    }
    else if ( current_state == HOVER_STATE ) {
      flyHover( cflieCopter );
    }
    count++;
  }
}

void countSleep2() {
  int count = 0;
  while ( count < SLEEP_COUNT ) {
    count++;
  }
}


int singleMotion(CCrazyflie *cflieCopter, int motion){
 
switch (motion) {
    
  case UP:
    setThrust( cflieCopter, 40001);
    break;
    
  case DOWN:
    setThrust( cflieCopter, 27001);
    break;
    
  case FORWARD:
    setPitch( cflieCopter, 10);
    break;
    
  case BACKWARDS:
    setPitch( cflieCopter, -10);
    break; 
    
  case RIGHT:
    setRoll( cflieCopter, 10);
    break;
    
  case LEFT:
    setRoll( cflieCopter, -10);
    break; 
    
  case NOTHING:
    break;
    
  case SUPERUP:
    setThrust( cflieCopter, 50001);
    break;
    
  case SUPERDOWN:
    setThrust( cflieCopter, 17001);
    break;

  case SUPERFORWARDS:
    setPitch( cflieCopter, 30);
    break;
    
  case SUPERBACKWARDS:
    setPitch( cflieCopter, -30);
    break; 
    
  case SUPERRIGHT:
    setRoll( cflieCopter, 30);
    break;
    
  case SUPERLEFT:
    setRoll( cflieCopter, -30);
    break; 
    
  default:
    return 1;
  }
  
  return 1;
  
}


//The leap motion call back functions
//Leap motion functions
void on_init(leap_controller_ref controller, void *user_info)
{
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
  int current_finger_count;
  
  for (int i = 0; i < leap_frame_hands_count(frame); i++) {
    
    leap_hand_ref hand = leap_frame_hand_at_index( frame, i );
      
    // Grab the hand velocity, direction, and position vectors
    leap_hand_palm_velocity(hand, &velocity);
    leap_hand_direction(hand, &direction);
    leap_hand_palm_position(hand, &position);
    
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
    
    // If we detect a swipe gesture (high velocity) for any hand, enter or exit hover mode
    //if ( current_signal == NO_SIG ) {
    // if ( velocity.x > HOVER_SWIPE_THRESHOLD ) {
    //   printf( "gesture detected" );
    //   countSleep2();
    //   current_signal = CHANGE_HOVER_SIG;
    //   return;
    // }
    //}

    current_finger_count = leap_hand_fingers_count( hand );
    printf( "%d\n", current_finger_count );
  }
  
  
  for (int k = 0; k < leap_frame_gestures_count(frame); k++) {
    
  leap_gesture_ref gesture = leap_frame_gesture_at_index(frame, k);
    
  leap_gesture_type gesture_type = leap_gesture_gesture_type(gesture);
    
  switch (gesture_type) {
      
  // case TYPE_CIRCLE:
  //     //Handle circle gestures
  //     current_gesture = 0;
  //     current_signal = GESTURE_SIG;
  //     leap_frame_release(frame);
  //     return;
      
  //   case TYPE_KEY_TAP:
  //     //Handle key tap gestures
  //     current_gesture = 1;
  //     current_signal = GESTURE_SIG;
  //     leap_frame_release(frame);
  //     return;
      
  //   case TYPE_SCREEN_TAP:
  //     //Handle screen tap gestures
  //     current_gesture = 2;
  //     current_signal = GESTURE_SIG;
  //     leap_frame_release(frame);;
  //     return;
      
    case Leap::Gesture::TYPE_SWIPE:
      //Handle swipe gestures
      current_signal = CHANGE_HOVER_SIG;
      leap_frame_release(frame);
      return;
      
    default:
      //Handle unrecognized gestures
      break;
      
    }
  }
    
  
  // Release the frame
  leap_frame_release(frame);
  
  // Update and assign appropriate signals
  if ( current_signal == NO_SIG ) {
    
    // If we have less than 2 fingers detected, set signal to land
    //if ( current_finger_count < FINGER_COUNT_THRESHOLD ) {
    // current_signal = LAND_SIG;
      // return;
    //}
    
    // Otherwise, we're just in normal mode
    // else {
      current_signal = NORMAL_SIG;
      return;
    // }
  }  
    // Wait for the current signal to be consumed before doing anything else
  else {
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
  new leap_gesture(Leap::Gesture::TYPE_SWIPE);
  while(1);
}

//This thread will check the current state and send corrsponding command to the copter
void* main_control(void * param){
  CCrazyflie *cflieCopter=(CCrazyflie *)param;

  while(cycle(cflieCopter)) {

    if ( current_signal == CHANGE_HOVER_SIG ) {
      if ( current_state == FLY_STATE ) {
	current_state = PRE_HOVER_STATE;
      }
      else if ( current_state == HOVER_STATE ) {
	current_state = PRE_FLY_STATE;
      }
    }

    switch( current_state ) {
   
      case GESTURE_STATE:
      printf( "gesture state\n" );
      //If sig is normal, keep flying
      if ( current_signal == GESTURE_SIG ) {
      int g = 0;
      int sleeper = 0;
      while(macros[current_gesture][g] != 0){
          singleMotion( cflieCopter, macros[current_gesture][g]);
        if (sleeper++ % 100){g++;}
      }
      }
      else {
      	current_state = FLY_STATE;
      }
    
    case FLY_STATE:
      printf( "fly state\n" );
      
      // If sig is normal, keep flying
      if ( current_signal == NORMAL_SIG ) {
	flyNormal( cflieCopter );
	break;
      }
      
      //If sig is change hover, change state to pre-hover
      else if ( current_signal == CHANGE_HOVER_SIG ) {
      	current_state = PRE_HOVER_STATE;
      	break;
      }
      
      // If sig is land, change state to land
      else if ( current_signal == LAND_SIG ) {
	current_state = LAND_STATE;
	break;
      }
     
      break;

    case LAND_STATE:
     printf( "land state\n" );
     
     // If sig is land, land
      if ( current_signal == LAND_SIG ) {
	land( cflieCopter );
	break;
      }

      // If sig is normal, change state to fly
      else if ( current_signal == NORMAL_SIG ) {
	current_state = FLY_STATE;
	break;
      }
      
      break;
      
    case HOVER_STATE:
      printf( "hover state\n" );
      
      // If sig is normal, hover
      if ( current_signal == NORMAL_SIG ) {
	flyHover( cflieCopter );
	break;
      }
      // If sig is change hover, change state to pre fly
      if ( current_signal == CHANGE_HOVER_SIG ) {
      	current_state = PRE_FLY_STATE;
      	break;
      }

      // Design choice: landing not allowed in hover
      break;

    case PRE_HOVER_STATE:
      printf( "pre hover state\n" );

      // Sleep for a bit, switch to hover state and turn on hover mode
      countSleep( cflieCopter );
      current_state = HOVER_STATE;
      turnOnHoverMode( cflieCopter );
      break;

    case PRE_FLY_STATE:
      printf( "pre fly state\n" );
   
      // Sleep for a bit, switch to fly state and turn off hover mode
      countSleep( cflieCopter );
      current_state = FLY_STATE;
      turnOffHoverMode( cflieCopter );
      break;
      
    }

    // Consume the current signal
    current_signal = NO_SIG;
        
  }
  
  printf("%s\n", "exit");
  return 0;
}


// int createMoveMacro(int args, ... ){
  
//   //accounting for the gesture param
//     args = args - 1;

//     va_list list;
//     va_start(list, args);

//     int gesture = va_arg(list, int);
//     CHECKFIRST(macros[gesture]);
//     leap.enableGesture((leap_gesture_type)gesture);

//     for (int i=0; i < args; i++){
//       macros[gesture][i] = va_arg(list,int);
//     }

// // Cleanup the va_list when we're done.
//     va_end(list);
//     return 1;
//   }



//This this the main function, use to set up the radio and init the copter
int main(int argc, char **argv) {
  CCrazyRadio *crRadio = new CCrazyRadio;
  CCrazyRadioConstructor(crRadio,"radio://0/34/250K");

  //createMoveMacro(6,TYPE_CIRCLE,FORWARD,LEFT,UP,RIGHT,DOWN);
  // leap.enableGesture( Leap::Gesture::TYPE_SCREEN_TAP );
  // leap.enableGesture( Leap::Gesture::TYPE_KEY_TAP );
  // leap.enableGesture( Leap::Gesture::TYPE_CIRCLE );

  if(startRadio(crRadio)) {
    cflieCopter=new CCrazyflie;
    CCrazyflieConstructor(crRadio,cflieCopter);

    //Initialize the thrust value to 36001
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
 
    // Loop until we exit the program
    while (1) {}

    // Failure to find dongle
  } else {
    printf("%s\n", "Could not connect to dongle. Did you plug it in?");
  }
  return 0;
}


