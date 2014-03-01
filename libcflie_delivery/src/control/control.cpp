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
//#define *other states*
//#define the states you will use here

//CS50_TODO:  define your own signals
//Define All the signals
#define NO_SIG 11//Each time when our state machine process the current signal
//A good practice is that when the current signal is processd, set the current signal variable back to "no signal"
#define CHANGE_HOVER_SIG 12//Use to transit state between Normal and Hover
#define TIME_OUT_SIG 13
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
float current_thrust;
float current_roll;
float current_pitch;
float current_velocity;

//The pointer to the crazy flie data structure
CCrazyflie *cflieCopter=NULL;

//CS50_TODO:  define other helper function here
//In normal state, the flyNormal function will be called, set different parameter
//In hover state, different function should be called, because at that time, we should set the thrust as a const value(32767), see reference 
//(http://forum.bitcraze.se/viewtopic.php?f=6&t=523&start=20)


//The helper functions 
void flyNormal(CCrazyflie *cflieCopter){
    setThrust( cflieCopter, 38500 + current_thrust * ( 5.0 - batteryLevel(cflieCopter) ) );
    setPitch( cflieCopter, current_pitch );
    setRoll ( cflieCopter, current_roll );
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
        
    for (int i = 0; i < leap_frame_hands_count(frame); i++) {
        leap_hand_ref hand = leap_frame_hand_at_index(frame, i);
        leap_vector velocity;
	leap_vector position;
	leap_vector direction;

        leap_hand_palm_velocity(hand, &velocity);
	leap_hand_direction(hand, &direction);
	leap_hand_palm_position(hand, &position);

	current_thrust = position.y * 15.0;
	current_velocity = velocity.x;
	current_roll = direction.x * 10.0;
	current_pitch = direction.y * 10.0;
	printf("%f %f %f\n", current_thrust, current_roll, current_pitch);
	
        //CS50_TODO 
        //*pseudocode*
        /*
            The above code show a simple example on get the velocity infomarion of the hand
            
            You need to find headers in the Leap folder, see what function you can call to 
            examine the hand's parameter

            My personal solution use the orientation of hand to compuate the get pitch and roll
            use the position of hand to get the thrust
            Use a "swipe" gesture to enable/disable the hover mode 

            you should also send out the signals in this function:

            if velocity.x > some value
              set current state to hover
            end

            if direction.x > some value
              set the pitch with some value
            end

            This function send out signal by set the current_signal variable

            This function should only send one signal each time, and only send signals when the last signal has been processed

            Recall that after the commander thread will set the current_signal back to NO_SIG, so we will only send out signal if we find this variable is equal to NO_SIG

            You may use lock to lock the global variable you use to synchronize the two thread
        */
        //*pseudocode* 
    }
    leap_frame_release(frame);
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

//This thread will check the current state and send corrsponding command to the copter
void* main_control(void * param){
  CCrazyflie *cflieCopter=(CCrazyflie *)param;

  while(cycle(cflieCopter)) {

    setThrust( cflieCopter, 38500 + current_thrust * ( 5.0 - batteryLevel(cflieCopter) ) );    
    setPitch( cflieCopter, current_pitch );
    setRoll ( cflieCopter, current_roll );

  }  
  
      //transition depend on the current state
        //CS50_TODO : depend on the current signal and current state, you can call the helper function here to control the copter
        //switch(current_signal){
            //}
        
  
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
  CCrazyRadioConstructor(crRadio,"radio://0/10/250K");
  
  if(startRadio(crRadio)) {
    cflieCopter=new CCrazyflie;
    CCrazyflieConstructor(crRadio,cflieCopter);

    //Initialize the set value
    setThrust(cflieCopter,0);
    
    
    // Enable sending the setpoints. This can be used to temporarily
    // stop updating the internal controller setpoints and instead
    // sending dummy packets (to keep the connection alive).
    setSendSetpoints(cflieCopter,true);


    //CS50_TODO
    //Do initialization here
    //And set up the threads
    pthread_t lThread;
    pthread_t mainThread;
    pthread_create(&lThread, NULL, leap_thread, NULL); 
    pthread_create(&mainThread, NULL, main_control, cflieCopter);
    //pthread_exit(NULL);
    while (1) {}

  } else {
    printf("%s\n", "Could not connect to dongle. Did you plug it in?");
  }
  return 0;
}
