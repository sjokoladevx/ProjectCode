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

// LEAP MOTION CALLBACK FUNCTIONS
void on_init(leap_controller_ref controller, void *user_info)
{
  leap_gesture_type gesture = LEAP_GESTURE_TYPE_CIRCLE;
  leap_controller_enable_gesture(controller, gesture, 1);
  leap_gesture_type gesture1 = LEAP_GESTURE_TYPE_SWIPE;
  leap_controller_enable_gesture(controller, gesture1, 1);
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

// This function is called every time the leap detects motion
void on_frame( leap_controller_ref controller, void *user_info )
{
  leap_frame_ref frame = leap_controller_copy_frame( controller, 0 );
  leap_vector velocity;
  leap_vector position;
  leap_vector direction;


    for ( int i = 0; i < leap_frame_gestures_count( frame ); i++ ) {

      leap_gesture_ref gesture = leap_frame_gesture_at_index( frame, i );

      switch (leap_gesture_gesture_type(gesture)){

      case LEAP_GESTURE_TYPE_CIRCLE:
       printf("CIRCLE GESTURE\n");
       leap_frame_release(frame);
       return;

      case LEAP_GESTURE_TYPE_SWIPE:
       printf("SWIPE GESTURE\n");
       leap_frame_release(frame);
       return;

       default: 
       printf("other type: %d", leap_gesture_gesture_type(gesture));
       leap_frame_release(frame);
       return;
      }
    }

 leap_frame_release(frame);
  return;

}

// This the leap motion thread and control callback function
// It calls the on_frame function when it senses movement
void* leap_thread( void * param ) {
  struct leap_controller_callbacks callbacks;
  callbacks.on_init = on_init;
  callbacks.on_connect = on_connect;
  callbacks.on_disconnect = on_disconnect;
  callbacks.on_exit = on_exit;
  callbacks.on_frame = on_frame;
  leap_listener_ref listener = leap_listener_new(&callbacks, NULL);
  leap_controller_ref controller = leap_controller_new();
  leap_controller_add_listener(controller, listener);
  while( 1 );
}


//This this the main function, use to set up the radio and init the copter
int main( int argc, char **argv ) {

    pthread_t leapThread;
    pthread_create( &leapThread, NULL, leap_thread, NULL ); 

    // Loop until we exit
    while ( 1 ) {}

    // Failure to find dongle
  
  return 0;
}
