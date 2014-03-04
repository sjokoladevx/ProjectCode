

#include <stdio.h>
#include <unistd.h>
#include <stdarg.h>

#define NUMELEMENTS(x)  (sizeof(x) / sizeof(x[0]))
#define CHECKFIRST(x)  if (x[0] != 0){printf("macro already exists\n");return 0;}

#define CIRCLE 0
#define KEYTAP 1
#define SCREENTAP 2

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

/*TODO:

1.Implement macro mode: read from arrray using flyMotion
2.Read Gestures to start macro mode

*/

//insert into main_control
case GESTURE_STATE:
	printf( "gesture state\n" );
	      // If sig is normal, keep flying
	if ( current_signal == GESTURE_SIG ) {
		int g = 0;
		int sleeper = 0;
		while(macros[gesture][g] != 0){
			singleMotion( cflieCopter, macros[gesture][g]);
			if (sleeper++ % 100){g++;}
		}
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

		case SUPERSUPERDOWN:
		setThrust( cflieCopter, 17001);
		break;

		case SUPERFORWARD:
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

		return 1;

	}


//cite: http://www.learncpp.com/cpp-tutorial/714-ellipses-and-why-to-avoid-them/

	int macros[2][20];

	int createMoveMacro(int args, ... ){

	//accounting for the gesture param
		args = args - 1;

		va_list list;
		va_start(list, args);

		int gesture = va_arg(list, int);
		CHECKFIRST(macros[gesture]);

		for (int i=0; i < args; i++){
			macros[gesture][i] = va_arg(list,int);
		}

// Cleanup the va_list when we're done.
		va_end(list);
		return 1;
	}



	int main(){

		createMoveMacro(6,CIRCLE,FORWARD,LEFT,UP,RIGHT,DOWN);

	}

