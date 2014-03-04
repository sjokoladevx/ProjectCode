

#include <stdio.h>
#include <unistd.h>
#include <cstdarg>

#define NUMELEMENTS(x)  (sizeof(x) / sizeof(x[0]))


//cite: http://www.learncpp.com/cpp-tutorial/714-ellipses-and-why-to-avoid-them/

int macros[4][];

void createMoveMacro(int args, int gesture, ...){
  if(NUMELEMENTS(macros[gesture]) == 0){ return 0;}

 // We access the ellipses through a va_list, so let's declare one
    va_list list;
 
    // We initialize the va_list using va_start.  The first parameter is
    // the list to initialize.  The second parameter is the last non-ellipse
    // parameter.
    va_start(list, args);

    for (int i=0; i < args; i++){
    	printf("%d\n",va_arg(list,int));
    	}

// Cleanup the va_list when we're done.
    va_end(list);
 
}

int main(){

	createMoveMacro(5, 0, 1,2,3,4,5);

}