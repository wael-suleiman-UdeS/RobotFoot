#include <iostream>
#include "test.h"

int main(int argc, char* argv[]) {
	int cReturn = EXIT_SUCCESS;
	
	test conception;
	cReturn = conception.cameraClibration(argc,argv);

	return cReturn;
}