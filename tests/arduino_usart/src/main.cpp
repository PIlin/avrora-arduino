#include <Arduino.h>

// #define S Serial
#define S Serial1


void setup() {

	S.begin(9600);

}

void loop() {
	S.write(0x42);
}