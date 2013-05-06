#include <Arduino.h>

// #define S Serial
#define S Serial1


void setup() {

	S.begin(9600);

}

void loop() {

	while (S.available())
	{
		uint8_t d = S.read();
		S.write(d);
	}
}
