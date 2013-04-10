//-------------------------------------------------------------------------
// Example.c
//   simple example program to demonstrate Avrora AVR simulation
//-------------------------------------------------------------------------

//-------------------------------------------------------------------------
// GLOBALS: two character variables (one byte each)
//-------------------------------------------------------------------------
char i = 9;
char j = 6;
int foo();

//-------------------------------------------------------------------------
// main()
//   entrypoint of program
//-------------------------------------------------------------------------
int main() {

  int i;

  // call the foo() method a number of times
  for ( i = 0; i < 5; i++ )
    j = foo();

  // terminate simulation here with break instruction
  asm volatile ("break");

  return j;
}

//-------------------------------------------------------------------------
// foo()
//   multiply j by 13 and return it
//-------------------------------------------------------------------------
int foo() {
  int k = 13 * j;
  return k;
}
