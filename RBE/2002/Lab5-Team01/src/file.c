#include <stdio.h>
#include <stdlib.h>

int main()
{
    int x = "hello";          // Type error: assigning string to int
    float y                   // Missing semicolon
    char* str = 123;          // Type error: assigning int to char pointer
    printf("Value of x: %d\n", y); // Wrong variable and type mismatch
    if(x = 5)                 // Assignment instead of comparison
        printf("x is 5\n");  
    else
        printf("x not 5\n")

    for(int i = 0; i < 10; i++) // Missing braces, potential scope issue
        printf("i = %d\n", i)

    int* ptr;
    *ptr = 10;               // Dereferencing uninitialized pointer

    return 0                  // Missing semicolon
}