/*
 * DB_string.c
 *
 *  Created on: 27 cze 2018
 *      Author: DBabraj
 */

/* Includes (zalaczone biblioteki) -----------------------------------------------------------------------------------*/
#include "DB_string.h"

#include <math.h> // dla powf()

// C program for implementation of ftoa()
// https://www.geeksforgeeks.org/convert-floating-point-number-string/

/* Preprocessor definition (definicje preprocesora) ------------------------------------------------------------------*/

/* Types definition (definicje typow) --------------------------------------------------------------------------------*/


/* Const declarations (deklaracje stalych) ---------------------------------------------------------------------------*/

/* Variable declarations (deklaracje zmiennych) ----------------------------------------------------------------------*/


/* Function declarations (deklaracje funkcji) ------------------------------------------------------------------------*/
static void reverse(char *str, int len);
static int intToStr(int x, char str[], int d);

/* Function definition (definicje funkcji) ---------------------------------------------------------------------------*/


/**
 * @brief Converts a floating point number to string.
 * @param n: Floating number.
 * @param *res: Out buffer.
 * @param afterpoint: The number of digits required in output.
 * @retval none.
 */
void ftoa_fast(float n, char *res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * powf(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}


//
/**
 * @brief Reverses a string 'str' of length 'len'
 * @param *str:	Pointer to string.
 * @param len:	String length.
 * @retval none
 */
static void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}


/**
 * @brief Converts a given integer x to string str[].
 * @param x:	Integer number.
 * @param str[]: String.
 * @param d:	d is the number of digits required in output.
 * 				If d is more than the number of digits in x, then 0s are added at the beginning.
 * @retval int
 */
static int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}
