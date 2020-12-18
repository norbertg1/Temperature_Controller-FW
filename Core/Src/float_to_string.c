
// C program for implementation of ftoa() 
#include "float_to_string.h"
  
// Reverses a string 'str' of length 'len' 
void reverse(char* str, int len) 
{ 
    int i = 0, j = len - 1, temp; 
    while (i < j) { 
        temp = str[i]; 
        str[i] = str[j]; 
        str[j] = temp; 
        i++; 
        j--; 
    } 
} 
  
// Converts a given integer x to string str[].  
// d is the number of digits required in the output.  
// If d is more than the number of digits in x,  
// then 0s are added at the beginning. 
int intToStr(int x, char str[], int d) 
{ 
    int i = 0;
    if(x == 0) {
    	str[0] = '0';
    	i++;
    }
    while (x) { 
        str[i++] = (x % 10) + '0'; 
        x = x / 10; 
    } 
  
    // If number of digits required is more, then 
    // add 0s at the beginning 
    while (i < d) 
        str[i++] = '0'; 
  
    reverse(str, i); 
    str[i] = '\0'; 
    return i; 
} 
  
// Converts a floating-point/double number to a string. 
int ftoa(float n, char* res, int afterpoint)
{ 
    // Extract integer part 
    short sign=0;
	/*if(n<0){
    	res[0]=45;	//The code on font table for "-" sign
    	sign = 1;
    }*/
	n=fabs(n);
	int ipart = (int)n;
  
    // Extract floating part 
    float fpart = n - (float)ipart; 
    float fff = pow(10,(float)afterpoint);
    // convert integer part to string 
    int i = intToStr(ipart, res + sign, 0);
  
    // check for display option after point 
    if (afterpoint != 0) { 
        res[i+sign] = '.'; // add dot
  
        // Get the value of fraction part upto given no. 
        // of points after dot. The third parameter  
        // is needed to handle cases like 233.007 
        float ff = pow(10, (float)afterpoint);
        fpart = round((fpart * ff)*1000)/1000;
  
        intToStr((int)fpart, res + i + 1 + sign, afterpoint);
    }
    return sign + i + afterpoint;
}
