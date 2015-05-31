#include "util.h"

//See http://supertech.csail.mit.edu/papers/debruijn.pdf, paper explains nicely how it works (it's very simple!)
//http://en.wikipedia.org/wiki/Find_first_set#FFS
#include <stdio.h>

#define DEBRUIJN32 0x077CB531UL  //debruijn32 = 0000 0111 0111 1100 1011 0101 0011 0001 
#define DEBRUIJN_RSHIFT (32 - 5) //each value extracted from the constant is 5 bits

unsigned int DEBRUIJN32_TABLE[32] = 
{ 0,  1,  28,  2,  29,  14,  24,  3, 
30, 22,  20, 15,  25,  17,   4,  8, 
31, 27,  13, 23,  21,  19,  16,  7, 
26, 12,  18,  6,  11,   5,  10,  9,};


//finds the index of the single bit which is set. Won't work if >1 bit is set
unsigned int find_first_set_single_one(unsigned int single_one)
{
	//Index into the debrujin sequence using the single_one 
	unsigned int index = (DEBRUIJN32 * single_one) >> DEBRUIJN_RSHIFT;
	
	//use a lookup table to extract the final value
	return DEBRUIJN32_TABLE[index];
}

//finds the index of the first one, starting from the right
unsigned int find_first_set(unsigned int num)
{
	//get only the rightmost 1 
	//(to convert a + num to a - num in 2's complement, invert and add one. after ANDING only the last bit will remain)
	unsigned int single_one = (unsigned int) (num & -num);
	
	return find_first_set_single_one(single_one);
}
