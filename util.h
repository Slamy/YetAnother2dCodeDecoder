/*
 * util.h
 *
 *  Created on: 30.10.2021
 *      Author: andre
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <stdio.h>

/**
 * Print a 32 bit value as a binary representation
 * @param x	value
 */
static void bin_prnt_byte(int x)
{
	int n;
	for (n = 0; n < 32; n++)
	{
		if ((x & 0x80000000) != 0)
		{
			printf("1");
		}
		else
		{
			printf("0");
		}
		x = x << 1;
	}
	printf("\n");
}

static int hammingDistance(uint16_t a, uint16_t b)
{
	int distance = 0;
	for (int i = 0; i < 16; i++)
	{
		if ((a ^ b) & 1)
			distance++;
		a >>= 1;
		b >>= 1;
	}
	return distance;
}
#endif /* UTIL_H_ */
