/*
 * util.h
 *
 *  Created on: 30.10.2021
 *      Author: andre
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <opencv2/opencv.hpp>
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

static cv::Point calculateLineIntercross(cv::Point a_pos, cv::Point a_dir, cv::Point b_pos, cv::Point b_dir)
{
	Matrix<float> linearsolver{
		{static_cast<float>(a_dir.x), static_cast<float>(-b_dir.x), static_cast<float>(b_pos.x - a_pos.x)},
		{static_cast<float>(a_dir.y), static_cast<float>(-b_dir.y), static_cast<float>(b_pos.y - a_pos.y)},
	};

	auto coefficients = linearsolver.gauss_jordan_elim();
	assert(coefficients.size() > 0);

	// linearsolver.print();
	// printf("calculateLineIntercross %d %d %f %d %d\n", a_pos.x, a_pos.y, coefficients.at(0), a_dir.x, a_dir.y);

	cv::Point solution = a_pos + coefficients.at(0) * a_dir;

	return solution;
}

static const cv::Scalar green{0, 255, 0};
static const cv::Scalar black{0, 0, 0};
static const cv::Scalar blue{255, 0, 0};
static const cv::Scalar red{0, 0, 255};
static const cv::Scalar yellow{0, 255, 255};
static const cv::Scalar white{255, 255, 255};

#endif /* UTIL_H_ */
