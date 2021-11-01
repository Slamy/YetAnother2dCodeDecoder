/*
 * scanner.cpp
 *
 *  Created on: 28.10.2021
 *      Author: andre
 */

#include "QrDecoder.h"

#include "Matrix.h"
#include "util.h"
#include <algorithm>
#include <glm/vec2.hpp>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <vector>

/*
 * https://stackoverflow.com/questions/10344246/how-can-i-convert-a-cvmat-to-a-gray-scale-in-opencv
 * https://stackoverflow.com/questions/7899108/opencv-get-pixel-channel-value-from-mat-image
 */
int main(int argc, char** argv)
{
	ExtFiniteField256::buildReciprocal();
	ExtFiniteField256::findPrimitiveElement();

	QrDecoder qr;

	// image = cv::imread("/home/andre/GIT/QrCode/Japan-qr-code-billboard.jpg", 1);
	// image = cv::imread("/home/andre/GIT/QrCode/QR_deWP.svg.png", 1);
	// qr.decodeFromFile("/home/andre/GIT/QrCode/testcode_wikipedia_perspective.png");
	// qr.decodeFromFile("/home/andre/GIT/QrCode/testcode_ArminHanisch.png");
	qr.decodeFromFile("/home/andre/GIT/QrCode/testcode_wikipedia.png");

	return 0;
}
