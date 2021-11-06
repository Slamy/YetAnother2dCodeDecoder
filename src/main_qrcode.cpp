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

	cv::Point2f a(2, 0);
	cv::Point2f b(2, 0);
	std::cout << a.dot(b) << std::endl;

	// return 0;
	QrDecoder qr;
	qr.debugMode = true;

	// qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/schild_qrcode.jpg");
	qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/qr-code.png");
	// qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/Qr-projekt-taunusanlage-beethoven-denkmal-2011-ffm-029.jpg");
	// qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/numeric3.png");
	// qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/testcode_car.jpg");
	// qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/testcode_car2.png");
	// qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/testcode_weird.png");
	// qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/Japan-qr-code-billboard.jpg");

	// qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/testcode_wikipedia_perspective.png");
	// qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/testcode_ArminHanisch.png");
	//	qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/testcode_wikipedia.png");

	return 0;
}
