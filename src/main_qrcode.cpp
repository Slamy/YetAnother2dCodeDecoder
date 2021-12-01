/**
 * @file scanner.cpp
 *
 *  Created on: 28.10.2021
 *      Author: andre
 */

#include "QrDecoder.h"

#include "DataMatrixDecoder.h"
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
	FFieldQr::buildReciprocal();
	FFieldQr::findPrimitiveElement();

	FFieldDm::buildReciprocal();
	FFieldDm::findPrimitiveElement();

	QrDecoder qr;
	qr.debugMode	   = true;
	qr.debugModeVisual = true;

	DataMatrixDecoder dm;
	dm.debugMode	   = true;
	dm.debugModeVisual = true;

	// qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/schild_qrcode.jpg");

	// qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/Qr-projekt-taunusanlage-beethoven-denkmal-2011-ffm-029.jpg");
	// auto ret = qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/numeric3.png");
	// qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/testcode_car.jpg");
	// qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/testcode_car2.png");
	// qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/testcode_weird.png");
	// qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/Japan-qr-code-billboard.jpg");
	// qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/testcode_wikipedia_perspective.png");
	// qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/testcode_ArminHanisch.png");
	// qr.decodeFromFile("/home/andre/GIT/QrCode/qrcode_ref/wikipedia_rot180.png");

	auto ret = dm.decodeFromFile("/home/andre/GIT/QrCode/datamatrix_ref/wikipedia.png");
	// auto ret = dm.decodeFromFile("/home/andre/GIT/QrCode/datamatrix_ref/wikipedia_perspective.png");
	// auto ret = dm.decodeFromFile("/home/andre/GIT/QrCode/datamatrix_ref/datamatrix_link_en_big.png");
	// auto ret = dm.decodeFromFile("/home/andre/GIT/QrCode/datamatrix_ref/datamatrix_link_en.png");
	// auto ret = dm.decodeFromFile("/home/andre/GIT/QrCode/datamatrix_ref/gen_abc.png");
	// auto ret = dm.decodeFromFile("/home/andre/GIT/QrCode/datamatrix_ref/gen_abcd.png");
	// auto ret = dm.decodeFromFile("/home/andre/GIT/QrCode/datamatrix_ref/gen_a2z.png");

	ret.push_back('\0');
	printf("Payload: %s", ret.data());
	return 0;
}
