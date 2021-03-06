/*
 * test_main.cpp
 *
 *  Created on: 19.10.2021
 *      Author: andre
 */

#include "ExtFiniteField256.h"
#include "FiniteField.h"
#include "gtest/gtest.h"

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);

	FiniteField7::buildReciprocal();
	FiniteField7::findPrimitiveElement();

	FiniteField19::buildReciprocal();
	FiniteField19::findPrimitiveElement();

	FFieldQr::buildReciprocal();
	FFieldQr::findPrimitiveElement();

	FFieldDm::buildReciprocal();
	FFieldDm::findPrimitiveElement();

	return RUN_ALL_TESTS();
}
