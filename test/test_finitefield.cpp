/*
 * test_finitefield.cpp
 *
 *  Created on: 21.10.2021
 *      Author: andre
 */

#include "FiniteField.h"
#include "gtest/gtest.h"

TEST(FiniteFieldTest, PrimitiveElementPow)
{
	auto primElem = FiniteField7::getPrimitiveElement();

	for (int i = -3; i <= 3; i++)
	{
		std::cout << i << " " << FiniteField7::getPrimitiveElementPow(i) << std::endl;
	}
	ASSERT_EQ(FiniteField7::getPrimitiveElementPow(-2), FiniteField7(1) / (primElem * primElem));
	ASSERT_EQ(FiniteField7::getPrimitiveElementPow(-1), FiniteField7(1) / primElem);
	ASSERT_EQ(FiniteField7::getPrimitiveElementPow(0), FiniteField7(1));
	ASSERT_EQ(FiniteField7::getPrimitiveElementPow(1), primElem);
	ASSERT_EQ(FiniteField7::getPrimitiveElementPow(2), primElem * primElem);
	ASSERT_EQ(FiniteField7::getPrimitiveElementPow(3), primElem * primElem * primElem);
}

TEST(FiniteFieldTest, Pow)
{
	auto primElem = FiniteField7::getPrimitiveElement();

	for (int i = -3; i <= 3; i++)
	{
		std::cout << i << " " << FiniteField7::getPrimitiveElementPow(i) << std::endl;
	}
	ASSERT_EQ(primElem.pow(-2), FiniteField7(1) / (primElem * primElem));
	ASSERT_EQ(primElem.pow(-1), FiniteField7(1) / primElem);
	ASSERT_EQ(primElem.pow(0), FiniteField7(1));
	ASSERT_EQ(primElem.pow(1), primElem);
	ASSERT_EQ(primElem.pow(2), primElem * primElem);
	ASSERT_EQ(primElem.pow(3), primElem * primElem * primElem);
}

TEST(FiniteFieldTest, MathRules)
{
	FiniteField19 one(1);
	FiniteField19 zero(0);

	for (int i = 0; i < 19; i++)
	{
		FiniteField19 a(i);

		// ensure existence of unary operator- which is sadly optional but important
		ASSERT_EQ(zero - a, -a);

		for (int j = 0; j < 19; j++)
		{
			FiniteField19 b(j);

			// Commutativity
			ASSERT_EQ(a * b, b * a);
			ASSERT_EQ(a + b, b + a);

			for (int k = 0; k < 19; k++)
			{
				FiniteField19 c(k);
				ASSERT_EQ(a * c + b * c, (b + a) * c); // Distributivity
			}
		}

		if (a != 0)
		{
			ASSERT_EQ(one / (one / a), a);
			ASSERT_EQ(a / a, one);
		}
	}
}
