/*
 * test_finitefield.cpp
 *
 *  Created on: 21.10.2021
 *      Author: andre
 */

#include "ExtFiniteField256.h"
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

template <class FF> void CheckMathRules(int fieldsize)
{
	FF one(1);
	FF zero(0);

	for (int i = 0; i < fieldsize; i++)
	{
		FF a(i);

		// ensure existence of unary operator- which is sadly optional but important
		ASSERT_EQ(zero - a, -a);

		for (int j = 0; j < fieldsize; j++)
		{
			FF b(j);

			// Commutativity
			ASSERT_EQ(a * b, b * a);
			ASSERT_EQ(a + b, b + a);

			for (int k = 0; k < fieldsize; k++)
			{
				FF c(k);
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

TEST(FiniteFieldTest, MathRules)
{
	CheckMathRules<FiniteField19>(19);
	CheckMathRules<FiniteField7>(7);
	CheckMathRules<ExtFiniteField256>(256);
}
