/*
 * test_polynom.cpp
 *
 *  Created on: 18.10.2021
 *      Author: andre
 */

#include "FiniteField.h"
#include "Polynom.h"
#include "gtest/gtest.h"

TEST(PolynomTest, Compare)
{
	Polynom<FiniteField7> a({1, 2, 3, 4, 5});

	Polynom<FiniteField7> b({1, 2, 3, 4, 5});
	Polynom<FiniteField7> c({1, 2, 3, 4});
	Polynom<FiniteField7> d({3, 2, 3, 4, 5});
	Polynom<FiniteField7> e({1, 2, 3, 4, 1});

	ASSERT_EQ(a, b);
	ASSERT_NE(a, c);
	ASSERT_NE(a, d);
	ASSERT_NE(a, e);
}

TEST(PolynomTest, AsString)
{
	Polynom<int> in({-1, -2, -3, -4, -5});
	ASSERT_EQ(in.getDegree(), 4);
	std::cout << in.asString() << std::endl;
	ASSERT_STREQ(in.asString().c_str(), "-x^4 + -2x^3 + -3x^2 + -4x + -5");
}

TEST(PolynomTest, Multiply)
{
	{
		Polynom<FiniteField7> a({1, 2, 3, 4, 5});
		Polynom<FiniteField7> b({1, 0}); // just x

		Polynom<FiniteField7> expected({1, 2, 3, 4, 5, 0});

		auto result = a * b;
		ASSERT_EQ(result, expected);
		std::cout << result.asString() << std::endl;
		ASSERT_STREQ(result.asString().c_str(), "x^5 + 2x^4 + 3x^3 + 4x^2 + 5x");
	}

	{
		Polynom<int> a({1, 2, 3, 4, 5});
		Polynom<int> b({1, 0}); // just x

		Polynom<int> expected({1, 2, 3, 4, 5, 0});

		auto result = a * b;
		ASSERT_EQ(result, expected);
		std::cout << result.asString() << std::endl;
		ASSERT_STREQ(result.asString().c_str(), "x^5 + 2x^4 + 3x^3 + 4x^2 + 5x");
	}

	{
		Polynom<int> a({1, 2, 3, 4, 5});
		Polynom<int> b({0, -1}); // just -1

		Polynom<int> expected({-1, -2, -3, -4, -5});

		auto result = a * b;
		ASSERT_EQ(result, expected);
		std::cout << result.asString() << std::endl;
		ASSERT_STREQ(result.asString().c_str(), "-x^4 + -2x^3 + -3x^2 + -4x + -5");
	}
}

TEST(PolynomTest, Divide)
{
	{
		Polynom<int> dividend({3, 2, 1});
		Polynom<int> divisor({1, 1}); // x + 1

		Polynom<int> expectedResult({3, -1});
		Polynom<int> expectedRemainder({2});

		auto result = dividend / divisor;
		std::cout << "Result: " << result.first.asString() << std::endl;
		std::cout << "Remainder: " << result.second.asString() << std::endl;

		ASSERT_EQ(result.first, expectedResult);
		ASSERT_EQ(result.second, expectedRemainder);
	}

	{
		Polynom<int> dividend({1, 2, 3, 4, 5});
		Polynom<int> divisor({1, 0}); // just x

		Polynom<int> expectedResult({1, 2, 3, 4});
		Polynom<int> expectedRemainder({5});

		auto result = dividend / divisor;
		std::cout << "Result: " << result.first.asString() << std::endl;
		std::cout << "Remainder: " << result.second.asString() << std::endl;

		ASSERT_EQ(result.first, expectedResult);
		ASSERT_EQ(result.second, expectedRemainder);
	}

	{
		Polynom<int> dividend({1, 2, 3, 4, 5, 0});
		Polynom<int> divisor({1, 0}); // just x

		Polynom<int> expectedResult({1, 2, 3, 4, 5});
		Polynom<int> expectedRemainder({0});

		auto result = dividend / divisor;
		std::cout << "Result: " << result.first.asString() << std::endl;
		std::cout << "Remainder: " << result.second.asString() << std::endl;

		ASSERT_EQ(result.first, expectedResult);
		ASSERT_EQ(result.second, expectedRemainder);
	}

	{
		Polynom<int> dividend({1, 2, 3, 4, 5});
		Polynom<int> divisor({1}); // just 1

		Polynom<int> expectedResult({1, 2, 3, 4, 5});
		Polynom<int> expectedRemainder({0});

		auto result = dividend / divisor;
		std::cout << "Result: " << result.first.asString() << std::endl;
		std::cout << "Remainder: " << result.second.asString() << std::endl;

		ASSERT_EQ(result.first, expectedResult);
		ASSERT_EQ(result.second, expectedRemainder);
	}

	{
		Polynom<int> dividend({1}); // just 1
		Polynom<int> divisor({1});	// just 1

		Polynom<int> expectedResult({1});
		Polynom<int> expectedRemainder({0});

		auto result = dividend / divisor;
		std::cout << "Result: " << result.first.asString() << std::endl;
		std::cout << "Remainder: " << result.second.asString() << std::endl;

		ASSERT_EQ(result.first, expectedResult);
		ASSERT_EQ(result.second, expectedRemainder);
	}

	{
		Polynom<int> dividend({1});	  // just 1
		Polynom<int> divisor({1, 0}); // just x

		Polynom<int> expectedResult({0});
		Polynom<int> expectedRemainder({1});

		auto result = dividend / divisor;
		std::cout << "Result: " << result.first.asString() << std::endl;
		std::cout << "Remainder: " << result.second.asString() << std::endl;

		ASSERT_EQ(result.first, expectedResult);
		ASSERT_EQ(result.second, expectedRemainder);
	}
}

TEST(PolynomTest, Adding)
{
	Polynom<FiniteField7> a({1, 2, 0, 1});
	Polynom<FiniteField7> b({6, 2});

	Polynom<FiniteField7> expected({1, 2, 6, 3});

	auto result = a + b;
	ASSERT_EQ(result, expected);
	std::cout << result.asString() << std::endl;
}
