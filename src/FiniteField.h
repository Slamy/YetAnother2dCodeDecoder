/*
 * FiniteField.h
 *
 *  Created on: 17.10.2021
 *      Author: andre
 */

#ifndef FINITEFIELD_H_
#define FINITEFIELD_H_

#include <array>
#include <cassert>
#include <iostream>

/**
 * Math utility class for numbers in finite fields GF(q) where q is a prime number
 *
 * @tparam prime	Must be a prime number
 */
template <int prime> class FiniteField
{
  private:
	/// Stored number
	int num;

	/// Neutral element for + and -
	static constexpr int addition_neutral{0};

	/// Neutral element for * and /
	static constexpr int multiplication_neutral{1};

	/**
	 * Calculating a reciprocal of an element is cpu intensive. We calculate them once and
	 * store them inside this look up table.
	 */
	static std::array<int, prime> reciprocal;

	/**
	 * The primitive element alpha of this finite field.
	 * Every element of the finite field can be reached using a power of alpha.
	 */
	static FiniteField primitiveElement;

  public:
	/**
	 * Provides access to the primitive element alpha.
	 * @return	alpha
	 */
	static FiniteField getPrimitiveElement()
	{
		return primitiveElement;
	}

	/**
	 * Calculates a power of this FiniteField
	 * @param p		exponent
	 * @return		result of calculation
	 */
	FiniteField pow(int p)
	{
		int pabs = labs(p);

		if (pabs == 0)
		{
			return FiniteField(multiplication_neutral);
		}

		FiniteField val(num);
		pabs--;
		while (pabs)
		{
			val *= *this;
			pabs--;
		}
		return (p > 0) ? val : FiniteField(reciprocal.at(val.num));
	}

	/**
	 * Calculates a power of the primitive element alpha of this FiniteField
	 * @param p		exponent
	 * @return		result of calculation
	 */
	static FiniteField getPrimitiveElementPow(int p)
	{
		FiniteField val(primitiveElement);
		return val.pow(p);
	}

	/**
	 * Brute force method of finding one possible primitive element alpha.
	 * Must be called at the start of the program before usage of \ref primitiveElement
	 * by various functions.
	 */
	static void findPrimitiveElement()
	{
		bool primitiveElementFound{false};

		for (int possible_primitive = 0; possible_primitive < prime; possible_primitive++)
		{
			std::array<bool, prime> found{false};
			int elementsfound = 0;
			FiniteField current(possible_primitive);
			// printf("Try %d : ", possible_primitive);
			for (int i = 0; i < prime; i++)
			{
				// printf("%d ", current);
				if (found.at(current) == false)
					elementsfound++;
				found.at(current) = true;
				current			  = current * FiniteField(possible_primitive);
			}
			if (elementsfound == prime - 1)
			{
				// printf("Primitive Element %d\n", possible_primitive);
				primitiveElement = FiniteField(possible_primitive);

				assert(getPrimitiveElementPow(prime) == primitiveElement);
				assert(getPrimitiveElementPow(prime - 1) == 1);

				primitiveElementFound = true;
				break;
			}
			// printf("\n");
		}

		assert(primitiveElementFound);
	}

	/**
	 * Brute force method to build a reciprocal lookup table. Must be called at the start of the program
	 * to make division and negative exponents with \ref pow possible.
	 */
	static void buildReciprocal()
	{
		for (int i = 0; i < prime; i++)
		{
			bool reciprocalFound{false};

			for (int j = 0; j < prime; j++)
			{
				FiniteField a(i);
				FiniteField b(j);

				// printf("%d ",(a * b));

				if ((a * b) == multiplication_neutral)
				{
					// printf("%d * %d == 1\n", i, j);
					assert(!reciprocalFound);
					reciprocal.at(i) = j;
					reciprocalFound	 = true;
				}
			}

			// printf("\n");
			assert(reciprocalFound || (i == 0));
		}
	}

	FiniteField()
	{
		num = addition_neutral;
	}

	/**
	 * Construct from an integer
	 * @param val	start value
	 */
	FiniteField(int val)
	{
		num = val;

		if (num < 0)
		{
			num += prime;
		}
		assert(num >= 0);
		assert(num < prime);
	}

	/**
	 * Provide access to the internal value for printing and other means.
	 */
	operator int() const
	{
		return num;
	}

	/**
	 * Overloaded compare function
	 * @param lhs	left hand side
	 * @param rhs	right hand side
	 * @return		true if lhs is greater
	 */
	friend bool operator>(const FiniteField& lhs, const FiniteField& rhs)
	{
		return lhs.num > rhs.num;
	}

	/**
	 * Overloaded unary negative operator
	 * @return	negative of current value
	 */
	FiniteField operator-() const
	{
		return -num;
	}

	/**
	 * Add two terms together
	 * @param lhs	Left term
	 * @param rhs	Right term
	 * @return		Sum
	 */
	friend FiniteField operator+(FiniteField lhs, const FiniteField& rhs)
	{
		assert(lhs.num < prime);
		assert(rhs.num < prime);

		lhs.num = (lhs.num + rhs.num) % prime;

		assert(lhs.num < prime);
		return lhs; // return the result by value (uses move constructor)
	}

	/**
	 * Increment by a term
	 * @param rhs	term
	 * @return		reference to the result.
	 */
	FiniteField& operator+=(const FiniteField& rhs)
	{
		assert(rhs.num < prime);
		num = (num + rhs.num) % prime;

		return *this; // return the result by reference
	}

	/**
	 * Perform multiplication with a factor and store the result
	 * @param rhs	factor
	 * @return		reference to the result.
	 */
	FiniteField& operator*=(const FiniteField& rhs)
	{
		assert(num < prime);
		assert(rhs.num < prime);

		num = (num * rhs.num) % prime;

		assert(num < prime);
		return *this;
	}

	/**
	 * Perform subtraction
	 * @param lhs	minuend
	 * @param rhs	subtrahend
	 * @return		difference
	 */
	friend FiniteField operator-(FiniteField lhs, const FiniteField& rhs)
	{
		assert(lhs.num < prime);
		assert(rhs.num < prime);

		lhs.num = (lhs.num - rhs.num);
		// printf("num %d\n",lhs.num);
		while (lhs.num < 0)
		{
			lhs.num += prime;
			// printf("num2 %d\n",lhs.num);
		}

		assert(lhs.num < prime);
		return lhs; // return the result by value (uses move constructor)
	}

	/**
	 * Perform multiplication
	 * @param lhs	left factor
	 * @param rhs	right factor
	 * @return		product
	 */
	friend FiniteField operator*(FiniteField lhs, const FiniteField& rhs)
	{
		assert(lhs.num < prime);
		assert(rhs.num < prime);

		lhs.num = (lhs.num * rhs.num) % prime;

		assert(lhs.num < prime);
		return lhs; // return the result by value (uses move constructor)
	}

	/**
	 * Perform division
	 * @param lhs	dividend
	 * @param rhs	divisor
	 * @return		Result which never has a remainder
	 */
	friend FiniteField operator/(FiniteField lhs, const FiniteField& rhs)
	{
		assert(rhs.num != 0);
		lhs.num = (lhs.num * reciprocal[rhs.num]) % prime;
		assert(lhs.num < prime);
		return lhs; // return the result by value (uses move constructor)
	}
};

template <int prime> std::array<int, prime> FiniteField<prime>::reciprocal;
template <int prime> FiniteField<prime> FiniteField<prime>::primitiveElement{0};

using FiniteField7	= FiniteField<7>;
using FiniteField19 = FiniteField<19>;

#endif /* FINITEFIELD_H_ */
