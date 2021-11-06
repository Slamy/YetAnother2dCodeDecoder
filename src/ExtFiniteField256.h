/*
 * ExtFiniteField256.h
 *
 *  Created on: 23.10.2021
 *      Author: andre
 */

#ifndef EXTFINITEFIELD256_H_
#define EXTFINITEFIELD256_H_

#include <array>
#include <cassert>
#include <iostream>

/**
 * Based on https://en.wikiversity.org/wiki/Reed%E2%80%93Solomon_codes_for_coders
 */
class ExtFiniteField256
{
  private:
	/// irreducible polynomial used to reduce a number to the field space
	static constexpr int irreducible_primitive_polynomial = 0x11d; // GF(2^8) for QR codes

	/// Stored number
	uint8_t num;

	/// Neutral element for + and -
	static constexpr int addition_neutral{0};

	/// Neutral element for * and /
	static constexpr int multiplication_neutral{1};

	/**
	 * Calculating a reciprocal of an element is cpu intensive. We calculate them once and
	 * store them inside this look up table.
	 */
	static std::array<uint8_t, 256> reciprocal;

	/**
	 * The primitive element alpha of this finite field.
	 * Every element of the finite field can be reached using a power of alpha.
	 */
	static ExtFiniteField256 primitiveElement;

  public:
	/**
	 * Calculates a power of this FiniteField
	 * @param p		exponent
	 * @return		result of calculation
	 */
	ExtFiniteField256 pow(int p)
	{
		int pabs = labs(p);

		if (pabs == 0)
		{
			return ExtFiniteField256(multiplication_neutral);
		}

		ExtFiniteField256 val(num);
		pabs--;
		while (pabs)
		{
			val *= *this;
			pabs--;
		}
		return (p > 0) ? val : ExtFiniteField256(reciprocal.at(val.num));
	}

	/**
	 * Calculates a power of the primitive element alpha of this FiniteField
	 * @param p		exponent
	 * @return		result of calculation
	 */
	static ExtFiniteField256 getPrimitiveElementPow(int p)
	{
		ExtFiniteField256 val(primitiveElement);
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

		for (int possible_primitive = 0; possible_primitive < 256; possible_primitive++)
		{
			std::array<bool, 256> found{false};
			int elementsfound = 0;
			ExtFiniteField256 current(possible_primitive);
			// printf("Try %d : ", possible_primitive);
			for (int i = 0; i < 256; i++)
			{
				// printf("%d ", current);
				if (found.at(current) == false)
					elementsfound++;
				found.at(current) = true;
				current			  = current * ExtFiniteField256(possible_primitive);
			}
			if (elementsfound == 256 - 1)
			{
				printf("Primitive Element %d\n", possible_primitive);
				primitiveElement = ExtFiniteField256(possible_primitive);

				assert(getPrimitiveElementPow(256) == primitiveElement);
				assert(getPrimitiveElementPow(256 - 1) == 1);

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
		for (int i = 0; i < 256; i++)
		{
			bool reciprocalFound{false};

			for (int j = 0; j < 256; j++)
			{
				ExtFiniteField256 a(i);
				ExtFiniteField256 b(j);

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

	ExtFiniteField256()
	{
		num = addition_neutral;
	}

	/**
	 * Construct from a byte
	 * @param val	start value
	 */
	ExtFiniteField256(uint8_t val)
	{
		num = val;
	}

	/**
	 * Add two terms together which means XORing them.
	 * @param lhs	Left term
	 * @param rhs	Right term
	 * @return		Sum
	 */
	friend ExtFiniteField256 operator+(ExtFiniteField256 lhs, const ExtFiniteField256& rhs)
	{
		lhs.num ^= rhs.num;
		return lhs; // return the result by value (uses move constructor)
	}

	/**
	 * Perform subtraction which is the same as addition for this finite field
	 * @param lhs	minuend
	 * @param rhs	subtrahend
	 * @return		difference
	 */
	friend ExtFiniteField256 operator-(ExtFiniteField256 lhs, const ExtFiniteField256& rhs)
	{
		lhs.num ^= rhs.num;
		return lhs; // return the result by value (uses move constructor)
	}

	/**
	 * Overloaded unary negative operator. As adding and subtracting is the same, the negative is the identity.
	 * @return	negative of current value which is the same
	 */
	ExtFiniteField256 operator-() const
	{
		return *this;
	}

	/**
	 * Increment by a term which is internally just XOR.
	 * @param rhs	term
	 * @return		reference to the result.
	 */
	ExtFiniteField256& operator+=(const ExtFiniteField256& rhs)
	{
		num ^= rhs.num;
		return *this; // return the result by reference
	}

	/**
	 * Perform polynomial multiplication with XORing the end result.
	 * Every bit is assumed to be a coefficient of either 0 or 1.
	 *
	 * @param x	factor
	 * @param y	factor
	 * @return	polynomial product
	 */
	static int xorMult(int x, int y)
	{
		int z = 0;
		int i = 0;
		while ((y >> i) > 0)
		{
			if (y & (1 << i))
				z ^= x << i;
			i += 1;
		}
		return z;
	}

	/**
	 * Provide index of highest set bit in value
	 * Example 0b -> 1 or 0b100 -> 3
	 *
	 * @param n	Value
	 * @return	index starting with one
	 */
	static int bit_length(int n)
	{
		int bits = 0;
		while (n)
		{
			n >>= 1;
			bits++;
		}
		return bits;
	}

	/**
	 * Perform binary polynomial division and provide only the remainder.
	 *
	 * @param dividend	Dividend
	 * @param divisor	Divisor
	 * @return			Remainder
	 */
	static int xorDiv(int dividend, int divisor)
	{
		int dl1 = bit_length(dividend);
		int dl2 = bit_length(divisor);

		if (dl1 < dl2)
			return dividend;

		int steps = dl1 - dl2;

		for (int i = steps; i >= 0; i--)
		{
			// bin_prnt_byte(dividend);
			// bin_prnt_byte((1 << (i + dl2-1)));
			// bin_prnt_byte(divisor << i);

			if (dividend & (1 << (i + dl2 - 1)))
			{
				dividend ^= divisor << i;
			}
		}

		return dividend;
	}

	/**
	 * Perform multiplication
	 * @param lhs	left factor
	 * @param rhs	right factor
	 * @return		product
	 */
	friend ExtFiniteField256 operator*(ExtFiniteField256 lhs, const ExtFiniteField256& rhs)
	{
		int result = xorMult(lhs.num, rhs.num);
		result	   = xorDiv(result, irreducible_primitive_polynomial);
		return result; // return the result by value (uses move constructor)
	}

	/**
	 * Perform multiplication with a factor and store the result
	 * @param rhs	factor
	 * @return		reference to the result.
	 */
	ExtFiniteField256& operator*=(const ExtFiniteField256& rhs)
	{
		int result = xorMult(num, rhs.num);
		result	   = xorDiv(result, irreducible_primitive_polynomial);

		num = result;

		return *this;
	}

	/**
	 * Perform division
	 * @param lhs	dividend
	 * @param rhs	divisor
	 * @return		Result which never has a remainder
	 */
	friend ExtFiniteField256 operator/(ExtFiniteField256 lhs, const ExtFiniteField256& rhs)
	{
		int result = xorMult(lhs.num, reciprocal.at(rhs.num));
		result	   = xorDiv(result, irreducible_primitive_polynomial);
		return result; // return the result by value (uses move constructor)
	}

	/**
	 * Provide access to the internal value for printing and other means.
	 */
	operator int() const
	{
		return num;
	}
};

#endif /* EXTFINITEFIELD256_H_ */
