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
	static constexpr int irreducible_primitive_polynomial = 0x11d; // GF(2^8) for QR codes
	uint8_t num;
	static constexpr int addition_neutral{0};
	static constexpr int multiplication_neutral{1};

	static std::array<uint8_t, 256> reciprocal;
	static ExtFiniteField256 primitiveElement;

  public:
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

	static ExtFiniteField256 getPrimitiveElementPow(int p)
	{
		ExtFiniteField256 val(primitiveElement);
		return val.pow(p);
	}

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
				// printf("Primitive Element %d\n", possible_primitive);
				primitiveElement = ExtFiniteField256(possible_primitive);

				assert(getPrimitiveElementPow(256) == primitiveElement);
				assert(getPrimitiveElementPow(256 - 1) == 1);

				primitiveElementFound = true;
				break;
			}
			// printf("\n");
		}

		// exit(1);
		// primitiveElement = 15;
		// exit(1);
		assert(primitiveElementFound);
	}

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

	ExtFiniteField256(uint8_t val)
	{
		num = val;
	}

	friend ExtFiniteField256 operator+(ExtFiniteField256 lhs, const ExtFiniteField256& rhs)
	{
		lhs.num ^= rhs.num;
		return lhs; // return the result by value (uses move constructor)
	}

	friend ExtFiniteField256 operator-(ExtFiniteField256 lhs, const ExtFiniteField256& rhs)
	{
		lhs.num ^= rhs.num;
		return lhs; // return the result by value (uses move constructor)
	}

	ExtFiniteField256 operator-() const
	{
		return *this;
	}

	ExtFiniteField256& operator+=(const ExtFiniteField256& rhs)
	{
		num ^= rhs.num;
		return *this; // return the result by reference
	}

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

	friend ExtFiniteField256 operator*(ExtFiniteField256 lhs, const ExtFiniteField256& rhs)
	{
		int result = xorMult(lhs.num, rhs.num);
		result	   = xorDiv(result, irreducible_primitive_polynomial);
		return result; // return the result by value (uses move constructor)
	}

	ExtFiniteField256& operator*=(const ExtFiniteField256& rhs)
	{
		int result = xorMult(num, rhs.num);
		result	   = xorDiv(result, irreducible_primitive_polynomial);

		num = result;

		return *this;
	}

	friend ExtFiniteField256 operator/(ExtFiniteField256 lhs, const ExtFiniteField256& rhs)
	{
		int result = xorMult(lhs.num, reciprocal.at(rhs.num));
		result	   = xorDiv(result, irreducible_primitive_polynomial);
		return result; // return the result by value (uses move constructor)
	}

	operator int() const
	{
		return num;
	}
};

#endif /* EXTFINITEFIELD256_H_ */
