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

template <int prime> class FiniteField
{
  private:
	int num;
	static constexpr int addition_neutral{0};
	static constexpr int multiplication_neutral{1};

	static std::array<int, prime> reciprocal;
	static FiniteField primitiveElement;

  public:
	static FiniteField getPrimitiveElement()
	{
		return primitiveElement;
	}

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

	static FiniteField getPrimitiveElementPow(int p)
	{
		int pabs = labs(p);

		if (pabs == 0)
		{
			return FiniteField(multiplication_neutral);
		}

		FiniteField val(primitiveElement);
		pabs--;
		while (pabs)
		{
			val *= primitiveElement;
			pabs--;
		}
		return (p > 0) ? val : FiniteField(reciprocal.at(val.num));
	}

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

		// exit(1);
		// primitiveElement = 15;
		// exit(1);
		assert(primitiveElementFound);
	}

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

	operator int() const
	{
		return num;
	}

	friend bool operator>(const FiniteField& lhs, const FiniteField& rhs)
	{
		return lhs.num > rhs.num;
	}

	FiniteField operator-() const
	{
		return -num;
	}

	friend FiniteField operator+(FiniteField lhs, const FiniteField& rhs)
	{
		assert(lhs.num < prime);
		assert(rhs.num < prime);

		lhs.num = (lhs.num + rhs.num) % prime;

		assert(lhs.num < prime);
		return lhs; // return the result by value (uses move constructor)
	}

	FiniteField& operator+=(const FiniteField& rhs)
	{
		assert(rhs.num < prime);
		num = (num + rhs.num) % prime;

		return *this; // return the result by reference
	}

	FiniteField& operator*=(const FiniteField& rhs)
	{
		assert(num < prime);
		assert(rhs.num < prime);

		num = (num * rhs.num) % prime;

		assert(num < prime);
		return *this;
	}

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

	friend FiniteField operator*(FiniteField lhs, const FiniteField& rhs)
	{
		assert(lhs.num < prime);
		assert(rhs.num < prime);

		lhs.num = (lhs.num * rhs.num) % prime;

		assert(lhs.num < prime);
		return lhs; // return the result by value (uses move constructor)
	}

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
