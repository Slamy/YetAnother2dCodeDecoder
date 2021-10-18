/*
 * FiniteField.h
 *
 *  Created on: 17.10.2021
 *      Author: andre
 */

#ifndef FINITEFIELD_H_
#define FINITEFIELD_H_

#include <array>
#include <assert.h>
#include <iostream>

template <int prime> class FiniteField
{
  private:
	int num;
	static constexpr int addition_neutral{0};
	static constexpr int multiplication_neutral{1};

	static std::array<int, prime> reciprocal;

  public:
	static void buildReciprocal()
	{
		for (int i = 0; i < prime; i++)
		{
			bool reciprocalFound{false};

			for (int j = 0; j < prime; j++)
			{
				FiniteField a(i);
				FiniteField b(j);

				// printf("%d ",(a * b).asInt());

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
		assert(val < prime);
		num = val;
	}

    operator int() const { return num;}

	friend bool operator>(const FiniteField& lhs, const FiniteField& rhs)
	{
		return lhs.num > rhs.num;
	}

	friend FiniteField operator+(FiniteField lhs, const FiniteField& rhs)
	{
		lhs.num = (lhs.num + rhs.num) % prime;
		return lhs; // return the result by value (uses move constructor)
	}

	friend FiniteField operator-(FiniteField lhs, const FiniteField& rhs)
	{
		lhs.num = (lhs.num - rhs.num);
		if (lhs.num < 0)
			lhs.num += prime;

		return lhs; // return the result by value (uses move constructor)
	}

	friend FiniteField operator*(FiniteField lhs, const FiniteField& rhs)
	{
		lhs.num = (lhs.num * rhs.num) % prime;
		return lhs; // return the result by value (uses move constructor)
	}

	friend FiniteField operator/(FiniteField lhs, const FiniteField& rhs)
	{
		lhs.num = (lhs.num * reciprocal[rhs.num]) % prime;
		return lhs; // return the result by value (uses move constructor)
	}
};

template <int prime> std::array<int, prime> FiniteField<prime>::reciprocal;

using FiniteField7 = FiniteField<7>;

#endif /* FINITEFIELD_H_ */
