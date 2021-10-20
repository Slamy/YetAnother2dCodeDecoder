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
				// 	printf("%d ", current);
				if (found.at(current) == false)
					elementsfound++;
				found.at(current) = true;
				current			  = current * FiniteField(possible_primitive);
			}
			if (elementsfound == prime - 1)
			{
				// printf("Yes!");
				primitiveElement	  = FiniteField(possible_primitive);
				primitiveElementFound = true;
			}
			// printf("\n");
		}

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
		lhs.num = (lhs.num * reciprocal[rhs.num]) % prime;
		assert(lhs.num < prime);
		return lhs; // return the result by value (uses move constructor)
	}
};

template <int prime> std::array<int, prime> FiniteField<prime>::reciprocal;
template <int prime> FiniteField<prime> FiniteField<prime>::primitiveElement{0};

using FiniteField7 = FiniteField<7>;

#endif /* FINITEFIELD_H_ */
