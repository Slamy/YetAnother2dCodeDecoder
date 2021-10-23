/*
 * Polynom.h
 *
 *  Created on: 17.10.2021
 *      Author: andre
 */

#ifndef POLYNOM_H_
#define POLYNOM_H_

#include <cassert>
#include <functional>
#include <initializer_list>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

template <class NumberType> class Polynom
{
  private:
	/**
	 *  the index is the grade
	 *  values[0] = v -> v
	 *  values[1] = v -> v*x
	 *  values[2] = v -> v*x^2
	 */
	std::vector<NumberType> values;

  public:
	int getSize()
	{
		return values.size();
	}

	void setCoefficient(int exponent, NumberType a)
	{
		if (values.size() < exponent + 1)
			values.resize(exponent + 1);
		values.at(exponent) = a;
	}

	int getDegree() const
	{
		int grade = 0;
		for (int i = 0; i < values.size(); i++)
		{
			if (values.at(i) != NumberType(0))
				grade = i;
		}
		return grade;
	}

	void optimize()
	{
		values.resize(getDegree() + 1);
	}

	void setPolynom(std::initializer_list<NumberType> il)
	{
		values.clear();

		// for (auto it = std::begin(il); it != std::end(il); ++it)
		for (auto it = std::rbegin(il); it != std::rend(il); ++it)
		{
			values.push_back(*it);
		}
		optimize();
	}

	Polynom() = default;

	Polynom(std::initializer_list<NumberType> il)
	{
		setPolynom(il);
	}

	Polynom(std::vector<NumberType> v)
	{
		values = v;
		optimize();
	}

	friend bool operator!=(const Polynom& lhs, const Polynom& rhs)
	{
		return !(lhs == rhs);
	}

	friend bool operator==(const Polynom& lhs, const Polynom& rhs)
	{
		int lhsOrder = lhs.getDegree();
		if (lhsOrder != rhs.getDegree())
			return false;

		for (int i = 0; i <= lhsOrder; i++)
		{
			if (lhs.values.at(i) != rhs.values.at(i))
				return false;
		}

		return true;
	}

	friend Polynom operator-(Polynom lhs, const Polynom& rhs)
	{

		int maxsize = std::max(lhs.values.size(), rhs.values.size());
		lhs.values.resize(maxsize);
		for (int j = 0; j < rhs.values.size(); j++)
		{
			lhs.values.at(j) = lhs.values.at(j) - rhs.values.at(j);
		}
		return lhs;
	}

	friend Polynom operator+(Polynom lhs, const Polynom& rhs)
	{

		int maxsize = std::max(lhs.values.size(), rhs.values.size());
		lhs.values.resize(maxsize);
		for (int j = 0; j < rhs.values.size(); j++)
		{
			lhs.values.at(j) = lhs.values.at(j) + rhs.values.at(j);
		}
		return lhs;
	}

	friend Polynom operator*(Polynom lhs, const Polynom& rhs)
	{
		std::vector<NumberType> result;
		result.resize(lhs.values.size() + rhs.values.size() - 1);

		for (int i = 0; i < lhs.values.size(); i++)
		{
			for (int j = 0; j < rhs.values.size(); j++)
			{
				result.at(i + j) = result.at(i + j) + lhs.values.at(i) * rhs.values.at(j);
			}
		}

		return Polynom(result);
	}

	/**
	 * Perform polynom division for signed integers.
	 *
	 * Example: {3,2,1} / (1,1)
	 *   3x^2 + 2x + 1 : x + 1 = 3x - 1
	 * -(3x^2 + 3x)
	 * ____________
	 *          -x + 1
	 *        -(-x - 1)
	 *        _________
	 *               2
	 *
	 * Result is the pair {3,-1}, {2}
	 * @param lhs dividend
	 * @param rhs divisor
	 * @return  pair of result and remainder
	 */
	friend std::pair<Polynom, Polynom> operator/(Polynom lhs, const Polynom& rhs)
	{
		int l_order	  = lhs.getDegree();
		int r_order	  = rhs.getDegree();
		auto dividend = lhs.values;
		auto divisor  = rhs.values;

		int maxgrade = std::max(l_order, r_order) + 1;

		std::vector<NumberType> result(maxgrade);
		// std::vector<NumberType> remainder(maxgrade);

		int gradeDiff = l_order - r_order;

		// printf("gradeDiff %d\n", gradeDiff);

		for (int i = 0; i <= gradeDiff; i++)
		{
			// printf("Cycle %d Dividend:%s\n", i, lhs.asString().c_str());

			NumberType divididendHighestRemainingGrade = dividend.at(l_order - i);
			NumberType divisorHighestGrade			   = divisor.at(r_order);

			NumberType factor = divididendHighestRemainingGrade / divisorHighestGrade;

			// printf("add to result: %dx^%d\n", static_cast<int>(factor), l_order - i - r_order);
			result.at(l_order - i - r_order) = factor;
			// printf("%d %d %d\n", divididendHighestRemainingGrade, divisorHighestGrade, factor);

			for (int j = 0; j <= r_order; j++)
			{
				// printf("Edit: %d %d\n", dividend.at(l_order - j - i), divisor.at(r_order - j));

				dividend.at(l_order - j - i) = dividend.at(l_order - j - i) - factor * divisor.at(r_order - j);

				// printf("After: %d\n", dividend.at(l_order - j - i));
			}
			assert(dividend.at(l_order - i) == 0);
		}
		// dividend is now the remainer

#if 0
		std::cout << "Polynomdivision " << lhs.asString() << " : " << rhs.asString() << " = "
				  << Polynom(result).asString() << " rest " << Polynom(dividend).asString() << std::endl;
#endif
		// verify result by reversing this with multiplication
		assert(Polynom(result) * rhs + dividend == lhs);

		return std::make_pair(Polynom(result), Polynom(dividend));
	}

	/**
	 * Evaluates polynom function using the Horner scheme
	 * @param x		as f(x)=...
	 * @return		f(x)
	 */
	NumberType evaluate(NumberType x) const
	{
		NumberType result{0};

		for (int i = values.size() - 1; i > 0; i--)
		{
			result = result * x + x * values.at(i);
		}
		result = result + values.at(0);

		return result;
	}

	std::string asString() const
	{
		std::stringstream ss;

		bool firstPrint = true;
		for (int i = values.size() - 1; i >= 0; i--)
		{
			int val = static_cast<int>(values.at(i));
			// printf("val %d\n",val);
			if (val)
			{
				if (firstPrint == false)
				{
					ss << " + ";
				}

				firstPrint = false;
				if ((val > 1) || (i == 0) || (val < -1))
					ss << val;
				else if (val == -1)
					ss << "-";

				if (i > 1)
					ss << "x^" << i;
				else if (i > 0)
					ss << "x";
			}
		}

		return ss.str();
	}
};

#endif /* POLYNOM_H_ */
