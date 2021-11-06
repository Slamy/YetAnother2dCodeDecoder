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

/**
 * Math utility class for Polynomials like
 * f(x) = x^2 + 2x + 1
 *
 * @tparam NumberType	data type
 */
template <class NumberType> class Polynom
{
  private:
	/**
	 *  the index is the degree
	 *  values[0] = v -> v
	 *  values[1] = v -> v*x
	 *  values[2] = v -> v*x^2
	 */
	std::vector<NumberType> values;

  public:
	/**
	 * Provides a way to access the raw coefficients.
	 * @return	coefficients, the index is the degree
	 */
	std::vector<NumberType> getRawData()
	{
		return values;
	}

	/**
	 * Allows setting a single coefficient.
	 * @param degree	degree of x to set
	 * @param a			value of coefficient
	 */
	void setCoefficient(int degree, NumberType a)
	{
		if (values.size() < degree + 1)
			values.resize(degree + 1);
		values.at(degree) = a;
	}

	/**
	 * Provides degree of polynomial
	 * @return	degree of polynomial
	 */
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

	/**
	 * Reduces size to minimum.
	 */
	void optimize()
	{
		values.resize(getDegree() + 1);
	}

	/**
	 * Sets the polynomial from an initializer list
	 * Example:
	 * {2,1,3} -> f(x)= 2*x^2 + x + 3
	 *
	 * @param il	Coefficients
	 */
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

	/**
	 * Constructs the polynomial from an initializer list
	 * Example:
	 * {2,1,3} -> f(x)= 2*x^2 + x + 3
	 *
	 * @param il	Coefficients
	 */
	Polynom(std::initializer_list<NumberType> il)
	{
		setPolynom(il);
	}

	/**
	 * Constructs the polynomial from a vector
	 * Example:
	 * {2,1,3} -> f(x)= 2*x^2 + x + 3
	 *
	 * @param v	Coefficients
	 */
	Polynom(std::vector<NumberType> v)
	{
		values = v;
		optimize();
	}

	/**
	 * Compares two polynomials
	 * @param lhs	left hand side
	 * @param rhs	right hand side
	 * @return		true if both polynomials are not equal
	 */
	friend bool operator!=(const Polynom& lhs, const Polynom& rhs)
	{
		return !(lhs == rhs);
	}

	/**
	 * Compares two polynomials
	 * @param lhs	left hand side
	 * @param rhs	right hand side
	 * @return		true if both polynomials are equal
	 */
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

	/**
	 * Perform subtraction
	 * @param lhs	minuend
	 * @param rhs	subtrahend
	 * @return		difference
	 */
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

	/**
	 * Add two terms together
	 * @param lhs	Left term
	 * @param rhs	Right term
	 * @return		Sum
	 */
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

	/**
	 * Perform polynomial multiplication
	 * @param lhs	factor
	 * @param rhs	factor
	 * @return		product
	 */
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
	 * Perform polynomial division
	 *
	 * Example with signed integers: {3,2,1} / (1,1)
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

	/**
	 * Formats the polynomial as a string
	 * @return	text representation
	 */
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
