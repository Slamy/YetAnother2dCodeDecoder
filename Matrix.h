/*
 * Matrix.h
 *
 *  Created on: 20.10.2021
 *      Author: andre
 */

#ifndef MATRIX_H_
#define MATRIX_H_

#include <cassert>
#include <iostream>
#include <vector>

template <class T> class Matrix
{
	std::vector<std::vector<T>> a;

  public:
	int getNumberOfRows()
	{
		return a.size();
	}

	int getNumberOfCols()
	{
		return a.at(0).size();
	}

	Matrix() = default;

	Matrix(std::vector<std::vector<T>> in)
	{
		a = in;
	}

	Matrix(std::initializer_list<std::initializer_list<T>> in)
	{
		a.clear();
		int columns = 0;

		for (auto& row : in)
		{
			if (!columns)
				columns = row.size();
			else
				assert(columns == row.size());

			a.push_back(std::vector<T>(row));
		}
	}

	void appendRight(std::vector<T> toAppend)
	{
		assert(getNumberOfRows() == toAppend.size());

		for (int i = 0; i < toAppend.size(); i++)
		{
			a.at(i).push_back(toAppend.at(i));
		}
	}

	friend std::vector<T> operator*(Matrix<T> lhs, const std::vector<T>& rhs)
	{
		assert(lhs.getNumberOfRows() == rhs.size());
		std::vector<T> result(rhs.size());

		for (int row = 0; row < rhs.size(); row++)
		{
			result.at(row) = 0;
			for (int col = 0; col < lhs.getNumberOfCols(); col++)
			{
				// printf("Multiply %d   %f * %f\n",row,rhs.at(row), lhs.a.at(row).at(col));

				result.at(row) += rhs.at(col) * lhs.a.at(row).at(col);
			}
			// printf("Result at end is %f\n",result.at(row));
		}

		return result;
	}

	void print()
	{
		for (auto& row : a)
		{
			for (auto& col : row)
			{
				std::cout << col << " ";
			}
			std::cout << std::endl;
		}
	}
	/**
	 * Stolen from https://www.tutorialspoint.com/cplusplus-program-to-implement-gauss-jordan-elimination
	 *
	 * @tparam T
	 * @param a
	 * @return
	 */
	std::vector<T> gauss_jordan_elim()
	{
		int n = a.size();
		std::vector<T> x(n);
		int row, pivot_row, k; // declare variables and matrixes as
		T b;
		int col;

		std::vector<bool> row_used_for_pivot(n);
		std::vector<int> pivot_pos(n);

		// iterate through the colums
		for (col = 0; col < n; col++)
		{
			// printf("Do column %d!\n", col);
			// find a pivot
			pivot_row = -1;
			for (row = 0; row < n; row++)
			{
				if (a.at(row).at(col) != 0.0f && row_used_for_pivot.at(row) == false)
				{
					pivot_row						 = row;
					row_used_for_pivot.at(pivot_row) = true;
					pivot_pos.at(col)				 = row;
					// printf("Found pivot for col in row %d\n", pivot_row);
					break;
				}
			}
			assert(pivot_row >= 0);

			// iterate through the other rows of this column
			for (row = 0; row < n; row++)
			{
				// only affect the other rows
				if (row != pivot_row)
				{
					assert(a.at(pivot_row).at(col) != 0.0f);

					// generate the factor to apply
					b = a.at(row).at(col) / a.at(pivot_row).at(col);

					// apply to all columns
					for (k = 0; k <= n; k++)
					{
						a.at(row).at(k) = a.at(row).at(k) - b * a.at(pivot_row).at(k);
					}
					// printf("- %d %d\n", row, pivot_row);
					// print();
				}
			}
		}
		std::cout << "\nThe solution is:\n";
		for (col = 0; col < n; col++)
		{
			int row	  = pivot_pos.at(col);
			x.at(col) = a.at(row).at(n) / a.at(row).at(col);
			std::cout << "x" << row << "=" << x.at(row) << " ";
		}
		std::cout << std::endl;
		return x;
	}
};

#endif /* MATRIX_H_ */
