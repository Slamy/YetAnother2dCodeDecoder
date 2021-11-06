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

/**
 * Utility class which provides matrix math operations
 * @tparam T	Data type to use for the elements
 */
template <class T> class Matrix
{
  private:
	/// Actual matrix data
	std::vector<std::vector<T>> a;

  public:
	/**
	 * Provides size of matrix
	 * @return	Number of Rows
	 */
	int getNumberOfRows()
	{
		return a.size();
	}

	/**
	 * Provides size of matrix
	 * @return	Number of Columns
	 */
	int getNumberOfCols()
	{
		return a.at(0).size();
	}

	Matrix() = default;

	/**
	 * Constructs an empty matrix with every element being 0
	 * @param rows	Rows
	 * @param cols	Columns
	 */
	Matrix(int rows, int cols)
	{
		a.clear();
		for (int r = 0; r < rows; r++)
		{
			a.emplace_back(std::vector<T>(cols));
		}
	}

	/**
	 * Provides access to a single element for reading and writing
	 * @param row	First index is 0
	 * @param col	First index is 0
	 * @return		Reference to element
	 */
	T& at(int row, int col)
	{
		return a.at(row).at(col);
	}

	/**
	 * Construct a matrix from vectors
	 * @param in	Vector of Row Vectors
	 */
	Matrix(std::vector<std::vector<T>> in)
	{
		a = in;
	}

	/**
	 * Construct a matrix from constant values
	 * @param in	List of rows
	 */
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

	/**
	 * Concatenate a vector to the matrix on the right side.
	 * Useful for \ref gauss_jordan_elim
	 *
	 * @param toAppend	Vector to append on the right side.
	 */
	void appendRight(std::vector<T> toAppend)
	{
		assert(getNumberOfRows() == toAppend.size());

		for (int i = 0; i < toAppend.size(); i++)
		{
			a.at(i).push_back(toAppend.at(i));
		}
	}

	/**
	 * Perform matrix multiplication with a vector
	 *
	 * @param lhs	matrix
	 * @param rhs	vector to multiply the matrix with
	 * @return		resulting vector
	 */
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

	/**
	 * Primitive debugging print of the matrix
	 */
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
	 * Performs a gauss jordan elimination on this matrix.
	 * The outer right column is supposed to be the result of the equations.
	 *
	 * Inspired by https://www.tutorialspoint.com/cplusplus-program-to-implement-gauss-jordan-elimination
	 * But this implementation also adds support for pivot finding.
	 *
	 * @return	Vector of coefficients if successful, empty vector of not
	 */
	std::vector<T> gauss_jordan_elim()
	{
		int leftCols = getNumberOfCols() - 1;
		int rows	 = getNumberOfRows();
		std::vector<T> coefficients(leftCols);
		int row, pivot_row, k;
		T b;
		int col;

		std::vector<bool> row_used_for_pivot(rows);
		std::vector<int> pivot_pos(leftCols);

		// iterate through the colums
		for (col = 0; col < leftCols; col++)
		{
			// printf("Do column %d!\n", col);
			// find a pivot
			pivot_row = -1;
			for (row = 0; row < rows; row++)
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
			if (pivot_row < 0)
			{
				// std::cout << "GJA failed to find a pivot!\n";
				// print();
				return std::vector<T>();
			}
			assert(pivot_row >= 0);

			// iterate through the other rows of this column
			for (row = 0; row < rows; row++)
			{
				// only affect the other rows
				if (row != pivot_row)
				{
					assert(a.at(pivot_row).at(col) != 0.0f);

					// generate the factor to apply
					b = a.at(row).at(col) / a.at(pivot_row).at(col);

					// apply to all columns
					for (k = 0; k <= leftCols; k++)
					{
						a.at(row).at(k) = a.at(row).at(k) - b * a.at(pivot_row).at(k);
					}
					// printf("- %d %d\n", row, pivot_row);
					// print();
				}
			}
		}

		for (col = 0; col < leftCols; col++)
		{
			int row				 = pivot_pos.at(col);
			coefficients.at(col) = a.at(row).at(leftCols) / a.at(row).at(col);
		}

		return coefficients;
	}
};

#endif /* MATRIX_H_ */
