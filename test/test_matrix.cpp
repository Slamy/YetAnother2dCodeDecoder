/*
 * test_matrix.cpp
 *
 *  Created on: 20.10.2021
 *      Author: andre
 */

#include "Matrix.h"
#include "gtest/gtest.h"
#include "FiniteField.h"

TEST(MatrixTest, GaussJordanElimination)
{
	// Example with floats
	{
		Matrix<float> polynoms({{1, 2, -4}, {7, 6, -2}, {0, -3, -5}});
		Matrix<float> polynoms_gje(polynoms);
		polynoms_gje.print();
		std::vector<float> f_x({2, -5, -8});

		polynoms_gje.appendRight(f_x);
		polynoms_gje.print();
		std::cout << "Do the Gauss Jordan Elimination\n";

		std::vector<float> x_cof = polynoms_gje.gauss_jordan_elim();

		std::cout << "Matrix after GJE\n";
		polynoms_gje.print();
		std::cout << "Original Matrix\n";
		polynoms.print();

		std::vector<float> verify = polynoms * x_cof;
		assert(verify.size() == f_x.size());

		for (int i = 0; i < verify.size(); i++)
		{
			printf("%f %f\n", f_x.at(i), verify.at(i));
			ASSERT_FLOAT_EQ(f_x.at(i), verify.at(i));
		}
	}

	// Example as before with altered row order
	{
		Matrix<float> polynoms({{0, -3, -5}, {1, 2, -4}, {7, 6, -2}});
		Matrix<float> polynoms_gje(polynoms);
		polynoms_gje.print();
		std::vector<float> f_x({-8, 2, -5});

		polynoms_gje.appendRight(f_x);
		polynoms_gje.print();
		std::cout << "Do the Gauss Jordan Elimination\n";

		std::vector<float> x_cof = polynoms_gje.gauss_jordan_elim();

		std::cout << "Matrix after GJE\n";
		polynoms_gje.print();
		std::cout << "Original Matrix\n";
		polynoms.print();

		std::vector<float> verify = polynoms * x_cof;
		assert(verify.size() == f_x.size());

		for (int i = 0; i < verify.size(); i++)
		{
			printf("%f %f\n", f_x.at(i), verify.at(i));
			ASSERT_FLOAT_EQ(f_x.at(i), verify.at(i));
		}
	}

	// Example with FiniteField7
	{
		Matrix<FiniteField7> polynoms({{1, 2, 4}, {3, 6, 2}, {0, 3, 5}});
		Matrix<FiniteField7> polynoms_gje(polynoms);
		polynoms_gje.print();
		std::vector<FiniteField7> f_x({2, 5, 4});

		polynoms_gje.appendRight(f_x);
		polynoms_gje.print();
		std::cout << "Do the Gauss Jordan Elimination\n";

		std::vector<FiniteField7> x_cof = polynoms_gje.gauss_jordan_elim();

		std::cout << "Matrix after GJE\n";
		polynoms_gje.print();
		std::cout << "Original Matrix\n";
		polynoms.print();

		std::vector<FiniteField7> verify = polynoms * x_cof;
		assert(verify.size() == f_x.size());

		for (int i = 0; i < verify.size(); i++)
		{
			printf("%d %d\n", f_x.at(i), verify.at(i));
			ASSERT_EQ(f_x.at(i), verify.at(i));
		}
	}
}
