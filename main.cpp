
#include "FiniteField.h"
#include "Polynom.h"
#include <iostream>

int main()
{
	FiniteField7::buildReciprocal();

	{
		FiniteField7 a(3);
		FiniteField7 b(5);
		std::cout << (a + b) << std::endl;
		std::cout << (a - b) << std::endl;
		std::cout << (a * b) << std::endl;
		std::cout << (a / b) << std::endl;
		std::cout << ((a * b) / b) << std::endl;
	}

	{
		Polynom<FiniteField7> p1({1, 2, 3, 4, 5});
		Polynom<FiniteField7> p3({1, 0}); // just x

		std::cout << p1.asString() << std::endl;
		std::cout << p3.asString() << std::endl;
		std::cout << "Mul:" << std::endl;
		std::cout << (p1 * p3).asString() << std::endl;
	}
	{
		Polynom<int> p1({3, 2, 1});
		Polynom<int> p3({1, 1}); // just x


		std::cout << "Grade:" << p1.getGrade()<< std::endl;
		std::cout << "Grade:" << p3.getGrade()<< std::endl;

		std::cout << "Div:" << std::endl;
		auto pdiv = p1 / p3;
		std::cout <<"Result: "<< pdiv.first.asString() << std::endl;
		std::cout <<"Remainder: "<< pdiv.second.asString() << std::endl;
	}
#if 0
	std::cout << Polynom<FiniteField7>({1, 0}).asString() << std::endl;
	std::cout << Polynom<FiniteField7>({1, 0, 0}).asString() << std::endl;
	std::cout << Polynom<FiniteField7>({1, 0, 1}).asString() << std::endl;
#endif

	{
		Polynom<int> p1;
		p1.setPolynom({1, 0, 0});
		std::cout << p1.evaluate(2) << std::endl;
		p1.setPolynom({0, 1, 0});
		std::cout << p1.evaluate(2) << std::endl;
		p1.setPolynom({0, 0, 1});
		std::cout << p1.evaluate(2) << std::endl;

		p1.setPolynom({2, 2, 2});
		std::cout << p1.evaluate(2) << std::endl;
	}

	{
		Polynom<int> p1;
		p1.setPolynom({0, 1, 0, 0});
		std::cout << p1.getGrade() <<" X "<<p1.asString()<< std::endl;
		p1.optimize();
		std::cout << p1.getGrade() <<" X "<<p1.asString()<< std::endl;

		p1.setPolynom({0, 0, 1, 0});
		std::cout << p1.getGrade() << std::endl;
		p1.setPolynom({0, 0, 0, 1});
		std::cout << p1.getGrade() << std::endl;
	}
}
