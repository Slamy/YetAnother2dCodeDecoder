
#include "FiniteField.h"
#include "Matrix.h"
#include "Polynom.h"
#include <algorithm>
#include <iostream>

Polynom<FiniteField7> generator;

void setup()
{
	FiniteField7 primitiveElement = FiniteField7::getPrimitiveElement();

	Polynom<FiniteField7> gen1({1, -1});
	Polynom<FiniteField7> gen2({1, -(primitiveElement)});
	generator = gen1 * gen2;

	std::cout << "generator " << generator.asString() << std::endl;
	for (int i = 0; i < 7; i++)
	{
		std::cout << generator.evaluate(FiniteField7(i)) << std::endl;
	}
}

Polynom<FiniteField7> sender()
{
	Polynom<FiniteField7> data({3, 2});

	// n = 4, k = 2
	auto shiftedleft = data * Polynom<FiniteField7>({1, 0, 0});
	std::cout << "shiftedleft " << shiftedleft.asString() << std::endl;

	auto division = shiftedleft / generator;
	std::cout << "shiftedleft " << shiftedleft.asString() << std::endl;
	std::cout << "redundancy " << division.second.asString() << std::endl;

	Polynom<FiniteField7> message = shiftedleft - division.second;

	std::cout << "message " << message.asString() << std::endl;

	return message;
}

void receiver(Polynom<FiniteField7> message)
{
	// message = message + Polynom<FiniteField7>({0, 0, 1, 0});

	auto syndromePair = message / generator;
	std::cout << "shall be 0 " << syndromePair.first.asString() << std::endl;
	auto& syndrome = syndromePair.second;
	std::cout << "syndrome " << syndrome.asString() << std::endl;

	for (int i = 0; i < 7; i++)
	{
		std::cout << syndrome.evaluate(FiniteField7(i)) << std::endl;
	}

	std::cout << std::endl;
	for (int i = 0; i < 7; i++)
	{
		std::cout << message.evaluate(FiniteField7(i)) << std::endl;
	}
}

int main()
{
	FiniteField7::buildReciprocal();
	FiniteField7::findPrimitiveElement();

	{
		std::vector<std::vector<float>> matrix{{1, 2, -4, 2}, {7, 6, -2, -5}, {0, -3, -5, -8}};
		std::vector<float> result;
		Matrix<float> m(matrix);
		m.print();
		result = m.gauss_jordan_elim();
		m.print();
	}
	return 0;

	std::cout << "Primitive Element " << FiniteField7::getPrimitiveElement() << std::endl;

	setup();
	receiver(sender());

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
		Polynom<int> p3({1, 1});

		std::cout << "Grade:" << p1.getOrder() << std::endl;
		std::cout << "Grade:" << p3.getOrder() << std::endl;

		std::cout << "Div:" << std::endl;
		auto pdiv = p1 / p3;
		std::cout << "Result: " << pdiv.first.asString() << std::endl;
		std::cout << "Remainder: " << pdiv.second.asString() << std::endl;
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
		std::cout << p1.getOrder() << " " << p1.asString() << std::endl;

		p1.setPolynom({0, 0, 1, 0});
		std::cout << p1.getOrder() << std::endl;
		p1.setPolynom({0, 0, 0, 1});
		std::cout << p1.getOrder() << std::endl;
	}
}
