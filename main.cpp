
#include "FiniteField.h"
#include "Matrix.h"
#include "Polynom.h"
#include "ReedSolomon.h"
#include <algorithm>
#include <exception>
#include <iostream>

// static constexpr int prime = 19;
static constexpr int prime = 53;

std::vector<FiniteField<prime>> generateData(int i)
{
	std::vector<FiniteField<prime>> result;

	while (i)
	{
		int rest = i % prime;
		int div	 = i / prime;

		result.push_back((rest + 3) % prime);

		i = div;
	}

	return result;
}

int myPow(int x, unsigned int p)
{
	if (p == 0)
		return 1;
	if (p == 1)
		return x;

	int tmp = myPow(x, p / 2);
	if (p % 2 == 0)
		return tmp * tmp;
	else
		return x * tmp * tmp;
}

int main()
{
	FiniteField<prime>::buildReciprocal();
	FiniteField<prime>::findPrimitiveElement();

	int n = 17;
	int t = 4;
	int k = n - t;
	RS::ReedSolomon<FiniteField<prime>> rs(n, k);

	int cycles = myPow(3, rs.k);
	assert(cycles > 0);
	for (int i = 0; i < cycles; i++)
	{
		if ((i % 10000) == 0)
			printf("%d / %d\n", i, cycles);

		auto data = generateData(i);
		data.resize(rs.k);
		auto sent_message = rs.encode(data);
		auto error		  = Polynom<FiniteField<prime>>({0, 0, 0, 0, 0, //
													 0, 0, 3, 0, 0, //
													 0, 0, 0, 0, 0, //
													 0, 0});

		assert(error.getSize() <= rs.n);

		Polynom<FiniteField<prime>> recv_message = sent_message + error;

		try
		{
			auto repaired_message = rs.decode(recv_message);
			assert(repaired_message == sent_message);
		}
		catch (const RS::MessageUncorrectable& e)
		{
			std::cout << e.what() << std::endl;
			abort();
		}
	}

	printf("Finished!\n");
	return 0;
}
