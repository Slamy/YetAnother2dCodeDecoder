/*
 * ReedSolomon.h
 *
 *  Created on: 23.10.2021
 *      Author: andre
 */

#ifndef REEDSOLOMON_H_
#define REEDSOLOMON_H_

#include "Matrix.h"
#include "Polynom.h"

namespace RS
{
class MessageUncorrectable : public std::exception
{
};

template <class FF> class ReedSolomon
{
	Polynom<FF> generator;

  public:
	/// Block length
	const int n = 10;

	/// Message length for actual payload
	const int k = 8;

	/// Number of check symbols
	/// Also degree of generator polynomial
	/// Also number of syndroms
	const int t = n - k;

	ReedSolomon(int np, int kp) : n(np), k(kp), t(np - kp)
	{
		generator = Polynom<FF>({1});
		for (int i = 0; i < t; i++)
		{
			Polynom<FF> genCoefficient({1, -(FF::getPrimitiveElementPow(i))});
			generator = generator * genCoefficient;
		}

		assert(t == generator.getDegree());

#if 0
		std::cout << "generator " << generator.asString() << std::endl;
		for (int i = 0; i < np; i++)
		{
			std::cout << i << " " << generator.evaluate(FF(i)) << std::endl;
		}
#endif
	}

	Polynom<FF> encode(std::vector<FF> vdata)
	{
		assert(vdata.size() == k);
		Polynom<FF> data(vdata);
		assert(data.getSize() <= k);

		Polynom<FF> shiftPoly;
		shiftPoly.setCoefficient(t, 1);

		assert(shiftPoly.getDegree() == t);
		auto shiftedleft = data * shiftPoly;
		// std::cout << "shiftedleft " << shiftedleft.asString() << std::endl;

		auto division = shiftedleft / generator;
		// std::cout << "shiftedleft " << shiftedleft.asString() << std::endl;
		// std::cout << "redundancy " << division.second.asString() << std::endl;

		Polynom<FF> message = shiftedleft - division.second;

		// std::cout << "message to send " << message.asString() << std::endl;

		return message;
	}

	std::pair<std::vector<FF>, bool> calculateSyndromes(const Polynom<FF>& message)
	{
		std::vector<FF> syndromes(t);
		bool allSyndromesZero{true};

		allSyndromesZero = true;
		for (int i = 0; i < t; i++)
		{
			syndromes.at(i) = message.evaluate(FF::getPrimitiveElementPow(i));
			// std::cout << "syndrome " << syndromes.at(i) << std::endl;
			if (syndromes.at(i) != 0)
				allSyndromesZero = false;
		}

		return std::make_pair(syndromes, allSyndromesZero);
	}

	Polynom<FF> decode(const Polynom<FF>& message)
	{
		// std::cout << "message recv " << message.asString() << std::endl;
#if 0
		std::cout << "syndrome" << std::endl;
		for (int i = 0; i < 20; i++)
		{
			std::cout << message.evaluate(FF::getPrimitiveElementPow(i)) << std::endl;
		}
#endif

		std::vector<FF> syndromes(t);
		bool allSyndromesZero;

		std::tie(syndromes, allSyndromesZero) = calculateSyndromes(message);
		if (allSyndromesZero)
		{
			// std::cout << "Looks correct!\n";
			return message;
		}

		/*
		 * Example for t==2,3
		 * [s0] * [o1] = -s1
		 *
		 * Example for t==4
		 * [s0 s1] * [o1] = [-s2]
		 * [s1 s2]   [o2]   [-s3]
		 */
		int v = t / 2;
		std::vector<FF> errorLocatorCoefficients;
		Polynom<FF> o;

		while (v > 0)
		{
			Matrix<FF> errorLocationMatrix(v, v + 1);
			for (int row = 0; row < v; row++)
			{
				for (int col = 0; col < v; col++)
				{
					errorLocationMatrix.at(row, col) = syndromes.at(row + col);
				}
				errorLocationMatrix.at(row, v) = -syndromes.at(v + row);
			}
			errorLocatorCoefficients = errorLocationMatrix.gauss_jordan_elim();
			// errorLocationMatrix.print();

			if (errorLocatorCoefficients.size())
			{
				o.setPolynom({1});
				for (int coef = 0; coef < v; coef++)
				{
					o.setCoefficient(coef + 1, errorLocatorCoefficients.at(v - coef - 1));
				}
				break;
			}
			v--;
		}

		// std::cout << std::endl;
		std::vector<int> error_positions;

		for (int i = 0; i < n; i++)
		{
			FF evalResult = o.evaluate(FF::getPrimitiveElementPow(-i));
			// std::cout << evalResult << std::endl;

			if (evalResult == 0)
			{
				error_positions.push_back(i);
				// std::cout << "Found error at position " << (n - i - 1) << std::endl;
			}
		}

		if (error_positions.size() == 0)
		{
			throw MessageUncorrectable();
		}

		/*
		 * Example for t==4
		 * j = first error position
		 * k = second error position
		 * a = primitive element of finite field
		 * [a^0^j a^0^k] * [f1] = [s0]
		 * [a^1^j a^1^k]   [f2]   [s1]
		 * [a^2^j a^2^k]          [s2]
		 * [a^3^j a^3^k]          [s3]
		 *
		 * Example for t==2
		 * [a^0^j] * [f1] = [s0]
		 * [a^1^j]          [s1]
		 */
		int error_number = error_positions.size();

		Matrix<FF> errorCorrectionMatrix(t, error_number + 1);

		for (int row = 0; row < t; row++)
		{
			for (int col = 0; col < error_number; col++)
			{
				errorCorrectionMatrix.at(row, col) = FF::getPrimitiveElementPow(row).pow(error_positions.at(col));
			}
			errorCorrectionMatrix.at(row, error_number) = syndromes.at(row);
		}

		// errorCorrectionMatrix.print();
		auto errorMagnitudes = errorCorrectionMatrix.gauss_jordan_elim();
		// errorCorrectionMatrix.print();

		assert(error_positions.size() == errorMagnitudes.size());

		Polynom<FF> error_correction;
		for (int i = 0; i < error_number; i++)
		{
			error_correction.setCoefficient(error_positions.at(i), errorMagnitudes.at(i));
		}
		// std::cout << "error_correction " << error_correction.asString() << std::endl;
		// std::cout << "message " << message.asString() << std::endl;
		Polynom<FF> repaired_message = message - error_correction;
		// std::cout << "repaired_message " << repaired_message.asString() << std::endl;

		std::tie(syndromes, allSyndromesZero) = calculateSyndromes(repaired_message);

		if (allSyndromesZero)
		{
			// std::cout << "Looks correct now!\n";
		}
		else
		{
			throw MessageUncorrectable();
		}

		return repaired_message;
	}
};

} // namespace RS

#endif /* REEDSOLOMON_H_ */
