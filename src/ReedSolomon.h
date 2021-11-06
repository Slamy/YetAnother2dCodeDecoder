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

/**
 * Is thrown in case the received message was uncorrectable.
 */
class MessageUncorrectable : public std::exception
{
};

/**
 * Abstract implementation of a Reed Solomon Encoder and Decoder.
 * Encoding is systematic.(Payload is visible in message)
 * Decoding is done via Peterson-Gorenstein-Zierler as it was easier to understand
 * than Berlekamp–Massey.
 *
 * Informations to create this were gathered from:
 *
 * http://www.cdsy.de/reed-solomon.html
 *
 * https://en.wikiversity.org/wiki/Reed%E2%80%93Solomon_codes_for_coders
 *
 * https://www.nayuki.io/page/reed-solomon-error-correcting-code-decoder
 *
 * https://en.wikipedia.org/wiki/Reed%E2%80%93Solomon_error_correction
 *
 * @tparam FF	Finite Field class to use here.
 */
template <class FF> class ReedSolomon
{
  private:
	/**
	 * Generator polynomial. To encode the message, the payload polynomial is divided by the generator polynomial.
	 * The generator polynomial has the same number of roots as check symbols exist.
	 * For 3 check symbols, the generator is calculated like this
	 * g(x) = (x - a^i) * (x - a^(i+1)) * (x - a^(i+2))
	 *
	 * a is the primitive element alpha of the Finite Field. What does that mean?
	 * The j power of alpha is unique as long as 0 <= j <= q where q is the field size.
	 * This ensures that we don't have the same root twice as this polynomial must be irreducible.
	 *
	 * What is i? For QR Code it must be 0. But it's not fixed as long as sender and receiver use the same generator.
	 */
	Polynom<FF> generator;

  public:
	/// Block length. Number of symbols in the message.
	const int n = 10;

	/// Message length for actual payload
	const int k = 8;

	/// Number of check symbols
	/// Also degree of generator polynomial
	/// Also number of syndroms
	const int t = n - k;

	/**
	 * Create a encoder/decoder with these code parameters
	 * @param np	Number of symbols total
	 * @param kp	Number of symbols for actual payload data
	 */
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

	/**
	 * Append error correction symbols and return result as polynomial.
	 * @param vdata		Payload data
	 * @return			Polynomial with error correction data
	 */
	Polynom<FF> encode(std::vector<FF> vdata)
	{
		assert(vdata.size() == k);
		Polynom<FF> data(vdata);
		assert(data.getDegree() <= k);

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

	/**
	 * Evaluate a message polynomial at the positions of the expected roots of the generator polynomial.
	 * If the message was unaltered, it should have roots at the exact same places. The result
	 * of this calculation are the syndromes. If these are 0, the message is expected to be correct.
	 *
	 * S(i) = M(a^i) -> 0 if everything is correct.
	 * a is the alpha of the finite field.
	 * i must be the roots of the generator polynomial
	 * @param message	Received message to check
	 * @return			Syndromes and a bool value which is true, if all syndromes are 0.
	 */
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

	/**
	 * Check a received message for issues and tries to fix them.
	 * Location of errors is done via Peterson-Gorenstein-Zierler.
	 *
	 * Can throw MessageUncorrectable if we can't decode a message.
	 *
	 * @param message	Received message as polynomial
	 * @return			Received message which is the same as the input if correct or repaired.
	 */
	Polynom<FF> decode(const Polynom<FF>& message)
	{
		/*
		 * We start by checking the syndromes. If these are all 0 then the message was received correctly.
		 */
		std::vector<FF> syndromes(t);
		bool allSyndromesZero;

		std::tie(syndromes, allSyndromesZero) = calculateSyndromes(message);
		if (allSyndromesZero)
		{
			return message;
		}

		/*
		 * So the message is not correct? How sad, but not all hope is lost.
		 * We have to construct an error locator polynomial at this point to check for
		 * the position of errors we might be able to correct.
		 *
		 * I can't really explain how it works exactly but the errors are supposed
		 * to depend on the syndromes not being zero. Please find further informations
		 * about this process in the URLs provided by \ref ReedSolomon
		 *
		 * We require solving a gauss jordan elimination to get the coefficients of
		 * the error locator polynomial.
		 *
		 * Example for t==2,3
		 * | s0 | * | o1 | = | -s1 |
		 *
		 * Example for t==4
		 * | s0 s1 | * | o1 | = | -s2 |
		 * | s1 s2 |   | o2 |   | -s3 |
		 *
		 */
		int v = t / 2; // With t error correction symbols one can correct v incorrectly transmitted symbols.

		std::vector<FF> errorLocatorCoefficients;
		Polynom<FF> o;

		/*
		 * If there are less errors than we want to correct, the gauss jordan algorithm will provide not an exact
		 * solution. Repeat with a smaller v and try again with a smaller matrix.
		 */
		while (v > 0)
		{
			// Construct matrix as described above
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

			// If the GJE was successful, construct the error locator polynomial
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

		/*
		 * We must now find the roots of the error locator polynomial. Every root is an error position.
		 * The easiest way to do this is by evaluating the error locator polynomial with the position
		 * of every received symbol.
		 *
		 * 0 <= i < n
		 * If o(alpha^-i) == 0, then i is the degree of the coefficient in the message which is wrong.
		 */
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

		/* The positions of the errors are now known. But we don't know their magnitude yet.
		 * Like with the error location polynomial I can't really explain how this works.
		 *
		 * But the values of the syndromes are linearly dependent on the positions and magnitudes.
		 * This can again be solved using gauss jordan elimination.
		 *
		 * Example for t==4
		 * j = first error position
		 * k = second error position
		 * a = primitive element of finite field
		 * f = error magnitudes
		 * We want to know f0 and f1
		 *
		 * | a^0^j a^0^k | * | f0 | = | s0 |
		 * | a^1^j a^1^k |   | f1 |   | s1 |
		 * | a^2^j a^2^k |            | s2 |
		 * | a^3^j a^3^k |            | s3 |
		 *
		 * Example for t==2
		 * | a^0^j | * | f0 | = | s0 |
		 * | a^1^j |            | s1 |
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

		/*
		 * We know know the error positions and the error magnitudes.
		 * Let's create a polynomial which we can subtract from the received data to recreate something
		 * which should resemble the originaly sent data.
		 */
		Polynom<FF> error_correction;
		for (int i = 0; i < error_number; i++)
		{
			error_correction.setCoefficient(error_positions.at(i), errorMagnitudes.at(i));
		}
		// std::cout << "error_correction " << error_correction.asString() << std::endl;
		// std::cout << "message " << message.asString() << std::endl;
		Polynom<FF> repaired_message = message - error_correction;
		// std::cout << "repaired_message " << repaired_message.asString() << std::endl;

		/*
		 * Let us calculate the syndromes again to double check our operation.
		 * This is not required but helps testing the algorithm.
		 */
		std::tie(syndromes, allSyndromesZero) = calculateSyndromes(repaired_message);

		if (allSyndromesZero)
		{
			// std::cout << "Looks correct now after fixing " << error_number << " errors!\n";
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
