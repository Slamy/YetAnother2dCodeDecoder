/*
 * test_datamatrix.cpp
 *
 *  Created on: 12.11.2021
 *      Author: andre
 */

#include "DataMatrixDecoder.h"
#include <gtest/gtest.h>

struct GroundTruth
{
	const char* filename;
	const char* reference_payload;
	bool syndromesExpectedZero;
	std::vector<int>& positionRef;
};

std::vector<int> emptyRef;

std::vector<int> posRef2{
	1,	  5,  // Standard Case
	3,	  3,  // Standard Case
	5,	  1,  // Standard Case
	8,	  2,  // Standard Case
	6,	  4,  // Standard Case
	4,	  6,  // Standard Case
	2,	  8,  // Standard Case
	1,	  13, // Standard Case
	3,	  11, // Standard Case
	5,	  9,  // Standard Case
	7,	  7,  // Standard Case
	9,	  5,  // Standard Case
	11,	  3,  // Standard Case
	13,	  1,  // Standard Case
	16,	  2,  // Standard Case
	14,	  4,  // Standard Case
	12,	  6,  // Standard Case
	10,	  8,  // Standard Case
	8,	  10, // Standard Case
	6,	  12, // Standard Case
	4,	  14, // Standard Case
	2,	  16, // Standard Case
	-100,	  // Corner Case
	3,	  19, // Standard Case
	5,	  17, // Standard Case
	7,	  15, // Standard Case
	9,	  13, // Standard Case
	11,	  11, // Standard Case
	13,	  9,  // Standard Case
	15,	  7,  // Standard Case
	17,	  5,  // Standard Case
	19,	  3,  // Standard Case
	20,	  6,  // Standard Case
	18,	  8,  // Standard Case
	16,	  10, // Standard Case
	14,	  12, // Standard Case
	12,	  14, // Standard Case
	10,	  16, // Standard Case
	8,	  18, // Standard Case
	6,	  20, // Standard Case
	11,	  19, // Standard Case
	13,	  17, // Standard Case
	15,	  15, // Standard Case
	17,	  13, // Standard Case
	19,	  11, // Standard Case
	20,	  14, // Standard Case
	18,	  16, // Standard Case
	16,	  18, // Standard Case
	14,	  20, // Standard Case
	19,	  19, // Standard Case
};
std::vector<int> posRef4{
	1,	5,	// Standard Case
	3,	3,	// Standard Case
	5,	1,	// Standard Case
	8,	2,	// Standard Case
	6,	4,	// Standard Case
	4,	6,	// Standard Case
	2,	8,	// Standard Case
	5,	9,	// Standard Case
	7,	7,	// Standard Case
	9,	5,	// Standard Case
	10, 8,	// Standard Case
	8,	10, // Standard Case

};
std::vector<int> posRef3{
	1, 5, // Standard Case
	3, 3, // Standard Case
	5, 1, // Standard Case
	8, 2, // Standard Case
	6, 4, // Standard Case
	4, 6, // Standard Case
	2, 8, // Standard Case
	7, 7, // Standard Case
};

std::vector<int> posRef{
	1,	  5,  // Standard Case
	3,	  3,  // Standard Case
	5,	  1,  // Standard Case
	8,	  2,  // Standard Case
	6,	  4,  // Standard Case
	4,	  6,  // Standard Case
	2,	  8,  // Standard Case
	1,	  13, // Standard Case
	3,	  11, // Standard Case
	5,	  9,  // Standard Case
	7,	  7,  // Standard Case
	9,	  5,  // Standard Case
	11,	  3,  // Standard Case
	13,	  1,  // Standard Case
	16,	  2,  // Standard Case
	14,	  4,  // Standard Case
	12,	  6,  // Standard Case
	10,	  8,  // Standard Case
	8,	  10, // Standard Case
	6,	  12, // Standard Case
	4,	  14, // Standard Case
	2,	  16, // Standard Case
	-100,	  // Corner Case
	3,	  19, // Standard Case
	5,	  17, // Standard Case
	7,	  15, // Standard Case
	9,	  13, // Standard Case
	11,	  11, // Standard Case
	13,	  9,  // Standard Case
	15,	  7,  // Standard Case
	17,	  5,  // Standard Case
	19,	  3,  // Standard Case
	22,	  4,  // Standard Case
	20,	  6,  // Standard Case
	18,	  8,  // Standard Case
	16,	  10, // Standard Case
	14,	  12, // Standard Case
	12,	  14, // Standard Case
	10,	  16, // Standard Case
	8,	  18, // Standard Case
	6,	  20, // Standard Case
	4,	  22, // Standard Case
	9,	  21, // Standard Case
	11,	  19, // Standard Case
	13,	  17, // Standard Case
	15,	  15, // Standard Case
	17,	  13, // Standard Case
	19,	  11, // Standard Case
	21,	  9,  // Standard Case
	22,	  12, // Standard Case
	20,	  14, // Standard Case
	18,	  16, // Standard Case
	16,	  18, // Standard Case
	14,	  20, // Standard Case
	12,	  22, // Standard Case
	17,	  21, // Standard Case
	19,	  19, // Standard Case
	21,	  17, // Standard Case
	22,	  20, // Standard Case
	20,	  22, // Standard Case

};

static struct GroundTruth data[] = {
	{"wikipedia.png", "Wikipedia, the free encyclopedia", true, posRef2},
	{"wikipedia_perspective.png", "Wikipedia, the free encyclopedia", true, posRef2},
	{"datamatrix_link_en_big.png", "http://www.free-barcode-generator.net/", true, posRef},
	{"gen_abcd.png", "ABCD", true, posRef4},
	{"gen_abc.png", "ABC", true, posRef3},
	{"gen_abcdefg.png", "ABCDEFG", true, emptyRef},
	{"gen_a2k.png", "ABCDEFGHIJK", true, emptyRef},
	{"gen_a2q.png", "ABCDEFGHIJKLMNOPQ", true, emptyRef},
	{"gen_a2z.png", "ABCDEFGHIJKLMNOPQRSTUVWXYZ", true, emptyRef},

};

TEST(DmTest, Validate)
{
	DataMatrixDecoder dm;
	std::vector<char> result;

	for (auto& g : data)
	{
		std::stringstream path;
		path << "/home/andre/GIT/QrCode/datamatrix_ref/" << g.filename;

		std::cout << "Reading " << g.filename << std::endl;

		dm.debugPosRef	 = g.positionRef;
		dm.debugPosRefIt = dm.debugPosRef.begin();

		result = dm.decodeFromFile(path.str().c_str());

		result.push_back('\0');
		ASSERT_STREQ(result.data(), g.reference_payload);

		ASSERT_EQ(dm.code_needed_correction_, !g.syndromesExpectedZero);

		std::cout << std::endl;
	}
}
