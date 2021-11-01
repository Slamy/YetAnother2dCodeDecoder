/*
 * BitStreamReader.h
 *
 *  Created on: 01.11.2021
 *      Author: andre
 */

#ifndef BITSTREAMREADER_H_
#define BITSTREAMREADER_H_

#include <cstdint>
#include <vector>

class BitStreamReader
{
  private:
	std::vector<uint8_t> data;
	int byte_pos{0};
	int bit_pos{0};

  public:
	BitStreamReader(std::vector<uint8_t> in) : data(in)
	{
	}

	int remainingBits()
	{
		return (data.size() - byte_pos - 1) * 8 + (8 - bit_pos);
	}

	bool isEof()
	{
		return (byte_pos >= data.size());
	}

	int readWord(int size)
	{
		int result = 0;

		while (size--)
		{
			result <<= 1;
			if (data.at(byte_pos) & 0x80)
				result |= 1;

			data.at(byte_pos) <<= 1;
			bit_pos++;

			if (bit_pos == 8)
			{
				bit_pos = 0;
				byte_pos++;
			}
		}

		return result;
	}
};

#endif /* BITSTREAMREADER_H_ */
