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

/**
 * Utility class to provide raw data in the form of a bit stream for easier access with unconvential data type sizes.
 */
class BitStreamReader
{
  private:
	/// Raw Data for the stream
	std::vector<uint8_t> data;

	/// Current index of \ref data
	int byte_pos{0};

	/// Current bit position for the current \ref byte_pos
	int bit_pos{0};

  public:
	/**
	 * Construct with some data to deliver
	 * @param in	Raw data
	 */
	BitStreamReader(std::vector<uint8_t> in) : data(in)
	{
	}

	/**
	 * Provides number of bits which can still be fetched from this.
	 */
	int remainingBits()
	{
		return (data.size() - byte_pos - 1) * 8 + (8 - bit_pos);
	}

	/**
	 * Current reading status
	 * @return	true if end of stream is reached
	 */
	bool isEof()
	{
		return (byte_pos >= data.size());
	}

	/**
	 * Fetch a word of given size from the bit stream
	 *
	 * @param size	Size in bits
	 */
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
