/**
 * @file QrDecoder.h
 *
 *  Created on: 30.10.2021
 *      Author: andre
 */

#ifndef QRDECODER_H_
#define QRDECODER_H_

#include "BitStreamReader.h"
#include "ExtFiniteField256.h"
#include "Matrix.h"
#include "ReedSolomon.h"
#include "glm/gtx/vector_angle.hpp"
#include "glm/vec2.hpp"
#include "util.h"
#include <decoderexception.h>
#include <numeric>
#include <opencv2/opencv.hpp>

/**
 * Helper class for \ref QrDecoder to store data about collected finder patterns.
 * Compared to simply being points, this class stores also the orientation of the Finder Pattern.
 */
class FinderPattern
{
  public:
	/// Position of the corner of the Finder pattern which shares the position with the corner of the Qr Code itself
	cv::Point outer;

	/// Position of the corner of the Finder pattern on the opposite side of \ref outer.
	cv::Point inner;

	/// Normalized vector from \ref outer to \ref inner
	glm::vec2 out_to_in;

	/// Remaining two corners of the Finder pattern
	std::vector<cv::Point> sides{2};

	/// Not normalized vector which points from \ref outer to the corner of the Qr Code which doesn't have the Finder
	/// pattern.
	cv::Point outer_to_finderless_corner;

	FinderPattern()
	{
	}

	/**
	 * Create a representation of a Qr code finder pattern
	 * @param in		Corners of the finder pattern. Must contain 4 points.
	 * @param center	Should be on average in the middle of the Qr Code. Used to identify orientation of the Finder
	 * 					Pattern.
	 */
	FinderPattern(const std::vector<cv::Point>& in, cv::Point center)
	{
		int innerst_id	= -1;
		int innerstdist = 9999;

		assert(in.size() == 4);
		for (int i = 0; i < in.size(); i++)
		{
			if (cv::norm(in.at(i) - center) < innerstdist)
			{
				innerstdist = cv::norm(in.at(i) - center);
				innerst_id	= i;
			}
		}
		assert(innerst_id >= 0);

		inner	 = in.at(innerst_id);
		outer	 = in.at((innerst_id + 2) % 4);
		sides[0] = in.at((innerst_id + 1) % 4);
		sides[1] = in.at((innerst_id + 3) % 4);

		out_to_in = glm::vec2(inner.x - outer.x, inner.y - outer.y);
		out_to_in = glm::normalize(out_to_in);
	}

	/**
	 * Function must only be called for the finder pattern on the top right or left bottom.
	 * Calculates \ref outer_to_finderless_corner for later use
	 * @param top_left_pattern	Ref to \ref FinderPattern on the top left
	 */
	void calculateFinderLessEdgeAngle(FinderPattern& top_left_pattern)
	{
		if (cv::norm(top_left_pattern.outer - sides.at(0)) > cv::norm(top_left_pattern.outer - sides.at(1)))
			outer_to_finderless_corner = sides.at(0) - outer;
		else
			outer_to_finderless_corner = sides.at(1) - outer;
	}
};

/**
 * Qr Code Decoder
 *
 * Doesn't supports a lot of versions and sizes. Just for academic purposes an intended for understandability
 *
 * http://www.cdsy.de/QR_Start.html
 * https://www.swetake.com/qrcode/qr2_en.html
 * http://www.cdsy.de/Arithmetik/DecodeTXT/QR_DE_Start.php
 */
class QrDecoder
{
  private:
	/// Prepare the object for another code to read.
	void reset()
	{
		contours_.clear();
		hierarchy_.clear();
		finder_contour_points_.clear();
		alignment_pattern_centers_.clear();
		finders_orientated.clear();

		cells.clear();
		debug_cells.clear();

		going_up_	= true;
		change_row_ = false;
	}

	/// OpenCV Image which was fed to this decoder.
	cv::Mat src_image_;

	/// OpenCV Image of the perspective corrected code, after the shape was reconstructed.
	cv::Mat monochrome_image_warped_;

	/// For detection of the Finder Patterns, \ref monochrome_image_warped_ is vectorized and stored
	/// in this as a vector of polygons.
	std::vector<std::vector<cv::Point>> contours_;

	/// The vectorizaton process here, stores the result as a tree of polygons.
	/// This is used to detect a finder pattern as a square inside a square inside a square.
	std::vector<cv::Vec4i> hierarchy_;

	/// Possible 4 corners of finder patterns are stored here for later processing.
	/// The order and orientation of points is undefined in this state.
	std::vector<std::vector<cv::Point>> finder_contour_points_;

	/// Possible alignment patterns are stored here.
	/// Those were squares in squares which were not detected as Finder Pattern
	std::vector<cv::Point> alignment_pattern_centers_;

	/// On average near the center of the Qr Code
	cv::Point average_center_pos;

	/// All found Finder patterns whose orientation could be extracted.
	std::vector<FinderPattern> finders_orientated;

	/// Width and Height of perspective corrected image
	static constexpr int warped_size = 1000;

	/// Width of a Cell, a cell being a "pixel" of the code
	float cell_size_x_;
	/// Height of a Cell, a cell being a "pixel" of the code
	float cell_size_y_;

	/// Usually half of \ref cell_size_x_. Used as a starting offset in X and Y direction to scan \ref
	/// monochrome_image_warped_ for cells.
	float half_cell_size_;

	/// Transformation matrix which is used rectify the code to full size
	cv::Mat transform;

	/// Vector of rows of with true being a black cell
	std::vector<std::vector<bool>> cells;
	/// Vector of rows of printable characters. Only used for development and debugging to check
	/// which portion of the code belongs to which data byte
	std::vector<std::vector<char>> debug_cells;

	/// Internal debugging variable. Only used for \ref debug_cells
	char debug_cell_val_ = '_';

	/// Width and Height of the code in "cells"
	int size_in_cells_;

	/// Used by \ref readBitFromCells to keep track of the current cell in decoding process.
	cv::Point current_cell_position_;

	/// Only used for debugging to provide textual representation for the error correction level
	const char* correctionLevelStr[4] = {"M", "L", "H", "Q"};

	static constexpr int kErrM = 0; ///< up to 15% damage
	static constexpr int kErrL = 1; ///< up to 7% damage
	static constexpr int kErrH = 2; ///< up to 30% damage
	static constexpr int kErrQ = 3; ///< up to 25% damage

	/// Product of Width and Height of monochrome_image_inverted as used in \ref decodeFromFile
	float monochrome_image_inverted_area;

	/// Qr code data is stored in columns with a width of 2 cells. If true, the next column is expected
	/// to be above the current one
	bool going_up_ = true;

	/**
	 * With each column of 2 cells to read from the code, we start with the right bit.
	 * This flag toggles with each fetched cell and indicates that we need to change row.
	 *
	 * The order of bits is like so.
	 * 76
	 * 54
	 * 32
	 * 10
	 *
	 * If false, the next cell is to the left of the current one.
	 * If true, we need to get diagonally up right.
	 *
	 */
	bool change_row_ = false;

	/// Number of data byte blocks in the code, which are stored interleaved for improved
	/// error correction.
	int db_blocks_;

	/**
	 * Lookup Table to help with the de-interleaving of the data byte blocks.
	 * The access in the end will be like so:
	 * rawData.at(db_deinterleaving_lut_.at(block).at(i))
	 * i is the offset of the byte which we want to have from the provided block.
	 */
	std::vector<std::vector<int>> db_deinterleaving_lut_;

	/// Number of data bytes in total to get from the code
	int number_of_Db_;

	/// Sum of the number of data bytes and error correction bytes
	int raw_db_plus_ecb;

	/// Collection of all data, which can be extracted from the Format area of the code
	/// as well as the BCH code used to describe it in the code
	struct FormatCode
	{
		int err;	   ///< Error correction level
		int mask;	   ///< Type of mask. Refer to \ref applyMask for more info
		uint32_t code; ///< raw word used to describe this format
	};

	/// Format of the code, which is currently processed
	struct FormatCode detected_format_code_;

	/// used for a database to look up config data for any supported version of code
	struct QrConfig
	{
		int version;			///< Version of Qr Code. Correlates with the size.
		int correctionLevel;	///< Error correction level from 0 to 3
		int ecbPerBlock;		///< Error correction bytes per block of data
		int blocksInGroup1;		///< Number of blocks in group 1
		int dbPerBlockInGroup1; ///< Data bytes per block in group 1
		int blocksInGroup2;		///< Number of blocks in group 2
		int dbPerBlockInGroup2; ///< Data bytes per block in group 2
	};

	/// Configuration of the code, which is currently process
	struct QrConfig detected_qr_config_;

	/**
	 * Currently supported versions and configurations to decode.
	 * Fetched from http://www.cdsy.de/QR-Vorgaben.html
	 * TODO still a lot configurations missing
	 */
	const std::initializer_list<QrConfig> configs_{
		{2, kErrM, 16, 1, 28, 0, 0}, //
		{5, kErrM, 24, 2, 43, 0, 0}, //

		{1, kErrL, 7, 1, 19, 0, 0},	  //
		{2, kErrL, 10, 1, 34, 0, 0},  //
		{5, kErrL, 26, 1, 108, 0, 0}, //

		{4, kErrH, 16, 4, 9, 0, 0},	  //
		{5, kErrH, 22, 2, 11, 2, 12}, //

		{4, kErrQ, 26, 2, 24, 0, 0}, //
	};

	/**
	 * Based on the size of the code, the version is calculated. \ref configs_ ist checked for a fitting
	 * configuration. If a fitting config was found \ref detected_qr_config_ is updated.
	 * If not, an exception is thrown.
	 */
	void grabQrConfig()
	{
		// sizeInCells = version * 4 + 17;
		// (sizeInCells - 17) / 4 = version
		int version = (size_in_cells_ - 17) / 4;
		assert(((size_in_cells_ - 17) % 4) == 0);

		for (auto& c : configs_)
		{
			if (c.version == version && detected_format_code_.err == c.correctionLevel)
			{
				detected_qr_config_ = c;

				raw_db_plus_ecb = (c.ecbPerBlock + c.dbPerBlockInGroup1) * c.blocksInGroup1 +
								  (c.ecbPerBlock + c.dbPerBlockInGroup2) * c.blocksInGroup2;
				return;
			}
		}

		if (debugMode)
		{
			printf("Unable to find config for version %d with correction level %s\n", version,
				   correctionLevelStr[detected_format_code_.err]);
		}
		throw DecoderException(DecoderException::Cause::kVersionNotSupported);
	}

	/**
	 * Utility function to detect rounded squares.
	 *
	 * @param out2	Contour with variable number of points
	 * @return		true, if probably a square
	 */
	bool isKindaSquare(std::vector<cv::Point> out2)
	{
		// Start by calculating the maximum length of an edge
		int maxLen = 0;
		for (int i = 0; i < out2.size(); i++)
		{
			int j	= (i + 1) % out2.size();
			int len = cv::norm(out2.at(i) - out2.at(j));
			if (len > maxLen)
				maxLen = len;

			if (debugMode)
			{
				printf("r %d %f\n", i, cv::norm(out2.at(i) - out2.at(j)));
			}
		}

		// Now count the number of edges which have at least
		// half of the maximum length
		int longEdges = 0;
		for (int i = 0; i < out2.size(); i++)
		{
			int j	= (i + 1) % out2.size();
			int len = cv::norm(out2.at(i) - out2.at(j));
			if (len > maxLen / 2)
			{
				longEdges++;
			}
		}

		// Only if we have found exactly 4 edges that fit the requirement, we can assume that this is a square.
		return longEdges == 4;
	}

	/**
	 * Is very similar to \ref isKindaSquare but even calculates the square which we are seeing here
	 * by removing rounded edges using \ref calculateLineIntercross
	 *
	 * @param inp	Contour with variable number of points
	 * @return		Empty vector in case of failure, exactly 4 points in case of success
	 */
	std::vector<cv::Point> reconstructSquare(std::vector<cv::Point> inp)
	{
		std::vector<cv::Point> out2;
		cv::approxPolyDP(inp, out2, 3, true);

		// Calculate longest edge
		int maxLen = 0;
		for (int i = 0; i < out2.size(); i++)
		{
			int j	= (i + 1) % out2.size();
			int len = cv::norm(out2.at(i) - out2.at(j));
			if (len > maxLen)
				maxLen = len;

			if (debugMode)
			{
				printf("r %d %f\n", i, cv::norm(out2.at(i) - out2.at(j)));
			}
		}

		// Collect "long" edges
		int longEdges = 0;
		std::vector<cv::Point> corner_points;
		std::vector<cv::Point> corner_dir;
		for (int i = 0; i < out2.size(); i++)
		{
			int j	= (i + 1) % out2.size();
			int len = cv::norm(out2.at(i) - out2.at(j));
			if (len > maxLen / 2)
			{
				longEdges++;
				corner_points.push_back(out2.at(i));
				corner_dir.push_back(out2.at(j) - out2.at(i));
			}
		}

		// Do we have 4 edges? Nice! This is a square.. probably
		// Calculate the 4 intercrossing positions of the 4 edges to get a sharp edged square.
		std::vector<cv::Point> out_points;
		if (longEdges == 4)
		{
			for (int i = 0; i < 4; i++)
			{
				out_points.push_back(calculateLineIntercross(
					corner_points.at(i), corner_dir.at(i), corner_points.at((i + 1) % 4), corner_dir.at((i + 1) % 4)));
			}
		}

		return out_points;
	}

	/**
	 * Based on the vector graphic presentation of the image, we try to find a finder pattern.
	 * We do this by searching for contours with 4 corners which are about the shape of a square.
	 * Noise is filtered from the contour via cv::approxPolyDP before.
	 *
	 * We define a Finder Pattern as a 4 corner shape inside a 4 corner shape inside another 4 corner shape.
	 * This function is recursive and calls itself with an incremented version of candidateForFinderPattern
	 * if the current contour is assumed to be a finder pattern square.
	 *
	 * This function performs a depth search on the hierarchy of contours.
	 *
	 * @param id 							Current index inside \ref contours_
	 * @param candidateForFinderPattern		Usually 0. Any higher number represents how deep we are inside an assumed
	 * 										finder pattern
	 */
	void searchFinderPatternPoints(int id, int candidateForFinderPattern)
	{
		for (;;)
		{
			double epsilon = 0.05 * cv::arcLength(contours_.at(id), true);
			std::vector<cv::Point> out;
			// printf("epsilon %lf\n", epsilon);
			cv::approxPolyDP(contours_.at(id), out, epsilon, true);

#if 0
			for (auto& v : out)
			{
				cv::circle(src_image_, v, 10, green, 2);
			}
#endif

			int nextparam = 0;

			float area = cv::contourArea(out);

			if (out.size() == 4 && area < monochrome_image_inverted_area / 8)
			{
				if (debugMode)
				{
					printf("Contour %d %d %d %d %lf\n", id, contours_.at(id).size(), out.size(),
						   candidateForFinderPattern, area);
				}

				if (candidateForFinderPattern == 2)
				{
					int outer1 = hierarchy_.at(id)[3];
					int outer2 = hierarchy_.at(outer1)[3];
					std::vector<cv::Point> out2;

					assert(contours_.at(outer2).size() >= 4);

					cv::approxPolyDP(contours_.at(outer2), out2, 3, true);

					// cv::polylines(src_image_, contours.at(outer2), true, red, 5, cv::LINE_AA);
					// cv::polylines(src_image_, contours.at(outer1), true, red, 5, cv::LINE_AA);

					if (out2.size() == 4)
					{
						if (debugMode)
						{
							printf("Direct use!\n");
						}
						finder_contour_points_.push_back(out2);
					}
					else
					{
						if (debugMode)
						{
							printf("reconstructSquare!\n");
						}
						auto square = reconstructSquare(contours_.at(outer2));

						if (square.size() == 4)
						{
							finder_contour_points_.push_back(square);
						}
					}
				}
				else if (candidateForFinderPattern == 1)
				{

					int outer1 = hierarchy_.at(id)[3];
					int inner  = hierarchy_.at(id)[2];

					if (inner == -1)
					{
						std::vector<cv::Point> out2;
						double epsilon = 0.02 * cv::arcLength(contours_.at(outer1), true);
						cv::approxPolyDP(contours_.at(outer1), out2, epsilon, true);

						if (out2.size() == 4 && isKindaSquare(out2))
						{
							// cv::polylines(src_image_, out2, true, red, 4, cv::LINE_AA);
							// cv::polylines(src_image_, out, true, red, 1, cv::LINE_AA);

							cv::Point center = calculateAveragePosition(out);

							if (debugMode)
							{
								printf("Alignment Pattern at %d %d    %d %d\n", center.x, center.y, out.size(),
									   out2.size());
							}

							cv::circle(src_image_, center, 4, red, 2);
							alignment_pattern_centers_.push_back(center);
						}
					}
				}

				// candidateForFinderPattern++;
				nextparam = candidateForFinderPattern + 1;
			}
			else
			{
				nextparam = 0;
			}

			if (hierarchy_.at(id)[2] >= 0)
			{
				searchFinderPatternPoints(hierarchy_.at(id)[2], nextparam);
			}

			// Next contour on this level
			id = hierarchy_.at(id)[0];
			if (id == -1)
				break;
		}
	}

	/**
	 * To improve scanning, the Qr code encoder tries to have evenly distributed black and white cells.
	 * This is done by selectively inverting the code during the encoding process using a mask.
	 * 8 different masks are available.
	 * This function restores the original pattern.
	 * @param mask Number between 0 to 7
	 */
	void applyMask(int mask)
	{
		/*
		 * from http://www.cdsy.de/masken_2.html
		 *
		 * Zeilennummer i; Spaltennummer j ... jeweils ab 0 gezählt; Start links oben
		 * Maske 0:   if (  (i + j) % 2 == 0 )
		 * Maske 1:   if ( i % 2 == 0 )
		 * Maske 2:   if ( j % 3 == 0 )
		 * Maske 3:   if (  (i + j) % 3 == 0 )
		 * Maske 4:   if (  (  (i / 2) + (j / 3)  ) % 2 == 0 )
		 * Maske 5:   if (  (  (i * j) % 2  ) + (  (i * j) % 3  ) == 0 )
		 * Maske 6:   if (  (  (  (i * j) % 2  ) + (  (i * j) % 3)  ) % 2 == 0 )
		 * Maske 7:   if (  (  (  (i + j) % 2  ) + (  (i * j) % 3)  ) % 2 == 0 )
		 */
		std::function<bool(int, int)> invert;

		switch (mask)
		{
		case 0:
			invert = [](int i, int j) { return ((i + j) % 2 == 0); };
			break;
		case 1:
			invert = [](int i, int j) { return (i % 2 == 0); };
			break;
		case 2:
			invert = [](int i, int j) { return (j % 3 == 0); };
			break;
		case 3:
			invert = [](int i, int j) { return ((i + j) % 3 == 0); };
			break;
		case 4:
			invert = [](int i, int j) { return (((i / 2) + (j / 3)) % 2 == 0); };
			break;
		case 5:
			invert = [](int i, int j) { return (((i * j) % 2) + ((i * j) % 3) == 0); };
			break;
		case 6:
			invert = [](int i, int j) { return ((((i * j) % 2) + ((i * j) % 3)) % 2 == 0); };
			break;
		case 7:
			invert = [](int i, int j) { return ((((i + j) % 2) + ((i * j) % 3)) % 2 == 0); };
			break;
		default:
			assert(0);
		}

		for (int y = 0; y < size_in_cells_; y++)
		{
			for (int x = 0; x < size_in_cells_; x++)
			{
				if (isCellDataBit(x, y) && invert(y, x))
					cells.at(y).at(x) = !cells.at(y).at(x);
			}
		}
	}

	/**
	 * Calculates the average position for all provided points
	 * @param in	Points to calculate the average from
	 * @return		Average positon
	 */
	static cv::Point calculateAveragePosition(std::vector<cv::Point> in)
	{
		cv::Point result(0, 0);
		for (const auto& p : in)
		{
			result.x += p.x;
			result.y += p.y;
		}

		result.x /= (in.size());
		result.y /= (in.size());

		return result;
	}

	/**
	 * Tries to find the center of the code, based on assumed finder counter positions.
	 * Later used to calculate the orientation of the finder patterns.
	 */
	void assumeFinderPatternCenter()
	{
		average_center_pos.x = 0;
		average_center_pos.y = 0;

		for (const auto& c : finder_contour_points_)
		{
			assert(c.size() == 4);
			// cv::polylines(src_image_, c, true, green, 1, cv::LINE_AA);

			for (const auto& p : c)
			{
				average_center_pos.x += p.x;
				average_center_pos.y += p.y;
			}
		}

		average_center_pos.x /= (finder_contour_points_.size() * 4);
		average_center_pos.y /= (finder_contour_points_.size() * 4);
	}

	/**
	 * At this point we have the position and orientation of the finder patterns.
	 * Using this knowledge, we can calculate the missing corner of the code at the bottom right
	 * and also perform perspective correction to stretch the detected code to a full size version
	 * for easier extraction of cells.
	 */
	void reconstructShapeOfCode()
	{
		assert(finder_contour_points_.size() == 3);

		cv::circle(src_image_, average_center_pos, 10, red, 3);

		for (const auto& c : finder_contour_points_)
		{
			finders_orientated.emplace_back(c, average_center_pos);
#if 0
			FinderPattern pattern(c, average_center_pos);
			cv::circle(src_image_, pattern.inner, 10, green, 3);
			cv::circle(src_image_, pattern.outer, 10, red, 3);
			cv::circle(src_image_, pattern.sides[0], 10, blue, 3);
			cv::circle(src_image_, pattern.sides[1], 10, blue, 3);
#endif

			// cv::circle(image, pattern.inner + pattern.out_to_in, 20, blue, 3);
		}

		if (debugMode)
		{
			printf("%f %f\n", finders_orientated.at(2).out_to_in.x, finders_orientated.at(2).out_to_in.y);
			printf("%f %f\n", finders_orientated.at(0).out_to_in.x, finders_orientated.at(0).out_to_in.y);
			printf("%f %f\n", finders_orientated.at(1).out_to_in.x, finders_orientated.at(1).out_to_in.y);
		}
		float angle01, angle02, angle12;

		angle01 = glm::orientedAngle(finders_orientated.at(0).out_to_in, finders_orientated.at(1).out_to_in);
		angle02 = glm::orientedAngle(finders_orientated.at(0).out_to_in, finders_orientated.at(2).out_to_in);
		angle12 = glm::orientedAngle(finders_orientated.at(1).out_to_in, finders_orientated.at(2).out_to_in);

		if (debugMode)
		{
			printf("%f %f %f\n", angle01, angle02, angle12);
		}

		FinderPattern finder_topLeft;
		FinderPattern finder_bottomLeft;
		FinderPattern finder_topRight;

		if (fabs(angle01) > 3)
		{
			finder_topLeft = finders_orientated.at(2);
			if (angle02 > 0)
			{
				finder_bottomLeft = finders_orientated.at(0);
				finder_topRight	  = finders_orientated.at(1);
			}
			else
			{
				finder_bottomLeft = finders_orientated.at(1);
				finder_topRight	  = finders_orientated.at(0);
			}
		}
		else if (fabs(angle02) > 3)
		{
			finder_topLeft = finders_orientated.at(1);
			if (angle01 > 0)
			{
				finder_bottomLeft = finders_orientated.at(0);
				finder_topRight	  = finders_orientated.at(2);
			}
			else
			{
				finder_bottomLeft = finders_orientated.at(2);
				finder_topRight	  = finders_orientated.at(0);
			}
		}
		else if (fabs(angle12) > 3)
		{
			finder_topLeft = finders_orientated.at(0);
			if (angle01 < 0)
			{
				finder_bottomLeft = finders_orientated.at(1);
				finder_topRight	  = finders_orientated.at(2);
			}
			else
			{
				finder_bottomLeft = finders_orientated.at(2);
				finder_topRight	  = finders_orientated.at(1);
			}
			// TODO must still be developed
		}
		else
		{
			assert(0);
		}

		if (debugMode)
		{
			printf("finder top left %d %d\n", finder_topLeft.outer.x, finder_topLeft.outer.y);
			printf("finder top right %d %d\n", finder_topRight.outer.x, finder_topRight.outer.y);
			printf("finder bottom left %d %d\n", finder_bottomLeft.outer.x, finder_bottomLeft.outer.y);
			// cv::circle(image, finder2.at(1).outer, 10, red, 3);
		}
		finder_bottomLeft.calculateFinderLessEdgeAngle(finder_topLeft);
		finder_topRight.calculateFinderLessEdgeAngle(finder_topLeft);

#if 0
		cv::circle(src_image_, finder_bottomLeft.outer, 10, blue, 3);
		cv::circle(src_image_, finder_bottomLeft.outer + finder_bottomLeft.outer_to_finderless_corner, 10, green, 3);

		cv::circle(src_image_, finder_topRight.outer, 10, red, 3);
		cv::circle(src_image_, finder_topRight.outer + finder_topRight.outer_to_finderless_corner, 10, green, 3);

		cv::circle(src_image_, finder_topLeft.outer, 10, red, 3);
		cv::circle(src_image_, finder_topLeft.inner, 10, green, 3);
#endif

		cv::Point solution;
		cv::Point avg_solution(0, 0);
		float avg_num = 0;

		solution = calculateLineIntercross(finder_bottomLeft.outer, finder_bottomLeft.outer_to_finderless_corner,
										   finder_topRight.outer, finder_topRight.outer_to_finderless_corner);
		avg_solution += solution;

		if (debugMode)
		{
			printf("edge of code %d %d\n", solution.x, solution.y);
			cv::circle(src_image_, solution, 5, red, 1);
		}
		avg_num++;

		if (alignment_pattern_centers_.size() > 0)
		{
			solution =
				calculateLineIntercross(finder_topLeft.outer, alignment_pattern_centers_.at(0) - finder_topLeft.outer,
										finder_topRight.outer, finder_topRight.outer_to_finderless_corner);
			avg_solution += solution;
			if (debugMode)
			{
				printf("edge of code %d %d\n", solution.x, solution.y);
				cv::circle(src_image_, solution, 5, red, 1);
			}
			avg_num++;

			solution =
				calculateLineIntercross(finder_bottomLeft.outer, finder_bottomLeft.outer_to_finderless_corner,
										finder_topLeft.outer, alignment_pattern_centers_.at(0) - finder_topLeft.outer);
			avg_solution += solution;
			if (debugMode)
			{
				printf("edge of code %d %d\n", solution.x, solution.y);
				cv::circle(src_image_, solution, 5, red, 1);
			}
			avg_num++;
		}
		solution.x = avg_solution.x / avg_num;
		solution.y = avg_solution.y / avg_num;

		cv::circle(src_image_, solution, 3, green, 2);

		cv::Point2f src[4];
		cv::Point2f dst[4];

		src[0] = finder_topLeft.outer;
		src[1] = finder_topRight.outer;
		src[2] = solution;
		src[3] = finder_bottomLeft.outer;

		std::array<cv::Point, 4> srcDraw;
		srcDraw[0] = finder_topLeft.outer;
		srcDraw[1] = finder_topRight.outer;
		srcDraw[2] = solution;
		srcDraw[3] = finder_bottomLeft.outer;
		cv::polylines(src_image_, srcDraw, true, red, 1, cv::LINE_AA);

		dst[0] = cv::Point2f(0, 0);
		dst[1] = cv::Point2f(warped_size, 0);
		dst[2] = cv::Point2f(warped_size, warped_size);
		dst[3] = cv::Point2f(0, warped_size);

		transform = cv::getPerspectiveTransform(src, dst);

		// std::array<cv::Point2f, 2> src2{finder_bottomLeft.outer, finder_bottomLeft.sides.at(0)};
		std::array<cv::Point2f, 12> src2{
			finder_topLeft.outer,		//
			finder_topLeft.sides.at(0), //
			finder_topLeft.sides.at(1), //
			finder_topLeft.inner,		//

			finder_bottomLeft.outer,	   //
			finder_bottomLeft.sides.at(0), //
			finder_bottomLeft.sides.at(1), //
			finder_bottomLeft.inner,	   //

			finder_topRight.outer,		 //
			finder_topRight.sides.at(0), //
			finder_topRight.sides.at(1), //
			finder_topRight.inner,		 //
		};
		std::array<cv::Point2f, 12> dst2;

		cv::perspectiveTransform(src2, dst2, transform);

		std::vector<float> avgs;
		avgs.push_back(cv::norm(dst2[1] - dst2[0]));
		avgs.push_back(cv::norm(dst2[2] - dst2[0]));
		avgs.push_back(cv::norm(dst2[1] - dst2[3]));
		avgs.push_back(cv::norm(dst2[2] - dst2[3]));

		avgs.push_back(cv::norm(dst2[5] - dst2[4]));
		avgs.push_back(cv::norm(dst2[6] - dst2[4]));
		avgs.push_back(cv::norm(dst2[5] - dst2[7]));
		avgs.push_back(cv::norm(dst2[6] - dst2[7]));

		avgs.push_back(cv::norm(dst2[9] - dst2[8]));
		avgs.push_back(cv::norm(dst2[10] - dst2[8]));
		avgs.push_back(cv::norm(dst2[9] - dst2[11]));
		avgs.push_back(cv::norm(dst2[10] - dst2[11]));

		cell_size_x_ = (std::accumulate(avgs.begin(), avgs.end(), 0) / avgs.size()) / 7.0f;
		cell_size_y_ = cell_size_x_;
		// std::sort(avgs.begin(), avgs.end());
		// cellsize = avgs.at(avgs.size() / 2) / 7.0f;

		// auto cellsize	   = cv::norm(dst2[1] - dst2[0]) / 7.0f;
		half_cell_size_ = cell_size_x_ / 2.0f;
		if (debugMode)
		{
			printf("cellsize %f %f\n", cv::norm(dst2[1] - dst2[0]), cell_size_x_);
		}
	}

	/**
	 * Utility function which gives the vertical distance in pixels from the current point
	 * until a white pixel is reached. Can be used on the timing pattern cells
	 * to check how well the cell size is tuned.
	 *
	 * @param image		Monochrome image
	 * @param x			X position of pixel
	 * @param yStart	Y position of pixel
	 * @return			Pair of top distance and bottom distance
	 */
	std::pair<int, int> verticalDistance(cv::Mat image, int x, int yStart)
	{
		int top	   = -1;
		int bottom = -1;
		int y;

		y = yStart;
		while (image.data[image.cols * y + x] < 127)
		{
			top++;
			y--;
		}

		y = yStart;
		while (image.data[image.cols * y + x] < 127)
		{
			bottom++;
			y++;
		}

		return std::make_pair(top, bottom);
	}

	/**
	 * Utility function which gives the horizontal distance in pixels from the current point
	 * until a white pixel is reached. Can be used on the timing pattern cells
	 * to check how well the cell size is tuned.
	 *
	 * @param image		Monochrome image
	 * @param y 		Y position of pixel
	 * @param xStart	X position of pixel
	 * @return			Pair of left distance and right distance
	 */
	std::pair<int, int> horizontalDistance(cv::Mat image, int xStart, int y)
	{
		int left  = -1;
		int right = -1;
		int x;

		x = xStart;
		while (image.data[image.cols * y + x] < 127)
		{
			left++;
			x--;
		}

		x = xStart;
		while (image.data[image.cols * y + x] < 127)
		{
			right++;
			x++;
		}

		return std::make_pair(left, right);
	}

	/**
	 * Must be called with a black cell which is surrounded by white cells in at least one
	 * direction. Is used for timing patterns.
	 *
	 * @param pixelX	X position of pixel
	 * @param pixelY	Y position of pixel
	 * @param vertical	If true, vertical orientation is used. If false, horizontal
	 */
	void optimizeCellsizeWithEnclosedCell(int pixelX, int pixelY, bool vertical)
	{
		bool val = monochrome_image_warped_.data[monochrome_image_warped_.cols * pixelY + pixelX] < 127;
		assert(val);

		if (vertical)
		{
			auto dist = verticalDistance(monochrome_image_warped_, pixelX, pixelY);
			if (debugMode)
				printf("Vertical distance Start %d %d\n", dist.first, dist.second);
			// (pixelY + diff - halfCellSize)/y =
			int diff	 = (dist.first - dist.second) / 2;
			float y		 = round((pixelY - half_cell_size_) / cell_size_y_);
			cell_size_y_ = (pixelY - diff - half_cell_size_) / y;
		}
		else
		{
			auto dist = horizontalDistance(monochrome_image_warped_, pixelX, pixelY);
			if (debugMode)
				printf("Horizontal Distance Start %d %d\n", dist.first, dist.second);
			int diff	 = (dist.first - dist.second) / 2;
			float x		 = round((pixelX - half_cell_size_) / cell_size_x_);
			cell_size_x_ = (pixelX - diff - half_cell_size_) / x;
		}
	}

	/**
	 * The cell size was previously approximated using the size of a finder pattern as every finder
	 * pattern is 7x7 cells in size.
	 * But this might not be 100% correct for the whole image.
	 * For this, every QrCode has the timing pattern to detect and correct this.
	 * We use the horizontal and vertical timing pattern to fine tune \ref cell_size_x_ and \ref cell_size_y_
	 */
	void optimizeCellsizeWithTimingPattern()
	{
		int pixelX;
		int pixelY;
		int cellX;
		int cellY;

		int timing_pattern_black_cells = (size_in_cells_ - 7 * 2) / 2;

		if (debugMode)
			printf("cellsize at start %f %f timing_pattern_black_cells %d\n", cell_size_x_, cell_size_y_,
				   timing_pattern_black_cells);

		for (int i = 0; i < timing_pattern_black_cells; i++)
		{
			cellX  = 6;
			cellY  = 8 + 2 * i;
			pixelX = round(half_cell_size_ + static_cast<float>(cellX) * cell_size_x_);
			pixelY = round(half_cell_size_ + static_cast<float>(cellY) * cell_size_y_);
			optimizeCellsizeWithEnclosedCell(pixelX, pixelY, true);

			pixelX = round(half_cell_size_ + static_cast<float>(cellX) * cell_size_x_);
			pixelY = round(half_cell_size_ + static_cast<float>(cellY) * cell_size_y_);

			auto dist = verticalDistance(monochrome_image_warped_, pixelX, pixelY);
			if (debugMode)
				printf("Vertical distance After %d %d\n", dist.first, dist.second);

			cellX  = 8 + 2 * i;
			cellY  = 6;
			pixelX = round(half_cell_size_ + static_cast<float>(cellX) * cell_size_x_);
			pixelY = round(half_cell_size_ + static_cast<float>(cellY) * cell_size_y_);
			optimizeCellsizeWithEnclosedCell(pixelX, pixelY, false);
		}

		if (alignment_pattern_centers_.size() > 0)
		{
			std::array<cv::Point2f, 1> x;
			x[0].x = alignment_pattern_centers_.at(0).x;
			x[0].y = alignment_pattern_centers_.at(0).y;
			std::array<cv::Point2f, 1> y;
			cv::perspectiveTransform(x, y, transform);
			optimizeCellsizeWithEnclosedCell(y[0].x, y[0].y, false);

			if (debugMode)
				printf("Alignment Center at %f %f\n", y[0].x, y[0].y);
		}

		if (debugMode)
			printf("cellsize at end %f %f\n", cell_size_x_, cell_size_y_);
	}

	/// Center of top left finder pattern
	cv::Point topLeftCenterRef;
	/// Center of top right finder pattern
	cv::Point topRightCenterRef;
	/// Center of bottom left finder pattern
	cv::Point bottomLeftCenterRef;
	/// Center of bottom right alignment pattern.
	/// If it doesn't exist it still defines the position where it would be assumed
	cv::Point bottomRightCenterRef;

	/// Calibrated nudge value for \ref topLeftCenterRef
	cv::Point topLeftNudge;
	/// Calibrated nudge value for \ref topRightCenterRef
	cv::Point topRightNudge;
	/// Calibrated nudge value for \ref bottomLeftCenterRef
	cv::Point bottomLeftNudge;
	/// Calibrated nudge value for \ref bottomRightCenterRef
	cv::Point bottomRightNudge;

	/**
	 * Normally a code should be flat. But this might not always be the case with deformed pieces of paper.
	 * We use the position and shape of the finder patterns and the alignment pattern as reference points
	 * to calculate an error value which we call the nudge here.
	 * Using bilinear filtering, those 4 nudge values can be used on the cell positions for better results,
	 */
	void calibrateCellNudge()
	{
		int pixelX;
		int pixelY;
		int cellX;
		int cellY;
		std::pair<int, int> vdist;
		std::pair<int, int> hdist;

		cellX  = 3;
		cellY  = 3;
		pixelX = round(half_cell_size_ + static_cast<float>(cellX) * cell_size_x_);
		pixelY = round(half_cell_size_ + static_cast<float>(cellY) * cell_size_y_);
		vdist  = verticalDistance(monochrome_image_warped_, pixelX, pixelY);
		hdist  = horizontalDistance(monochrome_image_warped_, pixelX, pixelY);
		cv::circle(monochrome_image_warped_, cv::Point(pixelX, pixelY), 6, white);

		topLeftCenterRef.x = pixelX;
		topLeftCenterRef.y = pixelY;
		topLeftNudge.x	   = -(hdist.first - hdist.second) / 2;
		topLeftNudge.y	   = -(vdist.first - vdist.second) / 2;

		cv::circle(monochrome_image_warped_, cv::Point(pixelX, pixelY) + topLeftNudge, 6, white);

		// Top right
		cellX = size_in_cells_ - 4;
		cellY = 3;

		pixelX = round(half_cell_size_ + static_cast<float>(cellX) * cell_size_x_);
		pixelY = round(half_cell_size_ + static_cast<float>(cellY) * cell_size_y_);
		vdist  = verticalDistance(monochrome_image_warped_, pixelX, pixelY);
		hdist  = horizontalDistance(monochrome_image_warped_, pixelX, pixelY);
		cv::circle(monochrome_image_warped_, cv::Point(pixelX, pixelY), 6, white);

		topRightCenterRef.x = pixelX;
		topRightCenterRef.y = pixelY;
		topRightNudge.x		= -(hdist.first - hdist.second) / 2;
		topRightNudge.y		= -(vdist.first - vdist.second) / 2;

		cv::circle(monochrome_image_warped_, cv::Point(pixelX, pixelY) + topRightNudge, 6, white);

		// Bottom left
		cellX = 3;
		cellY = size_in_cells_ - 4;

		pixelX = round(half_cell_size_ + static_cast<float>(cellX) * cell_size_x_);
		pixelY = round(half_cell_size_ + static_cast<float>(cellY) * cell_size_y_);
		vdist  = verticalDistance(monochrome_image_warped_, pixelX, pixelY);
		hdist  = horizontalDistance(monochrome_image_warped_, pixelX, pixelY);
		cv::circle(monochrome_image_warped_, cv::Point(pixelX, pixelY), 6, white);

		bottomLeftCenterRef.x = pixelX;
		bottomLeftCenterRef.y = pixelY;
		bottomLeftNudge.x	  = -(hdist.first - hdist.second) / 2;
		bottomLeftNudge.y	  = -(vdist.first - vdist.second) / 2;

		cv::circle(monochrome_image_warped_, cv::Point(pixelX, pixelY) + bottomLeftNudge, 6, white);

		// Alignment Pattern - Bottom Right
		cellX = size_in_cells_ - 7;
		cellY = size_in_cells_ - 7;

		pixelX = round(half_cell_size_ + static_cast<float>(cellX) * cell_size_x_);
		pixelY = round(half_cell_size_ + static_cast<float>(cellY) * cell_size_y_);

		bottomRightCenterRef.x = pixelX;
		bottomRightCenterRef.y = pixelY;

		if (alignment_pattern_centers_.size() > 0)
		{
			vdist = verticalDistance(monochrome_image_warped_, pixelX, pixelY);
			hdist = horizontalDistance(monochrome_image_warped_, pixelX, pixelY);
			cv::circle(monochrome_image_warped_, cv::Point(pixelX, pixelY), 6, white);

			bottomRightNudge.x = -(hdist.first - hdist.second) / 2;
			bottomRightNudge.y = -(vdist.first - vdist.second) / 2;

			cv::circle(monochrome_image_warped_, cv::Point(pixelX, pixelY) + bottomRightNudge, 6, white);
		}
		else
		{
			bottomRightNudge.x = (topRightNudge.x + bottomLeftNudge.x) / 2;
			bottomRightNudge.y = (topRightNudge.y + bottomLeftNudge.y) / 2;
		}
	}

	/**
	 * Performs linear interpolation
	 *
	 * @param x1	Left ref point
	 * @param f_x1	Value at left ref point
	 * @param x2	Right ref point
	 * @param f_x2	Value at right ref point
	 * @param x		Position between Left and right ref point
	 * @return		Linear interpolated value
	 */
	float linearInterpolation(float x1, float f_x1, float x2, float f_x2, float x)
	{
		float result = (x - x1) / (x2 - x1) * f_x2 + (x2 - x) / (x2 - x1) * f_x1;
		return result;
	}

	/**
	 * Get pixel position for a given cell coordinate.
	 * @param x 			Horizontal cell coordinate
	 * @param y				Vertical cell coordinate
	 * @param fineNudge		Use nudge system to alter the resulting position
	 * @return				Pixel position in \ref monochrome_image_warped_
	 */
	cv::Point getCellPosition(int x, int y, bool fineNudge)
	{
		// Coarse
		cv::Point point;
		cv::Point nudge(0, 0);
		point.x = round(half_cell_size_ + static_cast<float>(x) * cell_size_x_);
		point.y = round(half_cell_size_ + static_cast<float>(y) * cell_size_y_);

		if (fineNudge)
		{
			// Fine nudging
			float topHorizontalX =
				linearInterpolation(topLeftCenterRef.x, topLeftNudge.x, topRightCenterRef.x, topRightNudge.x, point.x);
			float bottomHorizontalX = linearInterpolation(bottomLeftCenterRef.x, bottomLeftNudge.x,
														  bottomRightCenterRef.x, bottomRightNudge.x, point.x);

			nudge.x = linearInterpolation(bottomLeftCenterRef.y, bottomHorizontalX, topRightCenterRef.y, topHorizontalX,
										  point.y);

			float topHorizontalY =
				linearInterpolation(topLeftCenterRef.x, topLeftNudge.y, topRightCenterRef.x, topRightNudge.y, point.x);
			float bottomHorizontalY = linearInterpolation(bottomLeftCenterRef.x, bottomLeftNudge.y,
														  bottomRightCenterRef.x, bottomRightNudge.y, point.x);
			nudge.y = linearInterpolation(bottomLeftCenterRef.y, bottomHorizontalY, topRightCenterRef.y, topHorizontalY,
										  point.y);
		}
		return point + nudge;
	}

	/**
	 * Extract a binary representation of the code from \ref monochrome_image_warped_
	 */
	void extractCells()
	{
		for (int y = 0; y < size_in_cells_; y++)
		{
			cells.push_back(std::vector<bool>(size_in_cells_));
			debug_cells.push_back(std::vector<char>(size_in_cells_));

			for (int x = 0; x < size_in_cells_; x++)
			{
				auto p			  = getCellPosition(x, y, true);
				auto pNoNudge	  = getCellPosition(x, y, false);
				cells.at(y).at(x) = monochrome_image_warped_.data[monochrome_image_warped_.cols * p.y + p.x] < 127;
				debug_cells.at(y).at(x) = '-';

				cv::circle(monochrome_image_warped_, p, 3, cells.at(y).at(x) ? white : black);
				cv::circle(monochrome_image_warped_, pNoNudge, 1, cells.at(y).at(x) ? white : black);
			}
		}
	}

	/**
	 * The finder pattern on the top left contains one of two format areas.
	 * Both are equal but in this case we ignore the format pattern on the other finder patterns.
	 * Reads the cells and performs a hamming distance analysis to get the best fit.
	 * Throws an exception if no format could be found.
	 * Capable of detecting the format, even so the format are was damaged.
	 */
	void extractFormat()
	{
		// from http://www.cdsy.de/formatbereiche.html
		std::array<struct FormatCode, 4 * 8> formats
			//
			{{{kErrL, 0, 0b111011111000100}, //
			  {kErrL, 1, 0b111001011110011}, //
			  {kErrL, 2, 0b111110110101010}, //
			  {kErrL, 3, 0b111100010011101}, //
			  {kErrL, 4, 0b110011000101111}, //
			  {kErrL, 5, 0b110001100011000}, //
			  {kErrL, 6, 0b110110001000001}, //
			  {kErrL, 7, 0b110100101110110}, //

			  {kErrM, 0, 0b101010000010010}, //
			  {kErrM, 1, 0b101000100100101}, //
			  {kErrM, 2, 0b101111001111100}, //
			  {kErrM, 3, 0b101101101001011}, //
			  {kErrM, 4, 0b100010111111001}, //
			  {kErrM, 5, 0b100000011001110}, //
			  {kErrM, 6, 0b100111110010111}, //
			  {kErrM, 7, 0b100101010100000}, //

			  {kErrQ, 0, 0b011010101011111}, //
			  {kErrQ, 1, 0b011000001101000}, //
			  {kErrQ, 2, 0b011111100110001}, //
			  {kErrQ, 3, 0b011101000000110}, //
			  {kErrQ, 4, 0b010010010110100}, //
			  {kErrQ, 5, 0b010000110000011}, //
			  {kErrQ, 6, 0b010111011011010}, //
			  {kErrQ, 7, 0b010101111101101}, //

			  {kErrH, 0, 0b001011010001001}, //
			  {kErrH, 1, 0b001001110111110}, //
			  {kErrH, 2, 0b001110011100111}, //
			  {kErrH, 3, 0b001100111010000}, //
			  {kErrH, 4, 0b000011101100010}, //
			  {kErrH, 5, 0b000001001010101}, //
			  {kErrH, 6, 0b000110100001100}, //
			  {kErrH, 7, 0b000100000111011}}};

		// std::cout << formats.begin()->code << std::endl;
		// auto formats = make_array<struct possibleFormats>({{err_L, 0, 0b111011111000100}});

		// X,Y
		std::array<std::array<int, 2>, 15> format_pos{{
			{0, 8}, // 1
			{1, 8}, // 2
			{2, 8}, // 3
			{3, 8}, // 4

			{4, 8}, // 5
			{5, 8}, // 6
			// Pause
			{7, 8}, // 7

			{8, 8}, // 8
			{8, 7}, // 9
			// Pause
			{8, 5}, // 10
			{8, 4}, // 11

			{8, 3}, // 12
			{8, 2}, // 13
			{8, 1}, // 14
			{8, 0}, // 15

		}};

		int formatVal = 0;

		for (auto& pos : format_pos)
		{
			formatVal <<= 1;

			if (cells.at(pos[1]).at(pos[0]))
				formatVal |= 1;
		}

		if (debugMode)
		{
			bin_prnt_byte(formatVal);
		}

		int minimalDist		= 16;
		int suggestedFormat = -1;

		for (int i = 0; i < formats.size(); i++)
		{
			auto& f = formats.at(i);

			int distance = hammingDistance(f.code, formatVal);
			if (distance < minimalDist)
			{
				suggestedFormat		  = i;
				detected_format_code_ = f;
				minimalDist			  = distance;
			}
			if (debugMode)
			{
				printf("hamming distance %d %d %d\n", f.err, f.mask, distance);
			}
		}

		if (minimalDist > 3)
		{
			throw DecoderException(DecoderException::Cause::kFormatNotDetected);
		}

		if (debugMode)
		{
			printf("detected mask %d, detected correction level %s\n", detected_format_code_.mask,
				   correctionLevelStr[detected_format_code_.err]);
		}
	}

	/**
	 * Utility function to check if a cell is part of the timing pattern
	 * @param x 	Horizontal cell coordinate
	 * @param y		Vertical cell coordinate
	 * @return		True, if part of timing pattern
	 */
	bool isCellOnTimingPattern(int x, int y)
	{
		return ((x == 6) || (y == 6));
	}

	/**
	 * Utility function to check if a cell is part of actual data
	 * @param x 	Horizontal cell coordinate
	 * @param y		Vertical cell coordinate
	 * @return		True, if cell which represents data
	 */
	bool isCellDataBit(int x, int y)
	{
		// Top right finder pattern with only one format area at the bottom
		if (x > size_in_cells_ - 9 && y < 9)
			return false;

		// Top left finder pattern with two format areas
		if (x < 9 && y < 9)
			return false;

		// Bottom left finder pattern
		if (x < 9 && y > size_in_cells_ - 9)
			return false;

		// Timing patterns
		if ((x == 6) || (y == 6))
			return false;

		if (detected_qr_config_.version > 1)
		{
			// Alignment pattern
			if (x >= size_in_cells_ - 9 && x < size_in_cells_ - 4 && y >= size_in_cells_ - 9 && y < size_in_cells_ - 4)
				return false;
		}

		return true;
	}

	/**
	 * With each call, a cell is extracted from the code and returned.
	 *
	 * @return	True, if 1, if black
	 */
	bool readBitFromCells()
	{
		int& x = current_cell_position_.x;
		int& y = current_cell_position_.y;

		bool gotDataBit = false;
		bool result;

		while (!gotDataBit)
		{
			if (isCellDataBit(x, y))
			{
				result = cells.at(y).at(x);
				// printf("Reading from %d %d  %d\n", x, y, result ? 1 : 0);
				debug_cells.at(y).at(x) = debug_cell_val_;
				gotDataBit				= true;
			}
			else
			{
				// Do nothing
				// printf("Ignore from %d %d  %d\n", x, y, result ? 1 : 0);
			}

			if (change_row_)
			{
				if (going_up_)
				{
					if (y == 0)
					{
						x -= 2;
						going_up_ = false;
						// printf("Going down!\n");

						if (isCellOnTimingPattern(x + 1, y))
						{
							// printf("Nudge left!");
							x--;
						}
					}
					else
						y--;
				}
				else
				{
					if (y == size_in_cells_ - 1)
					{
						x -= 2;
						going_up_ = true;
						// printf("Going up!\n");

						if (isCellOnTimingPattern(x + 1, y))
						{
							// printf("Nudge left!");
							x--;
						}
					}
					else
						y++;
				}

				x++;
			}
			else
			{
				x--;
			}

			change_row_ = !change_row_;
		}

		return result;
	}

	/**
	 * Reads a number of cells and pack them together as a word.
	 *
	 * @param bitlen	Number of cells to read
	 * @return			The resulting word
	 */
	int readWordFromCells(int bitlen)
	{
		int result = 0;
		while (bitlen--)
		{
			result <<= 1;
			if (readBitFromCells())
				result |= 1;
		}

		// printf("%d %x %c ", result, result, result);
		// bin_prnt_byte(result);
		return result;
	}

	/**
	 * The Qr code uses interleaving of the data for bigger sizes. The data is split in multiple blocks, which
	 * itself are part of a maximum of 2 groups.
	 * Each group has a separate configuration for the number of payload and error correction bytes.
	 * For 2 groups with 2 data blocks, the interleaving is like so
	 *
	 * G1B1[0] G1B2[0] G2B1[0] G2B2[0] G1B1[1] G1B2[1] G2B1[1] G2B2[1] ...
	 * after the data, the error correction bytes follow with the same interleaving order.
	 *
	 * This function builds an easy to use lookup table for the next functions which are actualy
	 * require the data in the correct order.
	 */
	void buildDbDeinterleavingLut()
	{
		db_blocks_	  = detected_qr_config_.blocksInGroup1 + detected_qr_config_.blocksInGroup2;
		number_of_Db_ = detected_qr_config_.blocksInGroup1 * detected_qr_config_.dbPerBlockInGroup1 +
						detected_qr_config_.blocksInGroup2 * detected_qr_config_.dbPerBlockInGroup2;

		db_deinterleaving_lut_.clear();
		for (int i = 0; i < detected_qr_config_.blocksInGroup1; i++)
		{
			db_deinterleaving_lut_.emplace_back(std::vector<int>());
		}
		for (int i = 0; i < detected_qr_config_.blocksInGroup2; i++)
		{
			db_deinterleaving_lut_.emplace_back(std::vector<int>());
		}

		int maxSizeOfDbBlock = std::max(detected_qr_config_.dbPerBlockInGroup1, detected_qr_config_.dbPerBlockInGroup2);

		int dbOffset  = 0;
		int dbToRead  = detected_qr_config_.dbPerBlockInGroup1 + detected_qr_config_.dbPerBlockInGroup2;
		int dbOffset2 = 0;
		while (maxSizeOfDbBlock > 0)
		{
			maxSizeOfDbBlock--;
			for (int j = 0; j < detected_qr_config_.blocksInGroup1; j++)
			{
				if (dbOffset2 < detected_qr_config_.dbPerBlockInGroup1)
				{
					db_deinterleaving_lut_.at(j).push_back(dbOffset);
					dbOffset++;
				}
			}

			for (int j = 0; j < detected_qr_config_.blocksInGroup2; j++)
			{
				if (dbOffset2 < detected_qr_config_.dbPerBlockInGroup2)
				{
					db_deinterleaving_lut_.at(detected_qr_config_.blocksInGroup1 + j).push_back(dbOffset);
					dbOffset++;
				}
			}
			dbOffset2++;
		}

		if (debugMode)
		{
			printf("Interleaving Lut ---- %d\n", db_deinterleaving_lut_.size());
			for (auto& b : db_deinterleaving_lut_)
			{
				for (auto& offset : b)
				{
					printf("%4d ", offset);
				}
				printf("\n");
			}
			printf("-------------\n");
		}
		assert(maxSizeOfDbBlock == 0);
	}

	/**
	 * With each call a ASCII character is extracted from the bitstream.
	 * Supports multiple encodings of the QrCode.
	 * Numeric, Alphanumeric and Byte mode are supported.
	 *
	 * @param bitstream	Raw bitstream, generated from the raw data of the QrCode. Must be deinterleaved
	 * @return			ASCII character.
	 */
	std::vector<char> extractPayload(BitStreamReader& bitstream)
	{
		bool reading = true;
		std::vector<char> payload_data;
		while (reading)
		{
			int mode = bitstream.readWord(4);
			if (debugMode)
				printf("Mode: %d\n", mode);

			switch (mode)
			{
			case 0: // Terminator
				reading = false;
				break;
			case 1: // Numeric
			{
				int payload_len = bitstream.readWord(10);
				if (debugMode)
					printf("Numeric Payload Len: %d\n", payload_len);
				printf("Payload: ");
				while (payload_len > 0)
				{
					uint32_t data;

					switch (payload_len)
					{
					case 2:
						data = bitstream.readWord(7);
						payload_len -= 2;
						printf("%02d", data);

						payload_data.push_back('0' + data / 10);
						payload_data.push_back('0' + data % 10);
						break;
					case 1:
						data = bitstream.readWord(4);
						payload_len -= 1;
						printf("%d", data);
						payload_data.push_back('0' + data);
						break;
					default:
						data = bitstream.readWord(10);
						payload_len -= 3;
						printf("%03d", data);
						payload_data.push_back('0' + data / 100);
						payload_data.push_back('0' + (data / 10) % 10);
						payload_data.push_back('0' + data % 10);

						break;
					}
				}
				printf("\n");
				break;
			}
			case 2: // Alphanumeric
			{
				int payload_len = bitstream.readWord(9);
				if (debugMode)
					printf("Alphanumeric Payload Len: %d\n", payload_len);
				printf("Payload: ");
				while (payload_len > 0)
				{
					uint32_t data;
					if (payload_len > 1)
					{
						data = bitstream.readWord(11);
						payload_len -= 2;

						char translated[2] = {alphanumeric_lut.at(data / 45), alphanumeric_lut.at(data % 45)};
						printf("%c%c", translated[0], translated[1]);
						payload_data.push_back(translated[0]);
						payload_data.push_back(translated[1]);
					}
					else
					{
						data = bitstream.readWord(6);
						payload_len--;
						char translated = alphanumeric_lut.at(data);
						payload_data.push_back(translated);
						printf("%c", translated);
					}
				}
				printf("\n");
				break;
			}
			case 4: // Byte mode
			{
				int payload_len = bitstream.readWord(8);

				if (debugMode)
					printf("Byte Mode Payload Len: %d\n", payload_len);
				printf("Payload: ");
				while (payload_len--)
				{
					uint8_t data = bitstream.readWord(8);
					payload_data.push_back(data);
					printf("%c", data);
					debug_cell_val_++;
				}
				printf("\n");
				break;
			}
			default:
				assert(0);
				break;
			}
		}

		return payload_data;
	}

	/**
	 * Uses the previously build deinterleaving lut and performs Reed Solomon decoding to
	 * check the integrity. Also corrects damaged data.
	 * @param rawData	Raw data of QrCode
	 * @return			Original or repaired raw data of QrCode
	 */
	std::vector<uint8_t> deinterleaveAndForwardErrorCorrect(std::vector<uint8_t> rawData)
	{
		std::vector<uint8_t> raw_decoded_data;

		for (int block = 0; block < db_blocks_; block++)
		{
			std::vector<FFieldQr> rsData;

			int k = db_deinterleaving_lut_.at(block).size();
			int t = detected_qr_config_.ecbPerBlock;
			int n = k + t;

			int dbstart	 = block;
			int eccstart = number_of_Db_ + block;
			int offset	 = dbstart;

			for (int i = 0; i < k; i++)
			{
				rsData.push_back(rawData.at(db_deinterleaving_lut_.at(block).at(i)));
			}

			for (int i = 0; i < t; i++)
			{
				rsData.push_back(rawData.at(eccstart + i * db_blocks_));
			}

			if (debugMode)
			{
				printf("In:  ");
				for (auto& v : rsData)
				{
					printf("%02x ", v);
				}
				printf("\n");
			}

			RS::ReedSolomon<FFieldQr> rs(n, k);
			std::reverse(rsData.begin(), rsData.end());

			auto rsPolynomial		= Polynom<FFieldQr>(rsData);
			auto syndromes			= rs.calculateSyndromes(rsPolynomial);
			bool allSyndromesZero	= syndromes.second;
			code_needed_correction_ = !allSyndromesZero;

			if (debugMode)
			{
				for (auto& s : syndromes.first)
				{
					std::cout << "Syndrome " << s << std::endl;
				}
			}
			auto decoded_message = rs.decode(rsPolynomial);

			if (debugMode)
			{
				std::cout << decoded_message.asString() << std::endl;
			}

			auto raw_decoded_data_extfield = decoded_message.getRawData();

			std::reverse(raw_decoded_data_extfield.begin(), raw_decoded_data_extfield.end());

			if (debugMode)
			{
				printf("Out: ");
				for (auto& v : raw_decoded_data_extfield)
				{
					printf("%02x ", v);
				}
				printf("\n");
			}
			raw_decoded_data_extfield.resize(k);

			for (auto it = std::begin(raw_decoded_data_extfield); it != std::end(raw_decoded_data_extfield); ++it)
			{
				raw_decoded_data.push_back(*it);
			}
		}

		if (debugMode)
		{
			printf("size : %d\n", raw_decoded_data.size());
			for (auto& v : raw_decoded_data)
			{
				printf("%d ", v);
			}
			printf("\n");
		}

		return raw_decoded_data;
	}

  public:
	/**
	 * Lookup table for the alphanumeric encoding mode
	 */
	const std::array<char, 45> alphanumeric_lut = {
		'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E',
		'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',
		'U', 'V', 'W', 'X', 'Y', 'Z', ' ', '$', '%', '*', '+', '-', '.', '/', ':',
	};

	/// If true, the class is more verbose
	bool debugMode{false};

	/// If true, the image data is displayed with label data for debugging
	bool debugModeVisual{false};

	/// Set internally to true, if the Reed Solomon decoder had to correct damaged cells
	bool code_needed_correction_{false};

	/**
	 * Reads an image from a file and tries to find and decode an QrCode
	 * Throws an exception if the decoding was not successful.
	 * @param filepath	Path to file.
	 * @return			ASCII Payload data of code
	 */
	std::vector<char> decodeFromFile(const char* filepath)
	{
		reset();

		if (debugMode)
		{
			printf("Reading %s\n", filepath);
		}

		src_image_ = cv::imread(filepath, 1);
		if (!src_image_.data)
		{
			printf("No image data \n");
			throw DecoderException(DecoderException::Cause::kFileNotReadable);
		}

		cv::Mat src_image_as_grayscale, grayscale_blurred;
		cv::Mat monochrome_image_inverted;
		cv::Mat monochrome_image_normal;

		// Convert image to grayscale
		cv::cvtColor(src_image_, src_image_as_grayscale, cv::COLOR_BGR2GRAY);

		// Blur the grayscale image to filter out small artifacts
		if (src_image_as_grayscale.cols < 500 || src_image_as_grayscale.rows < 500)
			cv::resize(src_image_as_grayscale, grayscale_blurred, cv::Size(0, 0), 3, 3);
		else
			cv::GaussianBlur(src_image_as_grayscale, grayscale_blurred, cv::Size(11, 11), 0, 0);

		// Convert grayscale to monochrome image
		cv::adaptiveThreshold(grayscale_blurred, monochrome_image_inverted, 255, cv::ADAPTIVE_THRESH_MEAN_C,
							  cv::THRESH_BINARY_INV, 131, 0);

		// Find Contours in the monochrome image
		cv::findContours(monochrome_image_inverted, contours_, hierarchy_, cv::RETR_TREE, cv::CHAIN_APPROX_TC89_KCOS);

		monochrome_image_inverted_area = monochrome_image_inverted.cols * monochrome_image_inverted.rows;

		// Try to find the finder patterns in the contour data
		searchFinderPatternPoints(0, 0);

		if (finder_contour_points_.size() != 3)
		{
			printf("No 3 finders\n");

			if (debugModeVisual)
			{
				cv::imshow("src_image_as_grayscale", src_image_as_grayscale);
				cv::imshow("grayscale_blurred", grayscale_blurred);
				cv::imshow("monochrome_image_inverted", monochrome_image_inverted);
				cv::imshow("src_image_", src_image_);

				cv::waitKey(0);
			}

			throw DecoderException(DecoderException::Cause::kNotEnoughFinderPattern);
		}

		// Find the average of all finder pattern points to check where the center of the code might be
		assumeFinderPatternCenter();

		if (debugModeVisual)
		{
			cv::imshow("src_image_as_grayscale", src_image_as_grayscale);
			cv::imshow("grayscale_blurred", grayscale_blurred);
			cv::imshow("monochrome_image_inverted", monochrome_image_inverted);
			cv::imshow("src_image_", src_image_);

			cv::waitKey(0);
		}

		reconstructShapeOfCode();

		cv::bitwise_not(monochrome_image_inverted, monochrome_image_normal);

		cv::warpPerspective(monochrome_image_normal, monochrome_image_warped_, transform,
							cv::Size(warped_size, warped_size));

		if (debugModeVisual)
		{
			cv::imshow("src_image_as_grayscale", src_image_as_grayscale);
			cv::imshow("grayscale_blurred", grayscale_blurred);
			cv::imshow("monochrome_image_inverted", monochrome_image_inverted);
			cv::imshow("src_image_", src_image_);

			cv::waitKey(0);
		}

		float cellsizeAvg  = (cell_size_x_ + cell_size_y_) / 2;
		float sizeInCellsF = static_cast<float>(warped_size) / cellsizeAvg;

		size_in_cells_ = round(sizeInCellsF);

		if (debugMode)
			printf("sizeInCells %d %f\n", size_in_cells_, sizeInCellsF);

		optimizeCellsizeWithTimingPattern();

		calibrateCellNudge();

		extractCells();

		if (debugModeVisual)
		{
			cv::imshow("src_image_as_grayscale", src_image_as_grayscale);
			cv::imshow("grayscale_blurred", grayscale_blurred);
			cv::imshow("monochrome_image_inverted", monochrome_image_inverted);
			cv::imshow("src_image_", src_image_);
			cv::imshow("monochrome_image_warped", monochrome_image_warped_);
			cv::imwrite("monochrome_image_warped.png", monochrome_image_warped_);

			cv::waitKey(0);
		}

		extractFormat();

		grabQrConfig();

		if (debugMode)
		{
			printf("--- Original Cells ------\n");
			for (const auto& c : cells)
			{
				for (const auto& p : c)
				{
					printf("%d", p ? 1 : 0);
				}
				printf("\n");
			}
		}

		applyMask(detected_format_code_.mask);

		if (debugMode)
		{
			printf("---- Mask applied -------\n");
			for (const auto& c : cells)
			{
				for (const auto& p : c)
				{
					printf("%d", p ? 1 : 0);
				}
				printf("\n");
			}
		}

		current_cell_position_.x = size_in_cells_ - 1;
		current_cell_position_.y = size_in_cells_ - 1;

		std::vector<uint8_t> rawData;
		int raw_len_to_read = raw_db_plus_ecb;
		debug_cell_val_		= 'A';

		while (raw_len_to_read--)
		{
			int data = readWordFromCells(8);
			rawData.push_back(data);

			if (debug_cell_val_ == 'Z')
				debug_cell_val_ = 'a';
			else
				debug_cell_val_++;
		}

		if (debugMode)
		{
			for (const auto& c : debug_cells)
			{
				for (const auto& p : c)
				{
					printf("%c", p);
				}
				printf("\n");
			}

			printf("size : %d\n", rawData.size());
			for (auto& v : rawData)
			{
				printf("%d ", v);
			}
			printf("\n");
		}

		buildDbDeinterleavingLut();

		std::vector<uint8_t> raw_decoded_data = deinterleaveAndForwardErrorCorrect(rawData);

		BitStreamReader bitstream(raw_decoded_data);

		auto payload_data = extractPayload(bitstream);

		return payload_data;
	}
};

#endif /* QRDECODER_H_ */
