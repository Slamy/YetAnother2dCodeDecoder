/**
 * @file DataMatrixDecoder.h
 *
 *  Created on: 06.11.2021
 *      Author: andre
 */

#ifndef SRC_DATAMATRIXDECODER_H_
#define SRC_DATAMATRIXDECODER_H_

#include "ExtFiniteField256.h"
#include "Matrix.h"
#include "ReedSolomon.h"
#include "glm/gtx/vector_angle.hpp"
#include "glm/vec2.hpp"
#include "util.h"
#include <decoderexception.h>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <span>

/**
 * Data Matrix Decoder
 *
 * Doesn't supports many sizes. Just for academic purposes an intended for understandability
 *
 * To construct this class, multiple sources were used:
 * https://en.wikipedia.org/wiki/Data_Matrix
 * https://www.pepperl-fuchs.com/germany/de/6404.htm
 * https://www.barcodefaq.com/2d/data-matrix/
 * https://www.fobalaser.com/blog/facts-about-data-matrix-codes-for-product-identification-part-1/
 * https://barcode-coder.com/en/datamatrix-specification-104.html
 * https://sudonull.com/post/104600-How-is-Data-Matrix-created
 *
 */
class DataMatrixDecoder
{
  private:
	/// Prepare the object for another code to read.
	void reset()
	{
		contours.clear();
		cells.clear();
		debug_cells.clear();
		timing_pattern_top.clear();
		timing_pattern_right.clear();
		goUpRight	  = true;
		text_mode_set = TextMode::Text_Set_0;
		payload_data.clear();
	}

	/// Used by \ref readByteFromCells to keep track of the current cell in decoding process.
	cv::Point current_cell_position_;

	/// Only used for debugging to keep track of incrementing debug characters.
	/// Used together with \ref debug_cells
	char debugCellChar = 'A';

	/// To get the next positon of a code byte, we require the current position and the direction.
	/// With Data matrix the bytes are scanned in a diagonal pattern
	bool goUpRight{true};

	/// Vector of rows of with true being a black cell
	std::vector<std::vector<bool>> cells;

	/// Vector of rows of printable characters. Only used for development and debugging to check
	/// which portion of the code belongs to which data byte
	std::vector<std::vector<char>> debug_cells;

	/// Width and Height of the code in "cells"
	int size_in_cells_;

	/// Possible states of the Text mode. Used for selecting the character set
	enum class TextMode
	{
		C40_Set_0,
		C40_Set_1,
		C40_Set_2,
		C40_Set_3,

		Text_Set_0,
		Text_Set_1,
		Text_Set_2,
		Text_Set_3,
	};

	/// Currently active text mode set
	TextMode text_mode_set = TextMode::C40_Set_0;

	/// Payload data is appended to this vector until the whole code is decoded.
	/// Printable ASCII data
	std::vector<char> payload_data;

	/// Width and Height of perspective corrected image
	static constexpr int warped_size = 1000;

	/// OpenCV Image which was fed to this decoder.
	cv::Mat src_image_;

	/// Transformation matrix which is used rectify the code to full size
	cv::Mat transform;

	/// OpenCV Image of the perspective corrected code, after the shape was reconstructed.
	cv::Mat monochrome_image_warped_;

	/// Vector graphic representation of the code
	/// Used to find the code in the image
	std::vector<std::vector<cv::Point>> contours;

	/**
	 * Utility function to calculate the difference of two integer numbers in percent.
	 * @param a		first number
	 * @param b		second number
	 * @return		difference in percent
	 */
	int percentageDiff(int a, int b)
	{
		return labs(a - b) * 100 / ((a + b) / 2);
	}

	/// Position of the corner connected to the straight edges
	cv::Point finder_bottom_left_;

	/// Positon of the top left corner, connected to the left straight edge and the top timing pattern
	cv::Point finder_top_left_;

	/// Positon of the bottom right corner, connected to the bottom straight edge and the right timing pattern
	cv::Point finder_bottom_right_;

	/// Positions of corners of the top edge of the top timing pattern
	/// Starting from top left, all thes points are expected to form a straight line,
	/// made from points with equal distance to each other.
	/// Suitable for extracting the size of a single cell.
	std::vector<cv::Point> timing_pattern_top;

	/// Same as \ref timing_pattern_top but fir the right timing pattern
	/// starting from bottom right.
	std::vector<cv::Point> timing_pattern_right;

	/// Width of a Cell, a cell being a "pixel" of the code
	float cell_size_x_;
	/// Height of a Cell, a cell being a "pixel" of the code
	float cell_size_y_;

	/// Used as starting offset to find the center of the to left cell
	float half_cell_size_;

	/**
	 * The orientation of bits in data matrix is arranged as something
	 * like a mirrored L shape.
	 *
	 * Visual representation:
	 * 01
	 * 234
	 * 567
	 *
	 * This array provides coordinate offsets, relative to the bottom right bit.
	 * It seems that the bottom right bit is also the base position, when it comes
	 * to create the layout of bytes in the code.
	 */
	const std::array<cv::Point, 8> cell_positions_{{
		{-2, -2}, //
		{-1, -2}, //
		{-2, -1}, //
		{-1, -1}, //
		{0, -1},  //
		{-2, 0},  //
		{-1, 0},  //
		{0, 0},	  //
	}};

	/**
	 * Special case of a shape in which data is aligned.
	 * Sometimes it's not possible to fill the code with the shape described with \ref cell_positions_
	 */
	const std::array<cv::Point, 8> cornerCase1{{
		{1, 0},	 // at bottom left
		{2, 0},	 // at bottom left
		{3, 0},	 // at bottom left
		{-1, 1}, // at top right
		{0, 1},	 //
		{0, 2},	 //
		{0, 3},	 //
		{0, 4},	 //
	}};

	/**
	 * Special case of a shape in which data is aligned.
	 * Sometimes it's not possible to fill the code with the shape described with \ref cell_positions_
	 */
	const std::array<cv::Point, 8> cornerCase2{{
		{1, -2}, // at bottom left
		{1, -1}, // at bottom left
		{1, 0},	 // at bottom left
		{-3, 1}, // at top right
		{-2, 1}, //
		{-1, 1}, //
		{0, 1},	 //
		{0, 2},	 //
	}};

	/**
	 * Possible configuration for a size of code
	 */
	struct DmConfig
	{
		int size; ///< width and height
		/// Code words are wrapped around the border to fill in the gaps. This is the offset in cells to
		/// shift the wrapped bits to the bottom and to the right.
		int overlapoffset;
		std::span<const cv::Point> cornercase; ///< used corner case for this size
		int cornerCaseId;					   ///< number of corner case or 0, in case no corner case exist
		int db;								   ///< number of data bytes with payload data
		int ecb;							   ///< number of error correction bytes
	};

	/// The config of the code which we are about to decode
	struct DmConfig detected_dm_config_;

	/**
	 * Taken from https://barcode-coder.com/en/datamatrix-specification-104.html
	 * TODO still quite some possible configurations missing
	 */
	const std::array<DmConfig, 8> configs_{{
		{10, 0, std::span<const cv::Point>(), 0, 3, 5},
		{12, -2, std::span<const cv::Point>(), 0, 5, 7},
		{14, 4, cornerCase1, 1, 8, 10},
		{16, 2, cornerCase2, 2, 12, 12},
		{18, 0, std::span<const cv::Point>(), 0, 18, 14},
		{20, -2, std::span<const cv::Point>(), 0, 22, 18},
		{22, 4, cornerCase1, 1, 30, 20},	  //
		{24, 2, cornerCase2, 2, 36, 60 - 36}, //
	}};

	/**
	 * The timing pattern is represented by points on the outer corners of the timing pattern made of
	 * alternating black and white cells. This function takes the current timing pattern and tries to
	 * find a possible next point to further refine the angle of the top and right edge of the data matrix
	 * code
	 *
	 * @param timing_pattern	Either the top of right timing pattern point data
	 */
	void continueTimingPattern(std::vector<cv::Point>& timing_pattern)
	{
		float smallestAngle = 100;
		cv::Point smallestAnglePoint;

		auto current = timing_pattern.at(timing_pattern.size() - 1);
		auto start	 = timing_pattern.at(timing_pattern.size() - 2);
		glm::vec2 currentStopPos(current.x, current.y);
		glm::vec2 currentStartPos(start.x, start.y);
		glm::vec2 currentDirVector(currentStopPos - currentStartPos);
		currentDirVector = glm::normalize(currentDirVector);

		float currentDirVectorLen = cv::norm(current - start);
		cv::line(src_image_, start, current, green, 2);

		if (debugMode)
			printf("timing pattern from %f %f to %f %f\n", currentStartPos.x, currentStartPos.y, currentStopPos.x,
				   currentStopPos.y);

		for (auto& cont : contours)
		{
			double epsilon = 4;
			std::vector<cv::Point> out;
			// printf("epsilon %lf\n", epsilon);
			cv::approxPolyDP(cont, out, epsilon, true);
			for (auto& p : out)
			{
				cv::circle(src_image_, p, 5, green, 1);
				float len = cv::norm(p - current);

				if (percentageDiff(len, currentDirVectorLen) < 34)
				{
					glm::vec2 p2(p.x, p.y);
					glm::vec2 dirVektor(p2 - currentStartPos);
					dirVektor = glm::normalize(dirVektor);

					/*
					printf("dirVektor %f %f    %f %f\n", dirVektor.x, dirVektor.y, currentDirVector.x,
						   currentDirVector.y);
					*/
					float currentAngle = glm::angle(currentDirVector, dirVektor);

					if (currentAngle < smallestAngle)
					{
						// printf("timing pattern %f %f   %d %d\n", currentAngle, len, p.x,p.y);
						smallestAngle	   = currentAngle;
						smallestAnglePoint = p;
					}
				}
			}
		}
		if (smallestAngle > 0.2)
			return;

		if (debugMode)
		{
			printf("smallestAngle %f\n", smallestAngle);
			printf("timing pattern from %f %f to %d %d\n", currentStartPos.x, currentStartPos.y, smallestAnglePoint.x,
				   smallestAnglePoint.y);
		}
		cv::circle(src_image_, smallestAnglePoint, 10, red, 5);

		timing_pattern.push_back(smallestAnglePoint);
	}

	/**
	 * A data matrix code has a destinctive shape. At the left and bottom, there is a row and column of black cells.
	 * So on the left and bottom, a long edge exists.
	 * At the top and right, there is a timing pattern of alternating black and white cells.
	 *
	 * We try to find the finder pattern by looking for a shape which has two long edges which are about the same size
	 * with two smaller ones next to them. Those two smaller edges are the top side of the top left cell and the right
	 * side of the bottom right cell.
	 * This will also be used to start the timing pattern which will later be refined.
	 *
	 * @param out2		Shape to check
	 */
	void checkContourForFinder(const std::vector<cv::Point> out2)
	{
		// double epsilon = 0.002 * cv::arcLength(inp, true);
		// std::vector<cv::Point> out2;
		// printf("epsilon %lf\n", epsilon);
		// cv::approxPolyDP(inp, out2, epsilon, true);
		// cv::polylines(src_image_, out2, true, green, 2, cv::LINE_AA);

		for (int i = 0; i < out2.size(); i++)
		{
			// cv::circle(src_image_, out2.at(i), 3, green, 1);
		}

		if (out2.size() < 6)
			return;

		std::vector<int> edge_lens;
		// Get all edges and calculate longest edge
		int maxLen = 0;
		for (int i = 0; i < out2.size(); i++)
		{

			int j	= (i + 1) % out2.size();
			int len = cv::norm(out2.at(i) - out2.at(j));
			edge_lens.push_back(len);
			if (len > maxLen)
				maxLen = len;

			if (debugMode)
			{
				// printf("r %d %f\n", i, cv::norm(out2.at(i) - out2.at(j)));
			}
		}

		// Search for a pattern of Short Long Long Short
		for (int i = 0; i < edge_lens.size(); i++)
		{
			int lens[4] = {
				edge_lens.at(i),
				edge_lens.at((i + 1) % edge_lens.size()),
				edge_lens.at((i + 2) % edge_lens.size()),
				edge_lens.at((i + 3) % edge_lens.size()),
			};

			int l = percentageDiff(lens[1], lens[2]);

			if (lens[2] > lens[0] * 7 && lens[1] > lens[0] * 7 && //
				lens[2] > lens[3] * 7 && lens[1] > lens[3] * 7 && l < 33)
			{
				if (debugMode)
					printf("X %d   %d %d %d %d  %d\n", out2.size(), lens[0], lens[1], lens[2], lens[3], l);

				auto timing_pattern_top_start	= out2.at(i);
				finder_top_left_				= out2.at((i + 1) % edge_lens.size());
				finder_bottom_left_				= out2.at((i + 2) % edge_lens.size());
				finder_bottom_right_			= out2.at((i + 3) % edge_lens.size());
				auto timing_pattern_right_start = out2.at((i + 4) % edge_lens.size());

				timing_pattern_top.clear();
				timing_pattern_right.clear();

				timing_pattern_top.push_back(finder_top_left_);
				timing_pattern_top.push_back(timing_pattern_top_start);

				timing_pattern_right.push_back(finder_bottom_right_);
				timing_pattern_right.push_back(timing_pattern_right_start);

				cv::circle(src_image_, timing_pattern_top_start, 10, blue, 2);
				cv::circle(src_image_, finder_top_left_, 10, green, 2);
				cv::circle(src_image_, finder_bottom_left_, 10, red, 2);
				cv::circle(src_image_, finder_bottom_right_, 10, green, 2);
				cv::circle(src_image_, timing_pattern_right_start, 10, yellow, 2);

				for (int i = 0; i < 23; i++)
				{
					continueTimingPattern(timing_pattern_top);
					continueTimingPattern(timing_pattern_right);
				}

				if (debugModeVisual)
				{
					cv::imshow("src_image_", src_image_);

					cv::waitKey(0);
				}
				if (debugMode)
					printf("timing_pattern len %d %d\n", timing_pattern_top.size(), timing_pattern_right.size());

				if (timing_pattern_top.size() < 4)
					return;

				cv::Point top_right =
					calculateLineIntercross(finder_top_left_, timing_pattern_top.back() - finder_top_left_,
											finder_bottom_right_, timing_pattern_right.back() - finder_bottom_right_);
				cv::Point2f src[4];
				cv::Point2f dst[4];

				src[0] = finder_top_left_;
				src[1] = top_right;
				src[2] = finder_bottom_right_;
				src[3] = finder_bottom_left_;

				std::array<cv::Point, 4> srcDraw;
				srcDraw[0] = finder_top_left_;
				srcDraw[1] = top_right;
				srcDraw[2] = finder_bottom_right_;
				srcDraw[3] = finder_bottom_left_;
				cv::polylines(src_image_, srcDraw, true, red, 1, cv::LINE_AA);

				dst[0] = cv::Point2f(0, 0);
				dst[1] = cv::Point2f(warped_size, 0);
				dst[2] = cv::Point2f(warped_size, warped_size);
				dst[3] = cv::Point2f(0, warped_size);

				transform = cv::getPerspectiveTransform(src, dst);

				std::vector<cv::Point2f> timing_pattern_top_float;
				std::vector<cv::Point2f> timing_pattern_top_perspective;
				std::vector<float> timing_pattern_lens;

				for (auto& p : timing_pattern_top)
				{
					timing_pattern_top_float.push_back(p);
				}

				cv::perspectiveTransform(timing_pattern_top_float, timing_pattern_top_perspective, transform);

				for (int i = 0; i < timing_pattern_top_perspective.size() - 1; i++)
				{
					timing_pattern_lens.push_back(
						cv::norm(timing_pattern_top_perspective.at(i) - timing_pattern_top_perspective.at(i + 1)));
				}

#if 1
				std::sort(timing_pattern_lens.begin(), timing_pattern_lens.end());
				cell_size_x_ = (timing_pattern_lens.at(timing_pattern_lens.size() / 2) +
								timing_pattern_lens.at(timing_pattern_lens.size() / 2 + 1) +
								timing_pattern_lens.at(timing_pattern_lens.size() / 2 - 1)) /
							   3.0f;
#else
				cell_size_x_ = (std::accumulate(timing_pattern_lens.begin(), timing_pattern_lens.end(), 0) /
								timing_pattern_lens.size());
#endif
				cell_size_y_ = cell_size_x_;

				return;
			}
		}
	}

	/**
	 * Filter the vector representation of the code and search for finder patterns.
	 */
	void searchFinderPatternPoints()
	{
		for (auto& cont : contours)
		{
			double epsilon = 4;
			std::vector<cv::Point> out;
			// printf("epsilon %lf\n", epsilon);
			cv::approxPolyDP(cont, out, epsilon, true);

			cv::polylines(src_image_, out, true, red, 2, cv::LINE_AA);
			checkContourForFinder(out);
		}
	}

	/**
	 * Get pixel position for a given cell coordinate.
	 * @param x 			Horizontal cell coordinate
	 * @param y				Vertical cell coordinate
	 * @return				Pixel position in \ref monochrome_image_warped_
	 */
	cv::Point getCellPosition(int x, int y)
	{
		// Coarse
		cv::Point point;
		point.x = round(half_cell_size_ + static_cast<float>(x) * cell_size_x_);
		point.y = round(half_cell_size_ + static_cast<float>(y) * cell_size_y_);
		return point;
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
				auto p			  = getCellPosition(x, y);
				cells.at(y).at(x) = monochrome_image_warped_.data[monochrome_image_warped_.cols * p.y + p.x] < 127;
				debug_cells.at(y).at(x) = '-';

				cv::circle(monochrome_image_warped_, p, 3, cells.at(y).at(x) ? white : black);
			}
		}
	}

	/**
	 * Gets the binary representation of a cell from the given coordinate.
	 * Wrap around is also handled here and reading will be redirected to the other side
	 * with the given offset applied to the right and down.
	 *
	 * @param x 			Horizontal cell coordinate.
	 * @param y				Vertical cell coordinate
	 * @param offset		Coordinate is shifted by this, when wrapped around
	 * @return				true, if black
	 */
	bool readCellWrapAround(int x, int y, int offset)
	{
		// fprintf(stderr, "{%d,%d}, //\n", x, y);

		if (debugMode)
			printf("Read from %d %d\n", x, y);
		if (x >= size_in_cells_ - 1)
		{
			x -= size_in_cells_ - 2;
			y -= offset;
		}
		if (x < 1)
		{
			x += size_in_cells_ - 2;
			y += offset;
		}

		if (y >= size_in_cells_ - 1)
		{
			y -= size_in_cells_ - 2;
			x -= offset;
		}
		if (y < 1)
		{
			y += size_in_cells_ - 2;
			x += offset;
		}

		if (debugMode)
			printf("Actual Read from %d %d -> %d\n", x, y, cells.at(y).at(x) ? 1 : 0);
		// printf("Actual Read from %d %d\n", x, y);

		// For debugging, ensure that it was only read once.
		assert(debug_cells.at(y).at(x) == '-');
		debug_cells.at(y).at(x) = debugCellChar;

		return cells.at(y).at(x);
	}

	/**
	 * At this point we know the size of the code. Grab the fitting configuration.
	 */
	void grabDmConfig()
	{
		for (auto& c : configs_)
		{
			if (c.size == size_in_cells_)
			{
				detected_dm_config_ = c;
				if (debugMode)
					printf("Config  %d   %d %d\n", c.size, c.db, c.ecb);
				return;
			}
		}

		assert(0);
	}

	/**
	 * With every call, one byte is extracted from the code.
	 * The task of this function is to evaluate where the next byte is positioned and in which
	 * shape the bits are aligned.
	 *
	 * @return	Raw byte from code
	 */
	uint8_t readByteFromCells()
	{
		uint8_t result = 0;
		if (debugMode)
			printf("readByteFromCells %c %d %d\n", debugCellChar, current_cell_position_.x, current_cell_position_.y);

		if (current_cell_position_.x == 1 && current_cell_position_.y >= size_in_cells_ - 3 &&
			detected_dm_config_.cornerCaseId != 0)
		{
			if (debugMode)
			{
				printf("Corner Case %d at %c\n", detected_dm_config_.cornerCaseId, debugCellChar);
				fprintf(stderr, "-100, //Corner Case\n");
			}
			if (debugPosRefIt != debugPosRef.end())
			{
				int ref = *debugPosRefIt;
				debugPosRefIt++;
				assert(ref == -100);
			}

			for (const auto& order : detected_dm_config_.cornercase)
			{
				auto p = order;
				result <<= 1;
				if (readCellWrapAround(p.x, p.y, 0))
					result |= 1;
			}
		}
		else
		{
			if (debugMode)
				fprintf(stderr, "%d,%d, //Standard Case\n", current_cell_position_.x, current_cell_position_.y);

			if (debugPosRefIt != debugPosRef.end())
			{
				int refX = *debugPosRefIt;
				debugPosRefIt++;
				int refY = *debugPosRefIt;
				debugPosRefIt++;

				if (refX != current_cell_position_.x || refY != current_cell_position_.y)
				{
					fprintf(stderr, "Expected %d %d\n", refX, refY);
					assert(0);
				}
			}

			for (const auto& order : cell_positions_)
			{
				auto p = order + current_cell_position_;
				result <<= 1;
				if (readCellWrapAround(p.x, p.y, detected_dm_config_.overlapoffset))
					result |= 1;
			}
		}

		if (debugMode)
			printf("readByteFromCells go %c %d %d\n", debugCellChar, current_cell_position_.x,
				   current_cell_position_.y);

		if (goUpRight)
		{
			if (current_cell_position_.x == size_in_cells_ - 5 && current_cell_position_.y == 3 &&
				detected_dm_config_.cornerCaseId == 2)
			{
				current_cell_position_.x += 3;
				current_cell_position_.y += 1;
				goUpRight = false;
			}
			else if (current_cell_position_.x + 2 >= size_in_cells_ - 1)
			{
				current_cell_position_.x += 1;
				current_cell_position_.y += 3;
				goUpRight = false;
			}
			else if (current_cell_position_.y - 2 < 1)
			{
				current_cell_position_.x += 3;
				current_cell_position_.y += 1;
				goUpRight = false;
			}
			else
			{
				current_cell_position_.x += 2;
				current_cell_position_.y -= 2;
			}
		}
		else
		{
			if (current_cell_position_.y + 2 >= size_in_cells_ - 1)
			{
				current_cell_position_.x += 5;
				current_cell_position_.y -= 1;
				goUpRight = true;
			}
			else if (current_cell_position_.x == 2 && current_cell_position_.y == size_in_cells_ - 4)
			{
				current_cell_position_.x += 3;
				current_cell_position_.y += 1;
				goUpRight = true;
			}
			else if (current_cell_position_.x == 2)
			{
				current_cell_position_.x -= 1;
				current_cell_position_.y += 5;
				goUpRight = true;
			}
			else
			{
				current_cell_position_.x -= 2;
				current_cell_position_.y += 2;
			}
		}

		if (debugCellChar == 'Z')
			debugCellChar = 'a';
		else if (debugCellChar == 'z')
			debugCellChar = '0';
		else
			debugCellChar++;
		return result;
	}

	/// Set 0 of characters in Text mode
	std::array<char, 40> text_mode_set0 = {
		' ', ' ', ' ', ' ', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f',
		'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
	};

	/// Set 1 of characters in Text mode. Reachable from set 0
	std::array<char, 32> text_mode_set2 = {'!', '"', '#', '$', '%', '&', '\'', '(', ')', '*',  '+', ',', '-',
										   '.', '/', ':', ';', '<', '=', '>',  '?', '[', '\\', ']', '^', '_'};

	/// Set 2 of characters in Text mode. Reachable from set 0
	std::array<char, 32> text_mode_set3 = {'`', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
										   'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U',
										   'V', 'W', 'X', 'Y', 'Z', '{', '|', '}', '~', -1};

	/// Set 0 of characters in C40 mode
	std::array<char, 40> c40_mode_set0 = {
		' ', ' ', ' ', ' ', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
		'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
	};

	/**
	 * Using a small state machine, this function reconstructs C40 and Text mode data as those modes
	 * do have a state which selects character tables.
	 * Helper function of \ref extractPayload
	 * @param raw	Raw word. 0 to 39 are allowed values.
	 */
	void reconstructTextModeData(uint8_t raw)
	{
		char ret = 0;

		switch (text_mode_set)
		{
		case TextMode::Text_Set_0:
			switch (raw)
			{
			case 0:
				text_mode_set = TextMode::Text_Set_1;
				break;
			case 1:
				text_mode_set = TextMode::Text_Set_2;
				break;
			case 2:
				text_mode_set = TextMode::Text_Set_3;
				break;
			default:
				if (debugMode)
					printf("%c\n", text_mode_set0.at(raw));
				payload_data.push_back(text_mode_set0.at(raw));
				break;
			}
			break;
		case TextMode::Text_Set_1:
			assert(0); /// TODO not yet implemented
			break;
		case TextMode::Text_Set_2:
			text_mode_set = TextMode::Text_Set_0;
			if (debugMode)
				printf("%c\n", text_mode_set2.at(raw));
			payload_data.push_back(text_mode_set2.at(raw));
			break;
		case TextMode::Text_Set_3:
			text_mode_set = TextMode::Text_Set_0;
			if (debugMode)
				printf("%c\n", text_mode_set3.at(raw));
			payload_data.push_back(text_mode_set3.at(raw));
			break;
		case TextMode::C40_Set_0:
			switch (raw)
			{
			case 0:
				text_mode_set = TextMode::C40_Set_1;
				break;
			case 1:
				text_mode_set = TextMode::C40_Set_2;
				break;
			case 2:
				text_mode_set = TextMode::C40_Set_3;
				break;
			default:
				if (debugMode)
					printf("%c\n", c40_mode_set0.at(raw));
				payload_data.push_back(c40_mode_set0.at(raw));
				break;
			}
			break;
		default:
			assert(0);
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
	 * Utility function which gives the vertical distance in pixels from the current point
	 * until a white pixel is reached. Can be used on the timing pattern cells
	 * to check how well the cell size is tuned.
	 *
	 * @param image		Monochrome image
	 * @param xStart	X position of pixel
	 * @param y			Y position of pixel
	 * @return			Pair of top distance and bottom distance
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
			int diff		= (dist.first - dist.second) / 2;
			float x			= round((pixelX - half_cell_size_) / cell_size_x_);
			cell_size_x_	= (pixelX - diff - half_cell_size_) / x;
			half_cell_size_ = cell_size_x_ / 2;
		}
	}

	/**
	 * The estimated cell size might not be 100% correct.
	 * Luckily the Data Matrix code offers two timing patterns with alternating black
	 * and white cells. We use the top pattern and go from left to right, refining
	 * the cell size with each black cell.
	 */
	void optimizeCellsizeWithTimingPattern()
	{
		int pixelX;
		int pixelY;
		int cellX;

		int timing_pattern_black_cells = size_in_cells_ / 2;

		if (debugMode)
			printf("cellsize at start %f %f\n", cell_size_x_, cell_size_y_);

		for (int i = 1; i < timing_pattern_black_cells; i++)
		{
			cellX  = i * 2;
			pixelX = round(half_cell_size_ + static_cast<float>(cellX) * cell_size_x_);
			// printf("Check %d\n", pixelX);
			pixelY = half_cell_size_;

			optimizeCellsizeWithEnclosedCell(pixelX, pixelY, false);

			pixelX = round(half_cell_size_ + static_cast<float>(cellX) * cell_size_x_);

			auto dist = horizontalDistance(monochrome_image_warped_, pixelX, pixelY);
			if (debugMode)
				printf("Vertical distance After %d %d\n", dist.first, dist.second);
		}

		if (debugMode)
			printf("cellsize at end %f %f\n", cell_size_x_, cell_size_y_);
	}

	/**
	 * After the whole code was stored as raw byte data, this function extracts the stored
	 * payload as ASCII data and stores it in \ref payload_data
	 * @param raw_decoded_data	All bytes of the data matrix
	 */
	void extractPayload(std::vector<uint8_t>& raw_decoded_data)
	{
		payload_data.clear();
		auto it = raw_decoded_data.begin();

		enum
		{
			ASCII,
			TEXT,
			END
		} mode = ASCII;

		while (it != raw_decoded_data.end())
		{
			switch (mode)
			{
			case ASCII:
				if (*it == 239)
				{
					if (debugMode)
						printf("Begin Text Encoding\n");
					text_mode_set = TextMode::Text_Set_0;
					mode		  = TEXT;
				}
				else if (*it == 129)
				{
					if (debugMode)
						printf("End of message\n");
					mode = END;
					return;
				}
				else if (*it == 230)
				{
					if (debugMode)
						printf("Begin C40 Encoding\n");
					mode		  = TEXT;
					text_mode_set = TextMode::C40_Set_0;
				}
				else if (*it <= 128)
				{
					if (debugMode)
						printf("ascii code %d %x %c\n", *it, *it, *it);
					payload_data.push_back(*it - 1);
				}
				else
				{
					assert(0);
				}
				it++;
				break;
			case TEXT:
			{
				if (it[0] == 254)
				{
					if (debugMode)
						printf("Return to ASCII\n");
					mode = ASCII;
				}
				else
				{
					uint16_t textWord = it[0] * 256 + it[1];
					it++;
					assert(it != raw_decoded_data.end());

					textWord--;
					uint16_t c1 = textWord / 1600;
					uint16_t c3 = (textWord % 1600) % 40;
					uint16_t c2 = (textWord % 1600) / 40;
					assert((c1 * 1600 + c2 * 40 + c3 + 1) == (textWord + 1));

					if (debugMode)
						printf("text code %d %x   %d %d %d\n", textWord, textWord, c1, c2, c3);
					reconstructTextModeData(c1);
					reconstructTextModeData(c2);
					reconstructTextModeData(c3);
				}
				it++;
				break;
			}
			case END:
				break;
			}
		}
	}

  public:
	/**
	 * The way, the bytes are placed in the data matrix is more complicated, as the column layout of the qr code.
	 * Every byte base position (the bottom right cell) which is expected to be used, can be stored here
	 * from a unit test. If \ref readByteFromCells is not reading from the expected positions,
	 * the decoding is stopped immediately. Very helpful for debugging.
	 */
	std::vector<int> debugPosRef;

	/// Helper, to iterate ober \ref debugPosRef.
	std::vector<int>::iterator debugPosRefIt;

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
			fprintf(stderr, "Reading %s\n", filepath);
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
		{
			cv::resize(src_image_as_grayscale, grayscale_blurred, cv::Size(0, 0), 3, 3);
			cv::resize(src_image_, src_image_, cv::Size(0, 0), 3, 3);
		}
		else
		{
			cv::GaussianBlur(src_image_as_grayscale, grayscale_blurred, cv::Size(11, 11), 0, 0);
		}

		// Convert grayscale to monochrome image
		cv::adaptiveThreshold(grayscale_blurred, monochrome_image_inverted, 255, cv::ADAPTIVE_THRESH_MEAN_C,
							  cv::THRESH_BINARY_INV, 201, 0);

		// Find Contours in the monochrome image
		cv::findContours(monochrome_image_inverted, contours, cv::RETR_LIST, cv::CHAIN_APPROX_TC89_KCOS);

		if (debugModeVisual)
		{
			cv::imshow("src_image_as_grayscale", src_image_as_grayscale);
			cv::imshow("grayscale_blurred", grayscale_blurred);
			cv::imshow("monochrome_image_inverted", monochrome_image_inverted);
			cv::imshow("src_image_", src_image_);

			cv::waitKey(0);
		}

		searchFinderPatternPoints();

		cv::bitwise_not(monochrome_image_inverted, monochrome_image_normal);

		cv::warpPerspective(monochrome_image_normal, monochrome_image_warped_, transform,
							cv::Size(warped_size, warped_size));

		float cellsizeAvg  = (cell_size_x_ + cell_size_y_) / 2;
		half_cell_size_	   = cellsizeAvg / 2;
		float sizeInCellsF = static_cast<float>(warped_size) / cellsizeAvg;

		size_in_cells_ = round(sizeInCellsF);
		if (debugMode)
			printf("size_in_cells_ %d\n", size_in_cells_);

		if (debugModeVisual)
		{
			cv::imshow("src_image_as_grayscale", src_image_as_grayscale);
			cv::imshow("grayscale_blurred", grayscale_blurred);
			cv::imshow("monochrome_image_inverted", monochrome_image_inverted);
			cv::imshow("src_image_", src_image_);
			cv::imshow("monochrome_image_warped", monochrome_image_warped_);

			cv::waitKey(0);
		}

		optimizeCellsizeWithTimingPattern();

		cell_size_y_ = cell_size_x_;

		cellsizeAvg	   = (cell_size_x_ + cell_size_y_) / 2;
		sizeInCellsF   = static_cast<float>(warped_size) / cellsizeAvg;
		size_in_cells_ = round(sizeInCellsF);
		if (debugMode)
		{
			printf("effective size_in_cells_ %d\n", size_in_cells_);
			fprintf(stderr, "effective size_in_cells_ %d\n", size_in_cells_);
		}
		grabDmConfig();

		extractCells();

		if (debugModeVisual)
		{
			cv::imshow("src_image_as_grayscale", src_image_as_grayscale);
			cv::imshow("grayscale_blurred", grayscale_blurred);
			cv::imshow("monochrome_image_inverted", monochrome_image_inverted);
			cv::imshow("src_image_", src_image_);
			cv::imshow("monochrome_image_warped", monochrome_image_warped_);

			cv::waitKey(0);
		}

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

		current_cell_position_.x = 1;
		current_cell_position_.y = 5;

		uint8_t byte;

		std::vector<FFieldDm> rawData;

		int n = detected_dm_config_.db + detected_dm_config_.ecb;
		int k = detected_dm_config_.db;

		if (debugMode)
			printf("n %d  k %d\n", n, k);
		//		int n = 60;
		//	int k = 36;

		for (int i = 0; i < n; i++)
		{
			if (debugMode)
				printf("Reading i=%d\n", i);
			byte = readByteFromCells();
			if (debugMode)
				printf("word %c  %d %x\n", debugCellChar - 1, byte, byte);
			rawData.push_back(byte);
		}

		if (debugMode)
		{
			printf("--- Debug Cells ------\n");
			for (const auto& c : debug_cells)
			{
				for (const auto& p : c)
				{
					printf("%c", p);
				}
				printf("\n");
			}
		}

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

		if (debugMode)
		{
			printf("In:  ");
			for (auto& v : rawData)
			{
				printf("%02x ", v);
			}
			printf("\n");
		}

		RS::ReedSolomon<FFieldDm> rs(n, k, 1);
		std::reverse(rawData.begin(), rawData.end());
		auto rsPolynomial		= Polynom<FFieldDm>(rawData);
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

		std::vector<uint8_t> raw_decoded_data;
		for (auto it = std::begin(raw_decoded_data_extfield); it != std::end(raw_decoded_data_extfield); ++it)
		{
			raw_decoded_data.push_back(*it);
		}

		extractPayload(raw_decoded_data);
		return payload_data;
	}
};

#endif /* SRC_DATAMATRIXDECODER_H_ */
