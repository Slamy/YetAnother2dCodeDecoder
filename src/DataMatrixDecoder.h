/*
 * DataMatrixDecoder.h
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
#include "qrcodeexception.h"
#include "util.h"
#include <numeric>
#include <opencv2/opencv.hpp>
#include <span>

class DataMatrixDecoder
{
  private:
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

	cv::Point current_cell_position_;
	char debugCellChar = 'A';
	bool goUpRight{true};

	std::vector<std::vector<bool>> cells;
	std::vector<std::vector<char>> debug_cells;

	int size_in_cells_;

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

	TextMode text_mode_set = TextMode::C40_Set_0;
	std::vector<char> payload_data;

	const cv::Scalar green{0, 255, 0};
	const cv::Scalar black{0, 0, 0};
	const cv::Scalar blue{255, 0, 0};
	const cv::Scalar red{0, 0, 255};
	const cv::Scalar yellow{0, 255, 255};
	const cv::Scalar white{255, 255, 255};

	static constexpr int warped_size = 1000;
	cv::Mat src_image_;
	cv::Mat transform;
	cv::Mat monochrome_image_warped;

	std::vector<std::vector<cv::Point>> contours;

	int percentageDiff(int a, int b)
	{
		return labs(a - b) * 100 / ((a + b) / 2);
	}

	cv::Point finder_bottom_left_;
	cv::Point finder_top_left_;
	cv::Point finder_bottom_right_;
	std::vector<cv::Point> timing_pattern_top;
	std::vector<cv::Point> timing_pattern_right;

	float cellsizeX, cellsizeY;
	float half_cell_size_;

	const std::array<cv::Point, 8> cellPositions{{
		{-2, -2}, //
		{-1, -2}, //
		{-2, -1}, //
		{-1, -1}, //
		{0, -1},  //
		{-2, 0},  //
		{-1, 0},  //
		{0, 0},	  //
	}};

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

	struct DmConfig
	{
		int size;
		int overlapoffset;
		std::span<const cv::Point> cornercase;
		int cornerCaseId;
		int db;
		int ecb;
	};

	struct DmConfig detected_dm_config_;

	// TODO still a lot configs missing
	// taken from https://barcode-coder.com/en/datamatrix-specification-104.html
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

#if 1
		printf("timing pattern from %f %f to %f %f\n", currentStartPos.x, currentStartPos.y, currentStopPos.x,
			   currentStopPos.y);
#endif

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

		printf("smallestAngle %f\n", smallestAngle);
		printf("timing pattern from %f %f to %d %d\n", currentStartPos.x, currentStartPos.y, smallestAnglePoint.x,
			   smallestAnglePoint.y);

		cv::circle(src_image_, smallestAnglePoint, 10, red, 5);

		timing_pattern.push_back(smallestAnglePoint);
	}

	void checkContourForFinder(std::vector<cv::Point> out2)
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
				cellsizeX = (timing_pattern_lens.at(timing_pattern_lens.size() / 2) +
							 timing_pattern_lens.at(timing_pattern_lens.size() / 2 + 1) +
							 timing_pattern_lens.at(timing_pattern_lens.size() / 2 - 1)) /
							3.0f;
#else
				cellsizeX = (std::accumulate(timing_pattern_lens.begin(), timing_pattern_lens.end(), 0) /
							 timing_pattern_lens.size());
#endif
				cellsizeY = cellsizeX;

				return;
			}
		}
	}

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

	cv::Point getCellPosition(int x, int y)
	{
		// Coarse
		cv::Point point;
		point.x = round(half_cell_size_ + static_cast<float>(x) * cellsizeX);
		point.y = round(half_cell_size_ + static_cast<float>(y) * cellsizeY);
		return point;
	}

	void extractCells()
	{
		for (int y = 0; y < size_in_cells_; y++)
		{
			cells.push_back(std::vector<bool>(size_in_cells_));
			debug_cells.push_back(std::vector<char>(size_in_cells_));

			for (int x = 0; x < size_in_cells_; x++)
			{
				auto p					= getCellPosition(x, y);
				cells.at(y).at(x)		= monochrome_image_warped.data[monochrome_image_warped.cols * p.y + p.x] < 127;
				debug_cells.at(y).at(x) = '-';

				cv::circle(monochrome_image_warped, p, 3, cells.at(y).at(x) ? white : black);
			}
		}
	}

	bool readCellWrapAround(int x, int y, int offset)
	{
		// fprintf(stderr, "{%d,%d}, //\n", x, y);

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

		printf("Actual Read from %d %d -> %d\n", x, y, cells.at(y).at(x) ? 1 : 0);
		// printf("Actual Read from %d %d\n", x, y);

		// For debugging, ensure that it was only read once.
		assert(debug_cells.at(y).at(x) == '-');
		debug_cells.at(y).at(x) = debugCellChar;

		return cells.at(y).at(x);
	}

	void grabDmConfig()
	{
		for (auto& c : configs_)
		{
			if (c.size == size_in_cells_)
			{
				detected_dm_config_ = c;
				printf("Config  %d   %d %d\n", c.size, c.db, c.ecb);
				return;
			}
		}

		assert(0);
	}

	uint8_t readByteFromCells()
	{
		// std::span<const cv::Point, 8> r = cornerCase2;

		uint8_t result = 0;
		printf("readByteFromCells %c %d %d\n", debugCellChar, current_cell_position_.x, current_cell_position_.y);

		if (current_cell_position_.x == 1 && current_cell_position_.y >= size_in_cells_ - 3 &&
			detected_dm_config_.cornerCaseId != 0)
		{
			printf("Corner Case %d at %c\n", detected_dm_config_.cornerCaseId, debugCellChar);

			fprintf(stderr, "-100, //Corner Case\n");
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

			for (const auto& order : cellPositions)
			{
				auto p = order + current_cell_position_;
				result <<= 1;
				if (readCellWrapAround(p.x, p.y, detected_dm_config_.overlapoffset))
					result |= 1;
			}
		}

		printf("readByteFromCells go %c %d %d\n", debugCellChar, current_cell_position_.x, current_cell_position_.y);

		if (goUpRight)
		{
			if (current_cell_position_.x == size_in_cells_ - 5 && current_cell_position_.y == 3 &&
				detected_dm_config_.cornerCaseId == 2)
			{
				printf("Special %s %d\n", __func__, __LINE__);
				current_cell_position_.x += 3;
				current_cell_position_.y += 1;
				goUpRight = false;
			}
			else if (current_cell_position_.x + 2 >= size_in_cells_ - 1)
			{
				printf("%s %d\n", __func__, __LINE__);
				current_cell_position_.x += 1;
				current_cell_position_.y += 3;
				goUpRight = false;
			}
			else if (current_cell_position_.y - 2 < 1)
			{
				printf("%s %d\n", __func__, __LINE__);
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
				printf("%s %d\n", __func__, __LINE__);
				current_cell_position_.x += 5;
				current_cell_position_.y -= 1;
				goUpRight = true;
			}
			else if (current_cell_position_.x == 2 && current_cell_position_.y == size_in_cells_ - 4)
			{
				printf("%s %d\n", __func__, __LINE__);
				current_cell_position_.x += 3;
				current_cell_position_.y += 1;
				goUpRight = true;
			}
			else if (current_cell_position_.x == 2)
			{
				printf("%s %d\n", __func__, __LINE__);
				// current_cell_position_.x += 1;
				// current_cell_position_.y += 3;
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

	std::array<char, 40> text_mode_set0 = {
		' ', ' ', ' ', ' ', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f',
		'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
	};

	std::array<char, 32> text_mode_set2 = {'!', '"', '#', '$', '%', '&', '\'', '(', ')', '*',  '+', ',', '-',
										   '.', '/', ':', ';', '<', '=', '>',  '?', '[', '\\', ']', '^', '_'};

	std::array<char, 32> text_mode_set3 = {'`', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J',
										   'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U',
										   'V', 'W', 'X', 'Y', 'Z', '{', '|', '}', '~', -1};

	std::array<char, 40> c40_mode_set0 = {
		' ', ' ', ' ', ' ', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F',
		'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
	};

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
				printf("%c\n", text_mode_set0.at(raw));
				payload_data.push_back(text_mode_set0.at(raw));
				break;
			}
			break;
		case TextMode::Text_Set_1:
			assert(0);
			break;
		case TextMode::Text_Set_2:
			text_mode_set = TextMode::Text_Set_0;
			printf("%c\n", text_mode_set2.at(raw));
			payload_data.push_back(text_mode_set2.at(raw));
			break;
		case TextMode::Text_Set_3:
			text_mode_set = TextMode::Text_Set_0;
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
				printf("%c\n", c40_mode_set0.at(raw));
				payload_data.push_back(c40_mode_set0.at(raw));
				break;
			}
			break;
		default:
			assert(0);
		}
	}

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

	void optimizeCellsizeWithEnclosedCell(int pixelX, int pixelY, bool vertical)
	{
		bool val = monochrome_image_warped.data[monochrome_image_warped.cols * pixelY + pixelX] < 127;
		assert(val);

		if (vertical)
		{
			auto dist = verticalDistance(monochrome_image_warped, pixelX, pixelY);
			if (debugMode)
				printf("Vertical distance Start %d %d\n", dist.first, dist.second);
			// (pixelY + diff - halfCellSize)/y =
			int diff  = (dist.first - dist.second) / 2;
			float y	  = round((pixelY - half_cell_size_) / cellsizeY);
			cellsizeY = (pixelY - diff - half_cell_size_) / y;
		}
		else
		{
			auto dist = horizontalDistance(monochrome_image_warped, pixelX, pixelY);
			if (debugMode)
				printf("Horizontal Distance Start %d %d\n", dist.first, dist.second);
			int diff		= (dist.first - dist.second) / 2;
			float x			= round((pixelX - half_cell_size_) / cellsizeX);
			cellsizeX		= (pixelX - diff - half_cell_size_) / x;
			half_cell_size_ = cellsizeX / 2;
		}
	}
	void optimizeCellsizeWithTimingPattern()
	{
		int pixelX;
		int pixelY;
		int cellX;

		int timing_pattern_black_cells = size_in_cells_ / 2;

		if (debugMode)
			printf("cellsize at start %f %f\n", cellsizeX, cellsizeY);

		for (int i = 1; i < timing_pattern_black_cells; i++)
		{
			cellX  = i * 2;
			pixelX = round(half_cell_size_ + static_cast<float>(cellX) * cellsizeX);
			// printf("Check %d\n", pixelX);
			pixelY = half_cell_size_;

			optimizeCellsizeWithEnclosedCell(pixelX, pixelY, false);

			pixelX = round(half_cell_size_ + static_cast<float>(cellX) * cellsizeX);

			auto dist = horizontalDistance(monochrome_image_warped, pixelX, pixelY);
			if (debugMode)
				printf("Vertical distance After %d %d\n", dist.first, dist.second);
		}

		if (debugMode)
			printf("cellsize at end %f %f\n", cellsizeX, cellsizeY);
	}

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
					printf("Begin Text Encoding\n");
					text_mode_set = TextMode::Text_Set_0;
					mode		  = TEXT;
				}
				else if (*it == 129)
				{
					printf("End of message\n");
					mode = END;
					return;
				}
				else if (*it == 230)
				{
					printf("Begin C40 Encoding\n");
					mode		  = TEXT;
					text_mode_set = TextMode::C40_Set_0;
				}
				else if (*it <= 128)
				{
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
	std::vector<int> debugPosRef;
	std::vector<int>::iterator debugPosRefIt;

	bool debugMode{false};
	bool debugModeVisual{false};

	bool code_needed_correction_{false};

	std::vector<char> decodeFromFile(const char* filepath)
	{
		reset();

		fprintf(stderr, "Reading %s\n", filepath);
		if (debugMode)
		{
			printf("Reading %s\n", filepath);
		}

		src_image_ = cv::imread(filepath, 1);
		if (!src_image_.data)
		{
			printf("No image data \n");
			throw QrCodeException(QrCodeException::Cause::kFileNotReadable);
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

		cv::warpPerspective(monochrome_image_normal, monochrome_image_warped, transform,
							cv::Size(warped_size, warped_size));

		float cellsizeAvg  = (cellsizeX + cellsizeY) / 2;
		half_cell_size_	   = cellsizeAvg / 2;
		float sizeInCellsF = static_cast<float>(warped_size) / cellsizeAvg;

		size_in_cells_ = round(sizeInCellsF);
		printf("size_in_cells_ %d\n", size_in_cells_);

		if (debugModeVisual)
		{
			cv::imshow("src_image_as_grayscale", src_image_as_grayscale);
			cv::imshow("grayscale_blurred", grayscale_blurred);
			cv::imshow("monochrome_image_inverted", monochrome_image_inverted);
			cv::imshow("src_image_", src_image_);
			cv::imshow("monochrome_image_warped", monochrome_image_warped);

			cv::waitKey(0);
		}

		optimizeCellsizeWithTimingPattern();

		cellsizeY = cellsizeX;

		cellsizeAvg	   = (cellsizeX + cellsizeY) / 2;
		sizeInCellsF   = static_cast<float>(warped_size) / cellsizeAvg;
		size_in_cells_ = round(sizeInCellsF);
		printf("effective size_in_cells_ %d\n", size_in_cells_);
		fprintf(stderr, "effective size_in_cells_ %d\n", size_in_cells_);

		grabDmConfig();

		extractCells();

		if (debugModeVisual)
		{
			cv::imshow("src_image_as_grayscale", src_image_as_grayscale);
			cv::imshow("grayscale_blurred", grayscale_blurred);
			cv::imshow("monochrome_image_inverted", monochrome_image_inverted);
			cv::imshow("src_image_", src_image_);
			cv::imshow("monochrome_image_warped", monochrome_image_warped);

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

		std::vector<ExtFiniteField256> rawData;

		int n = detected_dm_config_.db + detected_dm_config_.ecb;
		int k = detected_dm_config_.db;

		printf("n %d  k %d\n", n, k);
		//		int n = 60;
		//	int k = 36;

		for (int i = 0; i < n; i++)
		{
			printf("Reading i=%d\n", i);
			byte = readByteFromCells();
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

		RS::ReedSolomon<ExtFiniteField256> rs(n, k);
		std::reverse(rawData.begin(), rawData.end());
		auto rsPolynomial		= Polynom<ExtFiniteField256>(rawData);
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
		printf("%s %d\n", __func__, __LINE__);
		return payload_data;
	}
};

#endif /* SRC_DATAMATRIXDECODER_H_ */
