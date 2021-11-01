/*
 * QrDecoder.h
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
#include "util.h"
#include <opencv2/opencv.hpp>

class FinderPattern
{
  public:
	// std::vector<cv::Point> points;
	cv::Point outer;
	cv::Point inner;
	cv::Point out_to_in;
	std::vector<cv::Point> sides{2};
	cv::Point outer_to_finderless_edge;

	FinderPattern(const std::vector<cv::Point>& in, cv::Point center)
	{
		int innerst_id	= -1;
		int innerstdist = 9999;

		for (int i = 0; i < in.size(); i++)
		{
			if (cv::norm(in.at(i) - center) < innerstdist)
			{
				innerstdist = cv::norm(in.at(i) - center);
				innerst_id	= i;
			}
		}

		inner	 = in.at(innerst_id);
		outer	 = in.at((innerst_id + 2) % 4);
		sides[0] = in.at((innerst_id + 1) % 4);
		sides[1] = in.at((innerst_id + 3) % 4);

		out_to_in = inner - outer;
		int len	  = cv::norm(out_to_in);

		out_to_in.x = out_to_in.x * 100 / len;
		out_to_in.y = out_to_in.y * 100 / len;
	}

	void calculateFinderLessEdgeAngle(FinderPattern& thirdPattern)
	{
		if (cv::norm(thirdPattern.outer - sides.at(0)) > cv::norm(thirdPattern.outer - sides.at(1)))
			outer_to_finderless_edge = sides.at(0) - outer;
		else
			outer_to_finderless_edge = sides.at(1) - outer;
	}
};

class QrDecoder
{
  private:
	const cv::Scalar green{0, 255, 0};
	const cv::Scalar red{0, 0, 255};
	const cv::Scalar blue{255, 0, 0};

	cv::Mat src_image_;

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	std::vector<std::vector<cv::Point>> finder_contour_points;
	cv::Point average_center_pos;
	std::vector<FinderPattern> finder2;

	const int warped_size = 1000;
	float cellsize;
	float halfCellSize;
	cv::Mat transform;

	std::vector<std::vector<bool>> cells;
	std::vector<std::vector<char>> debug_cells;
	char debug_cell_val = '_';

	int sizeInCells;

	cv::Mat monochrome_image_warped;
	cv::Point analyzePosition;

	static constexpr int err_L = 1;
	static constexpr int err_M = 0;
	static constexpr int err_Q = 3;
	static constexpr int err_H = 2;

	struct formatCode
	{
		int err;
		int mask;
		uint32_t code;
	};

	struct formatCode detectedFormatCode;

	void searchFinderPatternPoints(int id, int candidateForFinderPattern)
	{
#if 1
		// cv::Scalar colour((rand() & 255), (rand() & 255), (rand() & 255));

		std::vector<cv::Scalar> colors = {
			{0, 0, 255},
			{0, 255, 0},
			{255, 0, 0},
			{255, 0, 255},
		};
		cv::Scalar squarecolour(0, 0, 255);

		cv::Scalar colour(0, 255, 0);
#endif

		for (;;)
		{
			// double epsilon = cv::arcLength(contours.at(id), true);
			std::vector<cv::Point> out;
			cv::approxPolyDP(contours.at(id), out, 6, true);

#if 1
			for (auto& v : out)
			{
				// cv::circle(src_image_, v, 10, colour);
			}
#endif

			int nextparam = 0;
			if (out.size() == 4)
			{
				printf("%d %d %d %d\n", id, contours.at(id).size(), out.size(), candidateForFinderPattern);

				if (candidateForFinderPattern == 2)
				{
					int outer1 = hierarchy.at(id)[3];
					int outer2 = hierarchy.at(outer1)[3];
					std::vector<cv::Point> out2;
					cv::approxPolyDP(contours.at(outer2), out2, 6, true);
					finder_contour_points.push_back(out2);
				}

				cv::polylines(src_image_, out, true, colors.at(0), 10, cv::LINE_AA);

				// candidateForFinderPattern++;
				nextparam = candidateForFinderPattern + 1;
			}
			else
			{
				// cv::polylines(image, out, true, colour, 3, cv::LINE_AA);
				nextparam = 0;
			}

			if (hierarchy.at(id)[2] >= 0)
			{
				searchFinderPatternPoints(hierarchy.at(id)[2], nextparam);
			}

			id = hierarchy.at(id)[0];
			if (id == -1)
				break;
		}
	}

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

		assert(mask == 2); // TODO

		for (int y = 0; y < sizeInCells; y++)
		{
			for (int x = 0; x < sizeInCells; x++)
			{
				if ((x % 3) == 0)
					cells.at(y).at(x) = !cells.at(y).at(x);
			}
		}
	}

	void assumeFinderPatternCenter()
	{
		average_center_pos.x = 0;
		average_center_pos.y = 0;

		for (const auto& c : finder_contour_points)
		{
			cv::polylines(src_image_, c, true, green, 1, cv::LINE_AA);

			for (const auto& p : c)
			{
				average_center_pos.x += p.x;
				average_center_pos.y += p.y;
			}
		}

		average_center_pos.x /= (finder_contour_points.size() * 4);
		average_center_pos.y /= (finder_contour_points.size() * 4);
	}

	void reconstructShapeOfCode()
	{
		assert(finder_contour_points.size() == 3);

		cv::circle(src_image_, average_center_pos, 10, red, 3);

		for (const auto& c : finder_contour_points)
		{
			finder2.emplace_back(c, average_center_pos);
#if 0
			FinderPattern pattern(c, average_center_pos);
			cv::circle(src_image_, pattern.inner, 10, green, 3);
			cv::circle(src_image_, pattern.outer, 10, red, 3);
			cv::circle(src_image_, pattern.sides[0], 10, blue, 3);
			cv::circle(src_image_, pattern.sides[1], 10, blue, 3);
#endif

			// cv::circle(image, pattern.inner + pattern.out_to_in, 20, blue, 3);
		}

		int dot01 = labs(finder2.at(0).out_to_in.dot(finder2.at(1).out_to_in));
		int dot02 = labs(finder2.at(0).out_to_in.dot(finder2.at(2).out_to_in));
		int dot12 = labs(finder2.at(1).out_to_in.dot(finder2.at(2).out_to_in));

		printf("%d %d %d\n", dot01, dot02, dot12);
		// cv::circle(image, finder2.at(1).outer, 10, red, 3);

		// TODO use dot product to check for real upper left pattern
#if 1
		FinderPattern thirdPattern = finder2.at(2);
		FinderPattern sides[2]{finder2.at(1), finder2.at(0)};
#else
		FinderPattern thirdPattern = finder2.at(1); // TODO
		FinderPattern sides[2]{finder2.at(0), finder2.at(2)};
#endif

		sides[0].calculateFinderLessEdgeAngle(thirdPattern);
		sides[1].calculateFinderLessEdgeAngle(thirdPattern);

		cv::circle(src_image_, sides[0].outer, 10, red, 3);
		cv::circle(src_image_, sides[1].outer, 10, red, 3);

		cv::circle(src_image_, sides[0].outer + sides[0].outer_to_finderless_edge, 10, green, 3);
		cv::circle(src_image_, sides[1].outer + sides[1].outer_to_finderless_edge, 10, green, 3);

		Matrix<float> linearsolver{
			{sides[0].outer_to_finderless_edge.x, -sides[1].outer_to_finderless_edge.x,
			 sides[1].outer.x - sides[0].outer.x},
			{sides[0].outer_to_finderless_edge.y, -sides[1].outer_to_finderless_edge.y,
			 sides[1].outer.y - sides[0].outer.y},
		};

		auto coefficients = linearsolver.gauss_jordan_elim();
		// printf("%f %f\n", coefficients.x, solution.y);
		linearsolver.print();
		cv::Point solution = sides[0].outer + coefficients.at(0) * sides[0].outer_to_finderless_edge;
		printf("edge of code %d %d\n", solution.x, solution.y);

		cv::circle(src_image_, solution, 5, red, 3);

		cv::Point2f src[4];
		cv::Point2f dst[4];

		// TODO Order might be wrong.
		src[0] = thirdPattern.outer;
		src[1] = sides[0].outer;
		src[2] = solution;
		src[3] = sides[1].outer;

		dst[0] = cv::Point2f(0, 0);
		dst[1] = cv::Point2f(warped_size, 0);
		dst[2] = cv::Point2f(warped_size, warped_size);
		dst[3] = cv::Point2f(0, warped_size);

		transform = cv::getPerspectiveTransform(src, dst);

		// std::array<cv::Point2f, 2> src2{sides[0].outer, sides[0].sides.at(0)};
		std::array<cv::Point2f, 12> src2{
			thirdPattern.outer,		  //
			thirdPattern.sides.at(0), //
			thirdPattern.sides.at(1), //
			thirdPattern.inner,		  //

			sides[0].outer,		  //
			sides[0].sides.at(0), //
			sides[0].sides.at(1), //
			sides[0].inner,		  //

			sides[1].outer,		  //
			sides[1].sides.at(0), //
			sides[1].sides.at(1), //
			sides[1].inner,		  //
		};
		std::array<cv::Point2f, 12> dst2;

		cv::perspectiveTransform(src2, dst2, transform);
		// cv::perspectiveTransform(sides[0].sides.at(0), dst2[1], transform);
		std::vector<float> avgs;
		avgs.push_back(cv::norm(dst2[1] - dst2[0]));
		avgs.push_back(cv::norm(dst2[2] - dst2[0]));
		avgs.push_back(cv::norm(dst2[1] - dst2[3]));
		avgs.push_back(cv::norm(dst2[2] - dst2[3]));

		avgs.push_back(cv::norm(dst2[5] - dst2[4]));
		avgs.push_back(cv::norm(dst2[6] - dst2[4]));
		avgs.push_back(cv::norm(dst2[5] - dst2[7]));
		avgs.push_back(cv::norm(dst2[6] - dst2[7]));

		// float cellsize = (std::accumulate(avgs.begin(), avgs.end(), 0) / avgs.size()) / 7.0f;
		std::sort(avgs.begin(), avgs.end());
		cellsize = avgs.at(1 + avgs.size() / 2) / 7.0f;

		// auto cellsize	   = cv::norm(dst2[1] - dst2[0]) / 7.0f;
		halfCellSize = cellsize / 2.0f;
		printf("%f %f\n", cv::norm(dst2[1] - dst2[0]), cellsize);
	}

	void extractCells()
	{
		for (int y = 0; y < sizeInCells; y++)
		{
			cells.push_back(std::vector<bool>(sizeInCells));
			debug_cells.push_back(std::vector<char>(sizeInCells));

			for (int x = 0; x < sizeInCells; x++)
			{
				int pixelX		  = round(halfCellSize + static_cast<float>(x) * cellsize);
				int pixelY		  = round(halfCellSize + static_cast<float>(y) * cellsize);
				cells.at(y).at(x) = monochrome_image_warped.data[monochrome_image_warped.cols * pixelY + pixelX] < 127;
				debug_cells.at(y).at(x) = '-';

				cv::circle(monochrome_image_warped,
						   cv::Point(halfCellSize + static_cast<float>(x) * cellsize,
									 halfCellSize + static_cast<float>(y) * cellsize),
						   3, red);
			}
		}
	}

	void extractFormat()
	{
		// from http://www.cdsy.de/formatbereiche.html
		std::array<struct formatCode, 4 * 8> formats
			//
			{{{err_L, 0, 0b111011111000100}, //
			  {err_L, 1, 0b111001011110011}, //
			  {err_L, 2, 0b111110110101010}, //
			  {err_L, 3, 0b111100010011101}, //
			  {err_L, 4, 0b110011000101111}, //
			  {err_L, 5, 0b110001100011000}, //
			  {err_L, 6, 0b110110001000001}, //
			  {err_L, 7, 0b110100101110110}, //

			  {err_M, 0, 0b101010000010010}, //
			  {err_M, 1, 0b101000100100101}, //
			  {err_M, 2, 0b101111001111100}, //
			  {err_M, 3, 0b101101101001011}, //
			  {err_M, 4, 0b100010111111001}, //
			  {err_M, 5, 0b100000011001110}, //
			  {err_M, 6, 0b100111110010111}, //
			  {err_M, 7, 0b100101010100000}, //

			  {err_Q, 0, 0b011010101011111}, //
			  {err_Q, 1, 0b011000001101000}, //
			  {err_Q, 2, 0b011111100110001}, //
			  {err_Q, 3, 0b011101000000110}, //
			  {err_Q, 4, 0b010010010110100}, //
			  {err_Q, 5, 0b010000110000011}, //
			  {err_Q, 6, 0b010111011011010}, //
			  {err_Q, 7, 0b010101111101101}, //

			  {err_H, 0, 0b001011010001001}, //
			  {err_H, 1, 0b001001110111110}, //
			  {err_H, 2, 0b001110011100111}, //
			  {err_H, 3, 0b001100111010000}, //
			  {err_H, 4, 0b000011101100010}, //
			  {err_H, 5, 0b000001001010101}, //
			  {err_H, 6, 0b000110100001100}, //
			  {err_H, 7, 0b000100000111011}}};

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

		bin_prnt_byte(formatVal);

		int minimalDist		= 16;
		int suggestedFormat = -1;

		for (int i = 0; i < formats.size(); i++)
		{
			auto& f = formats.at(i);

			int distance = hammingDistance(f.code, formatVal);
			if (distance < minimalDist)
			{
				suggestedFormat	   = i;
				detectedFormatCode = f;
				minimalDist		   = distance;
			}
			printf("hamming distance %d %d %d\n", f.err, f.mask, distance);
		}

		assert(minimalDist < 2);
		printf("detected mask %d\n", detectedFormatCode.mask);
	}

	bool isCellOnTimingPattern(int x, int y)
	{
		return ((x == 6) || (y == 6));
	}

	bool isCellDataBit(int x, int y)
	{
		// Top right finder pattern with only one format area at the bottom
		if (x > sizeInCells - 9 && y < 9)
			return false;

		// Top left finder pattern with two format areas
		if (x < 9 && y < 9)
			return false;

		// Bottom left finder pattern
		if (x < 9 && y > sizeInCells - 9)
			return false;

		// Timing patterns
		if ((x == 6) || (y == 6))
			return false;

		// Alignment pattern
		if (x >= 16 && x < 16 + 5 && y >= 16 && y < 16 + 5)
			return false;

		return true;
	}

	bool goingUp   = true;
	bool changeRow = false;

	bool readBit()
	{
		int& x = analyzePosition.x;
		int& y = analyzePosition.y;

		bool gotDataBit = false;
		bool result;

		while (!gotDataBit)
		{
			if (isCellDataBit(x, y))
			{
				result = cells.at(y).at(x);
				// printf("Reading from %d %d  %d\n", x, y, result ? 1 : 0);
				debug_cells.at(y).at(x) = debug_cell_val;
				gotDataBit				= true;
			}
			else
			{
				// Do nothing
				// printf("Ignore from %d %d  %d\n", x, y, result ? 1 : 0);
			}

			if (changeRow)
			{
				if (goingUp)
				{
					if (y == 0)
					{
						x -= 2;
						goingUp = false;
						printf("Going down!\n");

						if (isCellOnTimingPattern(x + 1, y))
						{
							printf("Nudge left!");
							x--;
						}
					}
					else
						y--;
				}
				else
				{
					if (y == sizeInCells - 1)
					{
						x -= 2;
						goingUp = true;
						printf("Going up!\n");

						if (isCellOnTimingPattern(x + 1, y))
						{
							printf("Nudge left!");
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

			changeRow = !changeRow;
		}

		return result;
	}

	int readWord(int bitlen)
	{
		int result = 0;
		while (bitlen--)
		{
			result <<= 1;
			if (readBit())
				result |= 1;
		}

		printf("%d %x %c ", result, result, result);
		bin_prnt_byte(result);
		return result;
	}

  public:
	void decodeFromFile(const char* filepath)
	{
		finder_contour_points.clear();
		finder2.clear();

		src_image_ = cv::imread(filepath, 1);
		if (!src_image_.data)
		{
			printf("No image data \n");
			return;
		}

		cv::Mat src_image_as_grayscale, grayscale_blurred;
		cv::Mat monochrome_image_inverted;
		cv::Mat monochrome_image_normal;

		// Convert image to grayscale
		cv::cvtColor(src_image_, src_image_as_grayscale, cv::COLOR_BGR2GRAY);

		// Blur the grayscale image to filter out small artefacts
		cv::GaussianBlur(src_image_as_grayscale, grayscale_blurred, cv::Size(11, 11), 0, 0);

		// Convert grayscale to monochrome image
		cv::adaptiveThreshold(grayscale_blurred, monochrome_image_inverted, 255, cv::ADAPTIVE_THRESH_MEAN_C,
							  cv::THRESH_BINARY_INV, 131, 12);

		// Find Contours in the monochrome image
		cv::findContours(monochrome_image_inverted, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_TC89_KCOS);

		// Try to find the finder patterns in the contour data
		searchFinderPatternPoints(0, 0);

		if (finder_contour_points.size() != 3)
		{
			printf("No 3 finders\n");
			return;
		}

		// Find the average of all finder pattern points to check where the center of the code might be
		assumeFinderPatternCenter();

		reconstructShapeOfCode();

		cv::bitwise_not(monochrome_image_inverted, monochrome_image_normal);

		cv::warpPerspective(monochrome_image_normal, monochrome_image_warped, transform,
							cv::Size(warped_size, warped_size));

		// cv::warpPerspective(image, warped, transform, cv::Size(size, size));
		float sizeInCellsF = static_cast<float>(warped_size) / cellsize;

		sizeInCells = round(sizeInCellsF);
		printf("sizeInCells %d %f\n", sizeInCells, sizeInCellsF);
		// assert(sizeInCells < 100);

		extractCells();

#if 0
		cv::imshow("Display Image", monochrome_image_warped);
		cv::imshow("Display Image2", src_image_);
		cv::imwrite("monochrome_image_warped.png", monochrome_image_warped);

		cv::waitKey(0);
#endif

		extractFormat();
		// detectedFormatCode.mask = 2;

#if 0
		printf("----------------------\n");
		for (const auto& c : cells)
		{
			for (const auto& p : c)
			{
				printf("%d", p ? 1 : 0);
			}
			printf("\n");
		}

#endif

		applyMask(detectedFormatCode.mask);

		analyzePosition.x = sizeInCells - 1;
		analyzePosition.y = sizeInCells - 1;

#if 0
		debug_cell_val = 'M';
		int mode	   = readWord(4);
		assert(mode == 4); // TODO currently only byte mode
		debug_cell_val	  = 'L';
		int payload_len	  = readWord(8);
		int ecb_len		  = 10;
		int remainder_len = 32 - payload_len;

		std::vector<ExtFiniteField256> rsData;

		printf("Grab payload data of len %d\n", payload_len);
		debug_cell_val = 'a';
		while (payload_len--)
		{
			readWord(8);
			debug_cell_val++;
		}

		int terminator = readWord(4);
		assert(terminator == 0);

		printf("Grab remaining padding data of len %d\n", remainder_len);
		debug_cell_val = '0';

		while (remainder_len--)
		{
			readWord(8);
			debug_cell_val++;
		}

		printf("Grab ecb data of len %d\n", ecb_len);
		debug_cell_val = 'X';

		while (ecb_len--)
		{
			readWord(8);
		}
#endif

		std::vector<ExtFiniteField256> rsData;
		int raw_len	   = 44;
		debug_cell_val = 'A';
		while (raw_len--)
		{
			int data = readWord(8);
			rsData.push_back(data);

			if (debug_cell_val == 'Z')
				debug_cell_val = 'a';
			else
				debug_cell_val++;
		}

		printf("size : %d\n", rsData.size());
		for (auto& v : rsData)
		{
			printf("%d ", v);
		}
		printf("\n");

#if 1
		// http://www.cdsy.de/QR-Vorgaben.html
		RS::ReedSolomon<ExtFiniteField256> rs(44, 34);
		std::reverse(rsData.begin(), rsData.end());

		auto rsPolynomial = Polynom<ExtFiniteField256>(rsData);
		auto syndromes	  = rs.calculateSyndromes(rsPolynomial);
		for (auto& s : syndromes.first)
		{
			std::cout << "Syndrome " << s << std::endl;
		}

		auto decoded_message = rs.decode(rsPolynomial);
		std::cout << decoded_message.asString() << std::endl;

		auto raw_decoded_data_extfield = decoded_message.getRawData();
		std::vector<uint8_t> raw_decoded_data;

		// std::reverse(raw_decoded_data.begin(), raw_decoded_data.end());
		for (auto it = std::rbegin(raw_decoded_data_extfield); it != std::rend(raw_decoded_data_extfield); ++it)
		{
			raw_decoded_data.push_back(*it);
		}

		printf("size : %d\n", raw_decoded_data.size());
		for (auto& v : raw_decoded_data)
		{
			printf("%d ", v);
		}
		printf("\n");

		BitStreamReader bitstream(raw_decoded_data);

		int mode = bitstream.readWord(4);
		printf("Mode: %d\n", mode);
		assert(mode == 4); // TODO currently only byte mode
		debug_cell_val	= 'L';
		int payload_len = bitstream.readWord(8);
		printf("Payload Len: %d\n", payload_len);
		int ecb_len		  = 10;
		int remainder_len = 32 - payload_len;

		std::vector<char> payload_data;

		printf("Payload: ");
		while (payload_len--)
		{
			uint8_t data = bitstream.readWord(8);
			payload_data.push_back(data);
			printf("%c", data);
			debug_cell_val++;
		}
		printf("\n");
		mode = bitstream.readWord(4);
		printf("Mode: %d\n", mode);
		assert(mode == 0);

		printf("Remainder: ");
		while (remainder_len--)
		{
			uint8_t data = bitstream.readWord(8);
			printf(" %d", data);
			debug_cell_val++;
		}
		printf("\n");
		printf("ECB:");
		while (ecb_len--)
		{
			uint8_t data = bitstream.readWord(8);
			printf(" %d", data);
		}
		printf("\n");

		printf("%d\n", bitstream.remainingBits());
		assert(bitstream.isEof());
#endif

#if 0
		for (const auto& c : debug_cells)
		{
			for (const auto& p : c)
			{
				printf("%c", p);
			}
			printf("\n");
		}
#endif

#if 0
		for (const auto& c : contours)
		{
			for (const auto& p : c)
			{
				printf("", p.x, p.y);
			}
		}
#endif

#if 0
		for (int y = 0; y < sizeInCells; y++)
		{
			for (int x = 0; x < sizeInCells; x++)
			{
				printf("%d", isCellDataBit(x,y) ? 1 : 0);
			}
			printf("\n");
		}
#endif

#if 0
		// cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
		cv::imshow("Display Image", monochrome_image_warped);
		cv::imshow("Display Image2", src_image_);
		cv::imwrite("monochrome_image_warped.png", monochrome_image_warped);

		cv::waitKey(0);
#endif
	}
};

#endif /* QRDECODER_H_ */
