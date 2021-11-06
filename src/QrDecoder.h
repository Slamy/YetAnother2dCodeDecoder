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
#include "glm/gtx/vector_angle.hpp"
#include "glm/vec2.hpp"
#include "util.h"
#include <numeric>
#include <opencv2/opencv.hpp>

class QrCodeException : public std::exception
{
  public:
	enum class Cause
	{
		kFileNotReadable,
		kNotEnoughFinderPattern,
		kVersionNotSupported,
		kFormatNotDetected
	};

	const char* whatStr[4] = {
		"File not readable",
		"Unable to detect Finder Pattern",
		"Qr Code Size / Version not supported",
		"Format not detected",
	};
	const Cause cause_;

	QrCodeException(Cause c) : cause_(c)
	{
	}

	const char* what() const noexcept override
	{
		return whatStr[static_cast<int>(cause_)];
	}
};

class FinderPattern
{
  public:
	cv::Point outer;
	cv::Point inner;
	glm::vec2 out_to_in;
	std::vector<cv::Point> sides{2};
	cv::Point outer_to_finderless_edge;

	FinderPattern()
	{
	}

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
	void reset()
	{
		contours.clear();
		hierarchy.clear();
		finder_contour_points.clear();
		alignment_pattern_centers.clear();
		finder2.clear();

		cells.clear();
		debug_cells.clear();

		going_up_	= true;
		change_row_ = false;
	}

	const cv::Scalar green{0, 255, 0};
	const cv::Scalar black{0, 0, 0};
	const cv::Scalar blue{255, 0, 0};
	const cv::Scalar red{0, 0, 255};
	const cv::Scalar white{255, 255, 255};

	cv::Mat src_image_;

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;

	std::vector<std::vector<cv::Point>> finder_contour_points;
	std::vector<cv::Point> alignment_pattern_centers;

	cv::Point average_center_pos;
	std::vector<FinderPattern> finder2;

	static constexpr int warped_size = 1000;
	float cellsizeX, cellsizeY;
	float half_cell_size_;
	cv::Mat transform;

	std::vector<std::vector<bool>> cells;
	std::vector<std::vector<char>> debug_cells;
	char debug_cell_val = '_';

	int size_in_cells_;

	cv::Mat monochrome_image_warped;
	cv::Point current_cell_position_;

	const char* correctionLevelStr[4] = {"M", "L", "H", "Q"};

	static constexpr int kErrM = 0;
	static constexpr int kErrL = 1;
	static constexpr int kErrH = 2;
	static constexpr int kErrQ = 3;

	float monochrome_image_inverted_area;

	bool going_up_	 = true;
	bool change_row_ = false;

	int db_blocks_;
	std::vector<std::vector<int>> db_deinterleaving_lut_;
	int number_of_Db_;

	int raw_db_plus_ecb;

	struct FormatCode
	{
		int err;
		int mask;
		uint32_t code;
	};

	struct FormatCode detected_format_code_;

	struct QrConfig
	{
		int version;
		int correctionLevel;
		int ecbPerBlock;
		int blocksInGroup1;
		int dbPerBlockInGroup1;
		int blocksInGroup2;
		int dbPerBlockInGroup2;
	};

	struct QrConfig detected_qr_config_;

	// http://www.cdsy.de/QR-Vorgaben.html
	// TODO still a lot configs missing
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
		throw QrCodeException(QrCodeException::Cause::kVersionNotSupported);
	}

	static cv::Point calculateLineIntercross(cv::Point a_pos, cv::Point a_dir, cv::Point b_pos, cv::Point b_dir)
	{
		Matrix<float> linearsolver{
			{static_cast<float>(a_dir.x), static_cast<float>(-b_dir.x), static_cast<float>(b_pos.x - a_pos.x)},
			{static_cast<float>(a_dir.y), static_cast<float>(-b_dir.y), static_cast<float>(b_pos.y - a_pos.y)},
		};

		auto coefficients = linearsolver.gauss_jordan_elim();
		assert(coefficients.size() > 0);

		// linearsolver.print();
		// printf("calculateLineIntercross %d %d %f %d %d\n", a_pos.x, a_pos.y, coefficients.at(0), a_dir.x, a_dir.y);

		cv::Point solution = a_pos + coefficients.at(0) * a_dir;

		return solution;
	}

	bool isKindaSquare(std::vector<cv::Point> out2)
	{
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

		return longEdges == 4;
	}

	std::vector<cv::Point> reconstructSquare(std::vector<cv::Point> inp)
	{
		std::vector<cv::Point> out2;
		cv::approxPolyDP(inp, out2, 3, true);

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

	void searchFinderPatternPoints(int id, int candidateForFinderPattern)
	{

		for (;;)
		{
			double epsilon = 0.05 * cv::arcLength(contours.at(id), true);
			std::vector<cv::Point> out;
			// printf("epsilon %lf\n", epsilon);
			cv::approxPolyDP(contours.at(id), out, epsilon, true);

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
					printf("Contour %d %d %d %d %lf\n", id, contours.at(id).size(), out.size(),
						   candidateForFinderPattern, area);
				}

				if (candidateForFinderPattern == 2)
				{
					int outer1 = hierarchy.at(id)[3];
					int outer2 = hierarchy.at(outer1)[3];
					std::vector<cv::Point> out2;

					assert(contours.at(outer2).size() >= 4);

					cv::approxPolyDP(contours.at(outer2), out2, 3, true);

					// cv::polylines(src_image_, contours.at(outer2), true, red, 5, cv::LINE_AA);
					// cv::polylines(src_image_, contours.at(outer1), true, red, 5, cv::LINE_AA);

					if (out2.size() == 4)
					{
						if (debugMode)
						{
							printf("Direct use!\n");
						}
						finder_contour_points.push_back(out2);
					}
					else
					{
						if (debugMode)
						{
							printf("reconstructSquare!\n");
						}
						auto square = reconstructSquare(contours.at(outer2));

						if (square.size() == 4)
						{
							finder_contour_points.push_back(square);
						}
					}
				}
				else if (candidateForFinderPattern == 1)
				{

					int outer1 = hierarchy.at(id)[3];
					int inner  = hierarchy.at(id)[2];

					if (inner == -1)
					{
						std::vector<cv::Point> out2;
						double epsilon = 0.02 * cv::arcLength(contours.at(outer1), true);
						cv::approxPolyDP(contours.at(outer1), out2, epsilon, true);

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
							alignment_pattern_centers.push_back(center);
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

			if (hierarchy.at(id)[2] >= 0)
			{
				searchFinderPatternPoints(hierarchy.at(id)[2], nextparam);
			}

			// Next contour on this level
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

	void assumeFinderPatternCenter()
	{
		average_center_pos.x = 0;
		average_center_pos.y = 0;

		for (const auto& c : finder_contour_points)
		{
			assert(c.size() == 4);
			// cv::polylines(src_image_, c, true, green, 1, cv::LINE_AA);

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

		if (debugMode)
		{
			printf("%f %f\n", finder2.at(2).out_to_in.x, finder2.at(2).out_to_in.y);
			printf("%f %f\n", finder2.at(0).out_to_in.x, finder2.at(0).out_to_in.y);
			printf("%f %f\n", finder2.at(1).out_to_in.x, finder2.at(1).out_to_in.y);
		}
		float angle01, angle02, angle12;

		angle01 = glm::orientedAngle(finder2.at(0).out_to_in, finder2.at(1).out_to_in);
		angle02 = glm::orientedAngle(finder2.at(0).out_to_in, finder2.at(2).out_to_in);
		angle12 = glm::orientedAngle(finder2.at(1).out_to_in, finder2.at(2).out_to_in);

		if (debugMode)
		{
			printf("%f %f %f\n", angle01, angle02, angle12);
		}

		FinderPattern finder_topLeft;
		FinderPattern finder_bottomLeft;
		FinderPattern finder_topRight;

		if (fabs(angle01) > 3)
		{
			finder_topLeft = finder2.at(2);
			if (angle02 > 0)
			{
				finder_bottomLeft = finder2.at(0);
				finder_topRight	  = finder2.at(1);
			}
			else
			{
				finder_bottomLeft = finder2.at(1);
				finder_topRight	  = finder2.at(0);
			}
		}
		else if (fabs(angle02) > 3)
		{
			finder_topLeft = finder2.at(1);
			if (angle01 > 0)
			{
				finder_bottomLeft = finder2.at(0);
				finder_topRight	  = finder2.at(2);
			}
			else
			{
				finder_bottomLeft = finder2.at(2);
				finder_topRight	  = finder2.at(0);
			}
		}
		else if (fabs(angle12) > 3)
		{
			finder_topLeft = finder2.at(0);
			if (angle01 < 0)
			{
				finder_bottomLeft = finder2.at(1);
				finder_topRight	  = finder2.at(2);
			}
			else
			{
				finder_bottomLeft = finder2.at(2);
				finder_topRight	  = finder2.at(1);
			}
			// TODO must still be developed
		}
		else
		{
			assert(0);
		}

		printf("finder top left %d %d\n", finder_topLeft.outer.x, finder_topLeft.outer.y);
		printf("finder top right %d %d\n", finder_topRight.outer.x, finder_topRight.outer.y);
		printf("finder bottom left %d %d\n", finder_bottomLeft.outer.x, finder_bottomLeft.outer.y);
		// cv::circle(image, finder2.at(1).outer, 10, red, 3);

		finder_bottomLeft.calculateFinderLessEdgeAngle(finder_topLeft);
		finder_topRight.calculateFinderLessEdgeAngle(finder_topLeft);

#if 0
		cv::circle(src_image_, finder_bottomLeft.outer, 10, blue, 3);
		cv::circle(src_image_, finder_bottomLeft.outer + finder_bottomLeft.outer_to_finderless_edge, 10, green, 3);

		cv::circle(src_image_, finder_topRight.outer, 10, red, 3);
		cv::circle(src_image_, finder_topRight.outer + finder_topRight.outer_to_finderless_edge, 10, green, 3);

		cv::circle(src_image_, finder_topLeft.outer, 10, red, 3);
		cv::circle(src_image_, finder_topLeft.inner, 10, green, 3);
#endif

		cv::Point solution;
		cv::Point avg_solution(0, 0);
		float avg_num = 0;

		solution = calculateLineIntercross(finder_bottomLeft.outer, finder_bottomLeft.outer_to_finderless_edge,
										   finder_topRight.outer, finder_topRight.outer_to_finderless_edge);
		avg_solution += solution;

		if (debugMode)
		{
			printf("edge of code %d %d\n", solution.x, solution.y);
			cv::circle(src_image_, solution, 5, red, 1);
		}
		avg_num++;

		if (alignment_pattern_centers.size() > 0)
		{
			solution =
				calculateLineIntercross(finder_topLeft.outer, alignment_pattern_centers.at(0) - finder_topLeft.outer,
										finder_topRight.outer, finder_topRight.outer_to_finderless_edge);
			avg_solution += solution;
			if (debugMode)
			{
				printf("edge of code %d %d\n", solution.x, solution.y);
				cv::circle(src_image_, solution, 5, red, 1);
			}
			avg_num++;

			solution =
				calculateLineIntercross(finder_bottomLeft.outer, finder_bottomLeft.outer_to_finderless_edge,
										finder_topLeft.outer, alignment_pattern_centers.at(0) - finder_topLeft.outer);
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

		cellsizeX = (std::accumulate(avgs.begin(), avgs.end(), 0) / avgs.size()) / 7.0f;
		cellsizeY = cellsizeX;
		// std::sort(avgs.begin(), avgs.end());
		// cellsize = avgs.at(avgs.size() / 2) / 7.0f;

		// auto cellsize	   = cv::norm(dst2[1] - dst2[0]) / 7.0f;
		half_cell_size_ = cellsizeX / 2.0f;
		if (debugMode)
		{
			printf("cellsize %f %f\n", cv::norm(dst2[1] - dst2[0]), cellsizeX);
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
			int diff  = (dist.first - dist.second) / 2;
			float x	  = round((pixelX - half_cell_size_) / cellsizeX);
			cellsizeX = (pixelX - diff - half_cell_size_) / x;
		}
	}

	void optimizeCellsizeWithTimingPattern()
	{
		int pixelX;
		int pixelY;
		int cellX;
		int cellY;

		int timing_pattern_black_cells = (size_in_cells_ - 7 * 2) / 2;

		if (debugMode)
			printf("cellsize at start %f %f timing_pattern_black_cells %d\n", cellsizeX, cellsizeY,
				   timing_pattern_black_cells);

		for (int i = 0; i < timing_pattern_black_cells; i++)
		{
			// pixelX = halfCellSize + cellX * cellsize
			// pixelX - halfCellSize = cellX * cellsize
			// (pixelX - halfCellSize) / cellsize = cellX

			cellX  = 6;
			cellY  = 8 + 2 * i;
			pixelX = round(half_cell_size_ + static_cast<float>(cellX) * cellsizeX);
			pixelY = round(half_cell_size_ + static_cast<float>(cellY) * cellsizeY);
			optimizeCellsizeWithEnclosedCell(pixelX, pixelY, true);

			pixelX = round(half_cell_size_ + static_cast<float>(cellX) * cellsizeX);
			pixelY = round(half_cell_size_ + static_cast<float>(cellY) * cellsizeY);

			auto dist = verticalDistance(monochrome_image_warped, pixelX, pixelY);
			if (debugMode)
				printf("Vertical distance After %d %d\n", dist.first, dist.second);

			cellX  = 8 + 2 * i;
			cellY  = 6;
			pixelX = round(half_cell_size_ + static_cast<float>(cellX) * cellsizeX);
			pixelY = round(half_cell_size_ + static_cast<float>(cellY) * cellsizeY);
			optimizeCellsizeWithEnclosedCell(pixelX, pixelY, false);
		}

		if (alignment_pattern_centers.size() > 0)
		{
			std::array<cv::Point2f, 1> x;
			x[0].x = alignment_pattern_centers.at(0).x;
			x[0].y = alignment_pattern_centers.at(0).y;
			std::array<cv::Point2f, 1> y;
			cv::perspectiveTransform(x, y, transform);
			optimizeCellsizeWithEnclosedCell(y[0].x, y[0].y, false);

			if (debugMode)
				printf("Alignment Center at %f %f\n", y[0].x, y[0].y);
		}

		if (debugMode)
			printf("cellsize at end %f %f\n", cellsizeX, cellsizeY);
	}

	cv::Point topLeftNudge;
	cv::Point topRightNudge;
	cv::Point bottomLeftNudge;
	cv::Point bottomRightNudge;

	cv::Point topLeftCenterRef;
	cv::Point topRightCenterRef;
	cv::Point bottomLeftCenterRef;
	cv::Point bottomRightCenterRef;

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
		pixelX = round(half_cell_size_ + static_cast<float>(cellX) * cellsizeX);
		pixelY = round(half_cell_size_ + static_cast<float>(cellY) * cellsizeY);
		vdist  = verticalDistance(monochrome_image_warped, pixelX, pixelY);
		hdist  = horizontalDistance(monochrome_image_warped, pixelX, pixelY);
		cv::circle(monochrome_image_warped, cv::Point(pixelX, pixelY), 6, white);

		topLeftCenterRef.x = pixelX;
		topLeftCenterRef.y = pixelY;
		topLeftNudge.x	   = -(hdist.first - hdist.second) / 2;
		topLeftNudge.y	   = -(vdist.first - vdist.second) / 2;

		cv::circle(monochrome_image_warped, cv::Point(pixelX, pixelY) + topLeftNudge, 6, white);

		// Top right
		cellX = size_in_cells_ - 4;
		cellY = 3;

		pixelX = round(half_cell_size_ + static_cast<float>(cellX) * cellsizeX);
		pixelY = round(half_cell_size_ + static_cast<float>(cellY) * cellsizeY);
		vdist  = verticalDistance(monochrome_image_warped, pixelX, pixelY);
		hdist  = horizontalDistance(monochrome_image_warped, pixelX, pixelY);
		cv::circle(monochrome_image_warped, cv::Point(pixelX, pixelY), 6, white);

		topRightCenterRef.x = pixelX;
		topRightCenterRef.y = pixelY;
		topRightNudge.x		= -(hdist.first - hdist.second) / 2;
		topRightNudge.y		= -(vdist.first - vdist.second) / 2;

		cv::circle(monochrome_image_warped, cv::Point(pixelX, pixelY) + topRightNudge, 6, white);

		// Bottom left
		cellX = 3;
		cellY = size_in_cells_ - 4;

		pixelX = round(half_cell_size_ + static_cast<float>(cellX) * cellsizeX);
		pixelY = round(half_cell_size_ + static_cast<float>(cellY) * cellsizeY);
		vdist  = verticalDistance(monochrome_image_warped, pixelX, pixelY);
		hdist  = horizontalDistance(monochrome_image_warped, pixelX, pixelY);
		cv::circle(monochrome_image_warped, cv::Point(pixelX, pixelY), 6, white);

		bottomLeftCenterRef.x = pixelX;
		bottomLeftCenterRef.y = pixelY;
		bottomLeftNudge.x	  = -(hdist.first - hdist.second) / 2;
		bottomLeftNudge.y	  = -(vdist.first - vdist.second) / 2;

		cv::circle(monochrome_image_warped, cv::Point(pixelX, pixelY) + bottomLeftNudge, 6, white);

		// Alignment Pattern - Bottom Right
		cellX = size_in_cells_ - 7;
		cellY = size_in_cells_ - 7;

		pixelX = round(half_cell_size_ + static_cast<float>(cellX) * cellsizeX);
		pixelY = round(half_cell_size_ + static_cast<float>(cellY) * cellsizeY);

		bottomRightCenterRef.x = pixelX;
		bottomRightCenterRef.y = pixelY;

		if (alignment_pattern_centers.size() > 0)
		{
			vdist = verticalDistance(monochrome_image_warped, pixelX, pixelY);
			hdist = horizontalDistance(monochrome_image_warped, pixelX, pixelY);
			cv::circle(monochrome_image_warped, cv::Point(pixelX, pixelY), 6, white);

			bottomRightNudge.x = -(hdist.first - hdist.second) / 2;
			bottomRightNudge.y = -(vdist.first - vdist.second) / 2;

			cv::circle(monochrome_image_warped, cv::Point(pixelX, pixelY) + bottomRightNudge, 6, white);
		}
		else
		{
			bottomRightNudge.x = (topRightNudge.x + bottomLeftNudge.x) / 2;
			bottomRightNudge.y = (topRightNudge.y + bottomLeftNudge.y) / 2;
		}
	}

	float linearInterpolation(float x1, float f_x1, float x2, float f_x2, float x)
	{
		float result = (x - x1) / (x2 - x1) * f_x2 + (x2 - x) / (x2 - x1) * f_x1;
		return result;
	}

	cv::Point getCellPosition(int x, int y, bool fineNudge)
	{
		// Coarse
		cv::Point point;
		cv::Point nudge(0, 0);
		point.x = round(half_cell_size_ + static_cast<float>(x) * cellsizeX);
		point.y = round(half_cell_size_ + static_cast<float>(y) * cellsizeY);

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

	void extractCells()
	{
		for (int y = 0; y < size_in_cells_; y++)
		{
			cells.push_back(std::vector<bool>(size_in_cells_));
			debug_cells.push_back(std::vector<char>(size_in_cells_));

			for (int x = 0; x < size_in_cells_; x++)
			{
				auto p					= getCellPosition(x, y, true);
				auto pNoNudge			= getCellPosition(x, y, false);
				cells.at(y).at(x)		= monochrome_image_warped.data[monochrome_image_warped.cols * p.y + p.x] < 127;
				debug_cells.at(y).at(x) = '-';

				cv::circle(monochrome_image_warped, p, 3, cells.at(y).at(x) ? white : black);
				cv::circle(monochrome_image_warped, pNoNudge, 1, cells.at(y).at(x) ? white : black);
			}
		}
	}

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
			throw QrCodeException(QrCodeException::Cause::kFormatNotDetected);
		}

		if (debugMode)
		{
			printf("detected mask %d, detected correction level %s\n", detected_format_code_.mask,
				   correctionLevelStr[detected_format_code_.err]);
		}
	}

	bool isCellOnTimingPattern(int x, int y)
	{
		return ((x == 6) || (y == 6));
	}

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
				debug_cells.at(y).at(x) = debug_cell_val;
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
					debug_cell_val++;
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

	std::vector<uint8_t> deinterleaveAndForwardErrorCorrect(std::vector<uint8_t> rawData)
	{
		std::vector<uint8_t> raw_decoded_data;

		for (int block = 0; block < db_blocks_; block++)
		{
			std::vector<ExtFiniteField256> rsData;

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

			RS::ReedSolomon<ExtFiniteField256> rs(n, k);
			std::reverse(rsData.begin(), rsData.end());

			auto rsPolynomial		= Polynom<ExtFiniteField256>(rsData);
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

			// std::reverse(raw_decoded_data.begin(), raw_decoded_data.end());
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
	std::array<char, 45> alphanumeric_lut = {
		'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E',
		'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T',
		'U', 'V', 'W', 'X', 'Y', 'Z', ' ', '$', '%', '*', '+', '-', '.', '/', ':',
	};

	bool debugMode{false};
	bool debugModeVisual{false};

	bool code_needed_correction_{false};

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
			throw QrCodeException(QrCodeException::Cause::kFileNotReadable);
		}

		cv::Mat src_image_as_grayscale, grayscale_blurred;
		cv::Mat monochrome_image_inverted;
		cv::Mat monochrome_image_normal;

		// Convert image to grayscale
		cv::cvtColor(src_image_, src_image_as_grayscale, cv::COLOR_BGR2GRAY);

		// Blur the grayscale image to filter out small artefacts
		if (src_image_as_grayscale.cols < 500 || src_image_as_grayscale.rows < 500)
			cv::resize(src_image_as_grayscale, grayscale_blurred, cv::Size(0, 0), 3, 3);
		else
			cv::GaussianBlur(src_image_as_grayscale, grayscale_blurred, cv::Size(11, 11), 0, 0);

		// Convert grayscale to monochrome image
		cv::adaptiveThreshold(grayscale_blurred, monochrome_image_inverted, 255, cv::ADAPTIVE_THRESH_MEAN_C,
							  cv::THRESH_BINARY_INV, 131, 0);

		// Find Contours in the monochrome image
		cv::findContours(monochrome_image_inverted, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_TC89_KCOS);

		monochrome_image_inverted_area = monochrome_image_inverted.cols * monochrome_image_inverted.rows;

		// Try to find the finder patterns in the contour data
		searchFinderPatternPoints(0, 0);

		if (finder_contour_points.size() != 3)
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

			throw QrCodeException(QrCodeException::Cause::kNotEnoughFinderPattern);
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

		cv::warpPerspective(monochrome_image_normal, monochrome_image_warped, transform,
							cv::Size(warped_size, warped_size));

		if (debugModeVisual)
		{
			cv::imshow("src_image_as_grayscale", src_image_as_grayscale);
			cv::imshow("grayscale_blurred", grayscale_blurred);
			cv::imshow("monochrome_image_inverted", monochrome_image_inverted);
			cv::imshow("src_image_", src_image_);

			cv::waitKey(0);
		}

		float cellsizeAvg  = (cellsizeX + cellsizeY) / 2;
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
			cv::imshow("monochrome_image_warped", monochrome_image_warped);
			cv::imwrite("monochrome_image_warped.png", monochrome_image_warped);

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
		debug_cell_val		= 'A';

		while (raw_len_to_read--)
		{
			int data = readWordFromCells(8);
			rawData.push_back(data);

			if (debug_cell_val == 'Z')
				debug_cell_val = 'a';
			else
				debug_cell_val++;
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
