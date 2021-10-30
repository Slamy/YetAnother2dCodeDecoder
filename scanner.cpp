/*
 * scanner.cpp
 *
 *  Created on: 28.10.2021
 *      Author: andre
 */

#include "Matrix.h"
#include <algorithm>
#include <glm/vec2.hpp>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <vector>

cv::Mat image;

bool compare(int a, int b)
{
	int percent = a * 100 / b;
	return labs(100 - percent) < 12;
}

void parseLine(uchar* line, int w, int y)
{
	std::vector<int> lens;

	w--;
	int segmentLen = 0;

	// printf("Y %d\n", y);
	while (w--)
	{
		segmentLen++;
		if (line[0] != line[1])
		{
			// printf("%d\n", segmentLen);
			lens.push_back(segmentLen);
			segmentLen = 0;
		}
		line++;
	}

	if (lens.size() < 5)
	{
		return;
	}

	int startX = 0;
	for (int i = 0; i < lens.size() - 5; i++)
	{
		// 11311
		if (compare(lens.at(i + 1), lens.at(i)) && //
			compare(lens.at(i + 3), lens.at(i)) && //
			compare(lens.at(i + 4), lens.at(i)) && //
			compare(lens.at(i + 2), lens.at(i) * 3))
		{
			int endX	 = startX + lens.at(i) + lens.at(i + 1) + lens.at(i + 2) + lens.at(i + 3) + lens.at(i + 4);
			int cell	 = (endX - startX) / (4 + 3);
			int halfCell = cell / 2;
			if (line[startX + halfCell] == 0 &&			 //
				line[startX + halfCell + cell] == 255 && //
				line[startX + halfCell + cell * 2] == 0)
			{
				printf("Starting at %d %d %d  %d\n", startX, endX, y, i);

				cv::line(image, cv::Point(startX, y), cv::Point(endX, y), cv::Scalar(0, 0, 255), 3, cv::FILLED);
			}
		}
		startX += lens.at(i);
	}
}

std::vector<std::vector<cv::Point>> contours;
std::vector<cv::Vec4i> hierarchy;

std::vector<std::vector<cv::Point>> finders;

void drawContourHierarchy(int id, int candidateForFinderPattern)
{
	// cv::Scalar colour((rand() & 255), (rand() & 255), (rand() & 255));

	std::vector<cv::Scalar> colors = {
		{0, 0, 255},
		{0, 255, 0},
		{255, 0, 0},
		{255, 0, 255},
	};
	cv::Scalar squarecolour(0, 0, 255);

	cv::Scalar colour(0, 255, 0);

	for (;;)
	{
		// double epsilon = cv::arcLength(contours.at(id), true);
		std::vector<cv::Point> out;
		cv::approxPolyDP(contours.at(id), out, 6, true);

#if 0
		for (auto& v : out)
		{
			cv::circle(image, v, 10, colour);
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
				finders.push_back(out2);
			}

			// cv::polylines(image, out, true, colors.at(0), 10, cv::LINE_AA);

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
			drawContourHierarchy(hierarchy.at(id)[2], nextparam);
		}

		id = hierarchy.at(id)[0];
		if (id == -1)
			break;
	}
}

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
/*
 * https://stackoverflow.com/questions/10344246/how-can-i-convert-a-cvmat-to-a-gray-scale-in-opencv
 * https://stackoverflow.com/questions/7899108/opencv-get-pixel-channel-value-from-mat-image
 */
int main(int argc, char** argv)
{
	// image = cv::imread("/home/andre/GIT/QrCode/Japan-qr-code-billboard.jpg", 1);
	// image = cv::imread("/home/andre/GIT/QrCode/QR_deWP.svg.png", 1);
	image = cv::imread("/home/andre/GIT/QrCode/QR_deWP.svg2.png", 1);
	if (!image.data)
	{
		printf("No image data \n");
		return -1;
	}
	cv::Mat greyMat, greyMat2;
	cv::Mat bwMat, bwMat2;
	cv::cvtColor(image, greyMat, cv::COLOR_BGR2GRAY);

	cv::GaussianBlur(greyMat, greyMat2, cv::Size(11, 11), 0, 0);

	cv::adaptiveThreshold(greyMat2, bwMat, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 131, 12);

#if 0
	int erosion_size=1;
cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
		cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
		cv::Point( erosion_size, erosion_size ) );

	cv::erode(bwMat, bwMat2, element);
#endif

	for (int y = 10; y < bwMat.rows; y += 10)
		parseLine(&bwMat.data[bwMat.cols * y], bwMat.cols, y);

	cv::findContours(bwMat, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_TC89_KCOS);
	// cv::polylines(image, contours, true, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);

	drawContourHierarchy(0, 0);

	cv::Scalar green(0, 255, 0);
	cv::Scalar red(0, 0, 255);
	cv::Scalar blue(255, 0, 0);

	cv::Point avg(0, 0);
	for (const auto& c : finders)
	{
		cv::polylines(image, c, true, green, 1, cv::LINE_AA);

		for (const auto& p : c)
		{
			avg.x += p.x;
			avg.y += p.y;
		}
	}

	avg.x /= (finders.size() * 4);
	avg.y /= (finders.size() * 4);

	// cv::circle(image, avg, 10, red, 3);
	std::vector<FinderPattern> finder2;

	for (const auto& c : finders)
	{
		finder2.emplace_back(c, avg);
		// FinderPattern pattern(c, avg);
		// cv::circle(image, pattern.inner, 10, green, 3);
		// cv::circle(image, pattern.outer, 10, red, 3);
		// cv::circle(image, pattern.sides[0], 10, blue, 3);
		// cv::circle(image, pattern.sides[1], 10, blue, 3);

		// cv::circle(image, pattern.inner + pattern.out_to_in, 20, blue, 3);
	}

	int dot01 = labs(finder2.at(0).out_to_in.dot(finder2.at(1).out_to_in));
	int dot02 = labs(finder2.at(0).out_to_in.dot(finder2.at(2).out_to_in));
	int dot12 = labs(finder2.at(1).out_to_in.dot(finder2.at(2).out_to_in));

	printf("%d %d %d\n", dot01, dot02, dot12);
	// cv::circle(image, finder2.at(1).outer, 10, red, 3);
	FinderPattern thirdPattern = finder2.at(1); // TODO

	FinderPattern sides[2]{finder2.at(0), finder2.at(2)};

	sides[0].calculateFinderLessEdgeAngle(thirdPattern);
	sides[1].calculateFinderLessEdgeAngle(thirdPattern);

	// cv::circle(image, sides[0].outer, 10, red, 3);
	// cv::circle(image, sides[1].outer, 10, red, 3);

	// cv::circle(image, sides[0].outer + sides[0].outer_to_finderless_edge, 10, green, 3);
	// cv::circle(image, sides[1].outer + sides[1].outer_to_finderless_edge, 10, green, 3);

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
	printf("%d %d\n", solution.x, solution.y);

	cv::circle(image, solution, 5, red, 3);

	cv::Mat warped;
	cv::Mat transform;
	cv::Point2f src[4];
	cv::Point2f dst[4];

	src[0] = thirdPattern.outer;
	src[1] = sides[0].outer;
	src[2] = solution;
	src[3] = sides[1].outer;

	int size = 1200;
	dst[0]	 = cv::Point2f(0, 0);
	dst[1]	 = cv::Point2f(size, 0);
	dst[2]	 = cv::Point2f(size, size);
	dst[3]	 = cv::Point2f(0, size);

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
	float cellsize = avgs.at(1 + avgs.size() / 2) / 7.0f;

	// auto cellsize	   = cv::norm(dst2[1] - dst2[0]) / 7.0f;
	float halfCellSize = cellsize / 2.0f;
	printf("%f %f\n", cv::norm(dst2[1] - dst2[0]), cellsize);
	cv::bitwise_not(bwMat, bwMat2);

	cv::warpPerspective(bwMat2, warped, transform, cv::Size(size, size));
	// cv::warpPerspective(image, warped, transform, cv::Size(size, size));
	float sizeInCellsF = static_cast<float>(size) / cellsize;

	int sizeInCells = round(sizeInCellsF);
	printf("sizeInCells %d %f\n", sizeInCells, sizeInCellsF);
	// assert(sizeInCells < 100);

	std::vector<std::vector<bool>> cells;

	for (int y = 0; y < sizeInCells; y++)
	{
		cells.push_back(std::vector<bool>(sizeInCells));

		for (int x = 0; x < sizeInCells; x++)
		{
			int pixelX		  = round(halfCellSize + static_cast<float>(x) * cellsize);
			int pixelY		  = round(halfCellSize + static_cast<float>(y) * cellsize);
			cells.at(y).at(x) = warped.data[warped.cols * pixelY + pixelX] > 127;

			cv::circle(warped,
					   cv::Point(halfCellSize + static_cast<float>(x) * cellsize,
								 halfCellSize + static_cast<float>(y) * cellsize),
					   3, red);
		}
	}

	for (const auto& c : cells)
	{
		for (const auto& p : c)
		{
			printf("%d", p ? 1 : 0);
		}
		printf("\n");
	}

#if 0
	for (const auto& c : contours)
	{
		for (const auto& p : c)
		{
			printf("", p.x, p.y);
		}
	}
#endif

#if 1
	// cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
	cv::imshow("Display Image", warped);
	cv::imshow("Display Image2", image);
	cv::imwrite("out.png", warped);

	cv::waitKey(0);
#endif

	return 0;
}
