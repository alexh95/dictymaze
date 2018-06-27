#include "dictymaze.h"
#include "dictymaze_math.cpp"
#include "dictymaze_opencv.cpp"

void
StringCopy(char* Dst, char* Src, u32 Count)
{
	for (u32 Index = 0; Index < Count; ++Index)
	{
		Dst[Index] = Src[Index];
	}
}

u32
StringLength(char* String)
{
	u32 Result = 0;

	if (String)
	{
		while (String[Result++]);
	}

	return Result;
}

image_set
GetImageSet(char* Name, char* Directory, b32 Load = false)
{
	image_set Result = {};

	StringCopy(Result.Name, Name, StringLength(Name));
	StringCopy(Result.Directory, Directory, StringLength(Directory));
	for (u32 ImageIndex = 0; ImageIndex < IMAGE_SET_SIZE; ++ImageIndex)
	{
		if (Name && Load)
		{
			char FileName[256] = {};
			sprintf(FileName, "data\\%s\\%s\\%s_%d.tif", Result.Directory, Result.Name, Result.Name, ImageIndex);
			Result.Images[ImageIndex] = ReadGrayscaleImage(FileName);
		}
		else
		{
			Result.Images[ImageIndex] = image();
		}
	}

	return Result;
}

image*
GetImage(image_set* ImageSet, u32 ImageIndex)
{
	image* Result = ImageSet->Images + ImageIndex;

	if (!Result->data)
	{
		if (StringLength(ImageSet->Name))
		{
			char FileName[256] = {};
			sprintf(FileName, "data\\%s\\%s\\%s_%d.tif", ImageSet->Directory, ImageSet->Name, ImageSet->Name, ImageIndex);
			ImageSet->Images[ImageIndex] = ReadGrayscaleImage(FileName);
			Result = ImageSet->Images + ImageIndex;
		}
	}

	return Result;
}

void
SaveImageSet(image_set* ImageSet)
{
	char FileName[256] = {};
	sprintf(FileName, "data");
	MakeDirectory(FileName);

	sprintf(FileName, "data\\%s", ImageSet->Directory);
	MakeDirectory(FileName);
	
	sprintf(FileName, "data\\%s\\%s", ImageSet->Directory, ImageSet->Name);
	MakeDirectory(FileName);

	for (u32 ImageIndex = 0; ImageIndex < IMAGE_SET_SIZE; ++ImageIndex)
	{
		sprintf(FileName, "data\\%s\\%s\\%s_%d.tif", ImageSet->Directory, ImageSet->Name, ImageSet->Name, ImageIndex);
		WriteImage(FileName, GetImage(ImageSet, ImageIndex));
	}
}

void
StabilizeImages(image_set* DstImageSet, image_set* SrcImageSet)
{
	char OutputWindowName[] = "Image Stabilization";
	CreateWindow(OutputWindowName);
	image StabilizationDisplay = ImageU8(480, 640);
	cv::putText(StabilizationDisplay, "Stabilizing images", {220, 240}, cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255));
	ShowImage(OutputWindowName, &StabilizationDisplay);
	WaitKey(1);

	image* PrevImage = GetImage(SrcImageSet, 0);
	image PrevImageEq = ImageU8(PrevImage);
	EqualizeHistogram(PrevImage, &PrevImageEq);
	v3 Trajectory = {};
	for (u32 ImageIndex = 0; ImageIndex < IMAGE_SET_SIZE; ++ImageIndex)
	{
		image* NextImage = GetImage(SrcImageSet, ImageIndex);
		image NextImageEq = ImageU8(NextImage);
		EqualizeHistogram(NextImage, &NextImageEq);

		v2 PrevCorners[MAX_CORNERS] = {};
		v2 NextCorners[MAX_CORNERS] = {};
		u32 FilteredCornerCount = 0;
		GoodFeaturesToTrack(&NextImageEq, PrevCorners, MAX_CORNERS, 0.005, 10);
		CalculateOpticalFlowPiramidsLucasKanade(&PrevImageEq, &NextImageEq, PrevCorners, NextCorners, MAX_CORNERS, &FilteredCornerCount);
		v3 Transform = EstimateRigidTransform(PrevCorners, NextCorners, FilteredCornerCount, false);
		Trajectory += Transform;
		image* DstImage = GetImage(DstImageSet, ImageIndex);
		WarpAffine(DstImage, NextImage, -Trajectory);

		PrevImageEq = NextImageEq;
	}

	cv::rectangle(StabilizationDisplay, {0, 0}, {640, 480}, cv::Scalar(0), CV_FILLED);
	cv::putText(StabilizationDisplay, "Saving images", {220, 240}, cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255));
	ShowImage(OutputWindowName, &StabilizationDisplay);
	WaitKey(1);

	SaveImageSet(DstImageSet);
	DestroyWindow(OutputWindowName);
}

b32
GetStabilizedImages(image_set* ImageSet, char* ImageSetName)
{
	*ImageSet = GetImageSet(ImageSetName, "Stabilized");
	// for (u32 ImageIndex = 0; ImageIndex < IMAGE_SET_SIZE; ++ImageIndex)
	{
		image* StabilizedImage = GetImage(ImageSet, 0 /*ImageIndex*/);
		if (StabilizedImage->data == 0)
		{
			return false;
		}
	}
	return true;
}

inline u32
PrevImageIndex(u32 ImageIndex)
{
	i32 PrevImageIndex = ImageIndex - 1;
	if (PrevImageIndex < 0)
	{
		PrevImageIndex = IMAGE_SET_SIZE - 1;
	}
	return PrevImageIndex;
}

inline u32
NextImageIndex(u32 ImageIndex)
{
	u32 NextImageIndex = ImageIndex + 1;
	if (NextImageIndex >= IMAGE_SET_SIZE)
	{
		NextImageIndex = 0;
	}
	return NextImageIndex;
}

inline b32
PointInBounds(image* Image, point_i32 Point)
{
	b32 Result = (Point.I >= 0 && Point.I < Image->rows) && (Point.J >= 0 && Point.J < Image->cols);
	return Result;
}

void
ExtractMaze(image* Src, image* Maze)
{
	EqualizeHistogram(Maze, Maze);
	Laplacian(Maze, Maze);
	MorphOpen(Maze, Maze, 3);
	AdaptiveThreshold(Maze, Maze);
	MorphClose(Maze, Maze, 3);
	MorphOpen(Maze, Maze, 3);
	image MazeLabels = ImageI32(Maze);
	u32 LabelCount = ConnectedComponents(Maze, &MazeLabels);
	ExtractLargestLabeledFeatures(Maze, Maze, &MazeLabels, LabelCount, 15, 30);
	MorphClose(Maze, Maze, 5);
	// TODO(alex): fill the small holes
}

void
Dictymaze()
{
	char ImageSetName[] = "ax2__52517_2";
	image_set ImageSet = GetImageSet(ImageSetName, "SourceSplit");

	image_set StabilizedImageSet;
	if (!GetStabilizedImages(&StabilizedImageSet, ImageSetName))
	{
		StabilizeImages(&StabilizedImageSet, &ImageSet);
	}

	char OutputWindowName1[] = "Output 1";
	CreateWindow(OutputWindowName1);

	char OutputWindowName2[] = "Output 2";
	CreateWindow(OutputWindowName2);

	// char HistogramWindowName1[] = "Histogram 1";
	// CreateWindow(HistogramWindowName1);

	// char HistogramWindowName2[] = "Histogram 2";
	// CreateWindow(HistogramWindowName2);

	image CellsHistDisplay = ImageU8(1000, 256 * 4);
	image CellsHistGradientDisplay = ImageU8(1000, 256 * 4);

	b32 Running = true;
	b32 Paused = true;
	u32 FrameTime = 33;
	u32 ImageIndex = 165;//101;

	cv::KalmanFilter KF(4, 2, 0);
	std::vector<v2> Predictions;
	std::vector<v2> Estimations;
	std::vector<v2> Actual;
	f32 LastSize = 0.f;
	b32 DrawCell = false;

	u32 CellsTrackingCount = 0;
	u32 CellsTrackingMax = 1;
	while (Running)
	{
		// image* Image = GetImage(&ImageSet, ImageIndex);
		image* StabilizedPrevImage = GetImage(&StabilizedImageSet, PrevImageIndex(ImageIndex));
		image StabilizedPrevImageEq = CloneImage(StabilizedPrevImage);
		EqualizeHistogram(&StabilizedPrevImageEq, &StabilizedPrevImageEq);
		image* StabilizedImage = GetImage(&StabilizedImageSet, ImageIndex);
		image StabilizedImageEq = CloneImage(StabilizedImage);
		EqualizeHistogram(&StabilizedImageEq, &StabilizedImageEq);

		image Maze = CloneImage(&StabilizedImageEq);
		ExtractMaze(&StabilizedImageEq, &Maze);
		image AreaOfInterestMask = ImageU8(&StabilizedImageEq);
		point_i32 AreaOfInterest[4] = {{82, 22}, {72, 335}, {930, 343}, {940, 47}};
		FillConvexPoly(&AreaOfInterestMask, AreaOfInterest, ArrayCount(AreaOfInterest), 255);

		AreaOfInterestMask = AreaOfInterestMask & Maze;

		image Difference = ImageU8(StabilizedImage);
		AbsoluteDifference(&StabilizedImageEq, &StabilizedPrevImageEq, &Difference);
		Difference = Difference & AreaOfInterestMask;

		image Cells = ImageU8(&Difference);
		DifferenceCellFilter(&Difference, &Cells);
		ShowImage(OutputWindowName1, &Cells);

		histogram CellsHist = CalculateHistogram(&Cells);

		// HistogramDraw(&CellsHistDisplay, &CellsHist, 8);
		// ShowImage(HistogramWindowName1, &CellsHistDisplay);

		i32 CellsHistGradient[256] = {};
		i32 MinGradient = 0;
		for (u32 Index = 1; Index < ArrayCount(CellsHist.Data); ++Index)
		{
			u32 PrevValue = CellsHist.Data[Index - 1];
			u32 Value = CellsHist.Data[Index];
			i32 Gradient = (i32)Value - (i32)PrevValue;
			if (Gradient < MinGradient)
			{
				MinGradient = Gradient;
			}
			CellsHistGradient[Index] = Gradient;
		}

		u32 Threshold = 0;
		for (u32 Index = 0; Index < ArrayCount(CellsHistGradient); ++Index)
		{
			i32 Gradient = CellsHistGradient[Index];
			if (Abs(Gradient) >= 20)
			{
				Threshold = Index;
			}
		}

		for (u32 Index = 1; Index < ArrayCount(CellsHistGradient); ++Index)
		{
			CellsHistGradient[Index] -= MinGradient;
		}
		// HistogramDraw(&CellsHistGradientDisplay, &CellsHist);
		// ShowImage(HistogramWindowName2, &CellsHistGradientDisplay);
		
		cv::threshold(Cells, Cells, Threshold, 255, cv::THRESH_TOZERO);
		ShowImage(OutputWindowName2, &Cells);

		image CandidateCellLabels = ImageI32(&Cells);
		u32 CandidateCellCount = ConnectedComponents(&Cells, &CandidateCellLabels) - 1;
		
		if (CandidateCellCount > 0)
		{
			candidate_cell* CandidateCells = (candidate_cell*)AllocateOnStackSafe(CandidateCellCount * SizeOf(candidate_cell));
			for (i32 LabelIndex = 0; LabelIndex < CandidateCellCount; ++LabelIndex)
			{
				CandidateCells[LabelIndex].Label = LabelIndex + 1;
				CandidateCells[LabelIndex].TopLeft = {Cells.cols, Cells.rows};
				CandidateCells[LabelIndex].BottomRight = {};
				CandidateCells[LabelIndex].Center = {};
				CandidateCells[LabelIndex].Size = 0;
				CandidateCells[LabelIndex].WeightedSize = 0.;
			}

			for (i32 Row = 0; Row < CandidateCellLabels.rows; ++Row)
			{
				for (i32 Col = 0; Col < CandidateCellLabels.cols; ++Col)
				{
					point_i32 Point = {Col, Row};
					i32 LabelIndex = GetAtI32(&CandidateCellLabels, Point) - 1;
					if (LabelIndex >= 0)
					{
						u8 Value = GetAtU8(&Cells, Point);

						if (Row < CandidateCells[LabelIndex].TopLeft.I)
						{
							CandidateCells[LabelIndex].TopLeft.I = Row;
						}
						if (Col < CandidateCells[LabelIndex].TopLeft.J)
						{
							CandidateCells[LabelIndex].TopLeft.J = Col;
						}

						if (Row > CandidateCells[LabelIndex].BottomRight.I)
						{
							CandidateCells[LabelIndex].BottomRight.I = Row;
						}
						if (Col > CandidateCells[LabelIndex].BottomRight.J)
						{
							CandidateCells[LabelIndex].BottomRight.J = Col;
						}

						CandidateCells[LabelIndex].Center.X += (f32)Col;
						CandidateCells[LabelIndex].Center.Y += (f32)Row;
						CandidateCells[LabelIndex].Size += 1;
						CandidateCells[LabelIndex].WeightedSize += (f32)Value / 255.;
					}
				}
			}

			for (i32 LabelIndex = 0; LabelIndex < CandidateCellCount; ++LabelIndex)
			{
				u32 Size = CandidateCells[LabelIndex].Size;
				CandidateCells[LabelIndex].Center.X /= (f32)Size;
				CandidateCells[LabelIndex].Center.Y /= (f32)Size;
			}

			// Sort by size
			for (i32 CandidateCellIndex1 = 0; CandidateCellIndex1 < CandidateCellCount - 1; ++CandidateCellIndex1)
			{
				for (i32 CandidateCellIndex2 = CandidateCellIndex1 + 1; CandidateCellIndex2 < CandidateCellCount; ++CandidateCellIndex2)
				{
					if (CandidateCells[CandidateCellIndex1].WeightedSize < CandidateCells[CandidateCellIndex2].WeightedSize)
					{
						candidate_cell Temp = CandidateCells[CandidateCellIndex1];
						CandidateCells[CandidateCellIndex1] = CandidateCells[CandidateCellIndex2];
						CandidateCells[CandidateCellIndex2] = Temp;
					}
				}
			}

			if (CellsTrackingCount < CellsTrackingMax)
			{
				// TODO(alex): initialize only on probable cells
				b32 ProbableCellDetected = (Threshold < 90) && (CandidateCells[0].WeightedSize >= 256.f);

				if (ProbableCellDetected)
				{
					candidate_cell Cell = CandidateCells[0];

					cv::setIdentity(KF.transitionMatrix);
					KF.transitionMatrix.at<f32>(0, 2) = 1.f;
					KF.transitionMatrix.at<f32>(1, 3) = 1.f;

					KF.statePre.setTo(0.f);
					KF.statePre.at<f32>(0) = Cell.Center.X;
					KF.statePre.at<f32>(1) = Cell.Center.Y;

					KF.statePost.setTo(0.f);
					KF.statePost.at<f32>(0) = Cell.Center.X;
					KF.statePost.at<f32>(1) = Cell.Center.Y;

					cv::setIdentity(KF.measurementMatrix);
					cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(0.001f));
					cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(0.1f));
					cv::setIdentity(KF.errorCovPost, cv::Scalar::all(0.1f));

					LastSize = Cell.WeightedSize;

					++CellsTrackingCount;
				}
			}

			for (u32 CellsTrackingIndex = 0; CellsTrackingIndex < CellsTrackingCount; ++CellsTrackingIndex)
			{
				image Prediction = KF.predict();
				v2 PredictedPoint = {Prediction.at<f32>(0), Prediction.at<f32>(1)};
				Predictions.push_back(PredictedPoint);

				// Extract movement data
				std::vector<f32> Scores;
				for (i32 CandidateCellIndex = 0; CandidateCellIndex < CandidateCellCount; ++CandidateCellIndex)
				{
					candidate_cell CandidateCell = CandidateCells[CandidateCellIndex];
					
					v2 CenterToPrediction = PredictedPoint - CandidateCell.Center;
					f32 DistanceSqrToPrediction = Dot(CenterToPrediction, CenterToPrediction);

					f32 DirectionScore = 0.f;
					if (CenterToPrediction.X > 0.f)
					{
						DirectionScore = 1.f;
					}
					else if (CenterToPrediction.X > -16.f)
					{
						DirectionScore = (16.f - CenterToPrediction.X) / 16.f;
					}
					f32 DistanceScore = (DistanceSqrToPrediction <= 625.f) ? (DistanceSqrToPrediction / 625.f) : 0.f;
					f32 SizeScore = 0.f;
					if (CandidateCell.WeightedSize > 32.f)
					{
						SizeScore = (CandidateCell.WeightedSize < LastSize) ? (LastSize / CandidateCell.WeightedSize) : 1.f;
					}

					f32 Score = DirectionScore * DistanceScore * SizeScore;
					Scores.push_back(Score);
				}

				// Sort
				for (i32 CandidateCellIndex1 = 0; CandidateCellIndex1 < CandidateCellCount - 1; ++CandidateCellIndex1)
				{
					for (i32 CandidateCellIndex2 = CandidateCellIndex1 + 1; CandidateCellIndex2 < CandidateCellCount; ++CandidateCellIndex2)
					{
						if (Scores[CandidateCellIndex1] < Scores[CandidateCellIndex2])
						{
							candidate_cell Temp = CandidateCells[CandidateCellIndex1];
							CandidateCells[CandidateCellIndex1] = CandidateCells[CandidateCellIndex2];
							CandidateCells[CandidateCellIndex2] = Temp;

							f32 TempScore = Scores[CandidateCellIndex1];
							Scores[CandidateCellIndex1] = Scores[CandidateCellIndex2];
							Scores[CandidateCellIndex2] = TempScore;
						}
					}
				}

				f32 TopScore = Scores[0];
				if (TopScore > 0.f)
				{
					DrawCell = true;

					candidate_cell TopScoreCandidateCell = CandidateCells[0];
					Actual.push_back(TopScoreCandidateCell.Center);

					image Correction = ImageF32(2, 1);
					Correction.at<f32>(0) = TopScoreCandidateCell.Center.X;
					Correction.at<f32>(1) = TopScoreCandidateCell.Center.Y;
					image Estimate = KF.correct(Correction);
					v2 EstimatedPoint = {Estimate.at<f32>(0), Estimate.at<f32>(1)};
					Estimations.push_back(EstimatedPoint);

					LastSize = TopScoreCandidateCell.WeightedSize;
				}
				else
				{
					DrawCell = false;

					image Correction = ImageF32(2, 1);
					Correction.at<f32>(0) = PredictedPoint.X;
					Correction.at<f32>(1) = PredictedPoint.Y;
					image Estimate = KF.correct(Correction);
					v2 EstimatedPoint = {Estimate.at<f32>(0), Estimate.at<f32>(1)};
					Estimations.push_back(EstimatedPoint);

					Actual.push_back(EstimatedPoint);
				}
			}

			// Draw
			image OutputCells = ImageU8C3(&Cells);
			for (i32 Row = 0; Row < Cells.rows; ++Row)
			{
				for (i32 Col = 0; Col < Cells.cols; ++Col)
				{
					point_i32 Point = {Col, Row};
					u8 Value = GetAtU8(&Cells, Point);
					pixel_rgb Pixel = {Value, Value, Value};
					SetAtU8C3(&OutputCells, Point, Pixel);
				}
			}

			if (DrawCell)
			{
				for (i32 CellIndex = 0; CellIndex < CellsTrackingCount; ++CellIndex)
				{
					candidate_cell Cell = CandidateCells[CellIndex];

					cv::Scalar Color(HighlightColors[CellIndex].B, HighlightColors[CellIndex].G, HighlightColors[CellIndex].R);
					cv::rectangle(OutputCells, {Cell.TopLeft.X - 1, Cell.TopLeft.Y - 1}, {Cell.BottomRight.X + 1, Cell.BottomRight.Y + 1}, Color);
				}
			}

			for (i32 PointIndex = 1; PointIndex < Predictions.size(); ++PointIndex)
			{
				v2 PrevActualPoint = Actual[PointIndex - 1];
				v2 ActualPoint = Actual[PointIndex];
				cv::line(OutputCells, {(i32)PrevActualPoint.X, (i32)PrevActualPoint.Y}, {(i32)ActualPoint.X, (i32)ActualPoint.Y}, cv::Scalar(255, 0, 0));

				v2 PrevPredictionPoint = Predictions[PointIndex - 1];
				v2 PredictionPoint = Predictions[PointIndex];
				cv::line(OutputCells, {(i32)PrevPredictionPoint.X, (i32)PrevPredictionPoint.Y}, {(i32)PredictionPoint.X, (i32)PredictionPoint.Y}, cv::Scalar(0, 0, 255));

				v2 PrevEstimatedPoint = Estimations[PointIndex - 1];
				v2 EstimatedPoint = Estimations[PointIndex];
				cv::line(OutputCells, {(i32)PrevEstimatedPoint.X, (i32)PrevEstimatedPoint.Y}, {(i32)EstimatedPoint.X, (i32)EstimatedPoint.Y}, cv::Scalar(0, 255, 0));
			}

			ShowImage("Cells", &OutputCells);

			FreeOnStackSafe(CandidateCells);
		}

		u32 KeyCode = WaitKey(Paused ? 0 : FrameTime);
		switch (KeyCode)
		{
			case KEY_SPACE:
			{
				Paused = !Paused;
			} break;
			case KEY_ESCAPE:
			{
				Running = false;				
			} break;
			case KEY_R:
			{
				ImageIndex = 1;
			} break;
			case KEY_S:
			{
				ImageIndex = 1;
				Paused = true;
			} break;
			case KEY_RIGHT:
			{
				ImageIndex = NextImageIndex(ImageIndex);
			} break;
			case KEY_LEFT:
			{
				ImageIndex = PrevImageIndex(ImageIndex);
			} break;
			case -1:
			{
				u32 NewImageIndex = NextImageIndex(ImageIndex);
				ImageIndex = NewImageIndex;
				if (NewImageIndex < ImageIndex)
				{
					Paused = true;
				}
			} break;
			default: 
			{
				int a = KeyCode;
			} break;
		}
	}
}
