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

inline b32
CompareCandidateCellsByWeightedSize(candidate_cell* A, candidate_cell* B)
{
	b32 Result = A->WeightedSize < B->WeightedSize;
	return Result;
}

inline b32
CompareCellRanking(cell_ranking* A, cell_ranking* B)
{
	b32 Result = A->Score < B->Score;
	return Result;
}

void
Dictymaze(dictymaze_parameters* Parameters)
{
	image_set ImageSet = GetImageSet(Parameters->ImageSetName, "SourceSplit");

	image_set StabilizedImageSet;
	if (!GetStabilizedImages(&StabilizedImageSet, Parameters->ImageSetName))
	{
		StabilizeImages(&StabilizedImageSet, &ImageSet);
	}

/*	char OutputWindowName1[] = "Output 1";
	CreateWindow(OutputWindowName1);*/

/*	char OutputWindowName2[] = "Output 2";
	CreateWindow(OutputWindowName2);*/

#if DRAW_HISTOGRAMS
	char HistogramWindowName1[] = "Histogram 1";
	CreateWindow(HistogramWindowName1);

	char HistogramWindowName2[] = "Histogram 2";
	CreateWindow(HistogramWindowName2);
#endif

	image CellsHistDisplay = ImageU8(1000, 256 * 4);
	image CellsHistGradientDisplay = ImageU8(1000, 256 * 4);

	b32 Running = true;
	b32 Paused = false;
	u32 FrameTime = 33;
	u32 ImageIndex = 1;

	u32 CellTrackerCount = 0;
	cell_tracker* CellTrackers = (cell_tracker*)MemoryAllocate(Parameters->MaxTrackedCells * SizeOf(cell_tracker));
	MemoryZero(CellTrackers, Parameters->MaxTrackedCells * SizeOf(cell_tracker));
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
		// ShowImage(OutputWindowName1, &Cells);

		histogram CellsHist = CalculateHistogram(&Cells);

#if DRAW_HISTOGRAMS
		HistogramDraw(&CellsHistDisplay, &CellsHist, 8);
		ShowImage(HistogramWindowName1, &CellsHistDisplay);
#endif

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

		u32 ThresholdValue = 0;
		for (u32 Index = 0; Index < ArrayCount(CellsHistGradient); ++Index)
		{
			i32 Gradient = CellsHistGradient[Index];
			if (Abs(Gradient) >= 20)
			{
				ThresholdValue = Index;
			}
		}

		for (u32 Index = 1; Index < ArrayCount(CellsHistGradient); ++Index)
		{
			CellsHistGradient[Index] -= MinGradient;
		}

#if DRAW_HISTOGRAMS
		HistogramDraw(&CellsHistGradientDisplay, &CellsHist);
		ShowImage(HistogramWindowName2, &CellsHistGradientDisplay);
#endif

		Threshold(&Cells, &Cells, ThresholdValue);
		// ShowImage(OutputWindowName2, &Cells);
		/*char O1Name[256] = {};
		sprintf(O1Name, "data\\Output\\Thresholded_%d.png", ImageIndex);
		cv::imwrite(O1Name, Cells);*/

		image OutputCells = ImageU8C3(&Cells);
		image OutputOriginal = ImageU8C3(&Cells);
		for (i32 Row = 0; Row < Cells.rows; ++Row)
		{
			for (i32 Col = 0; Col < Cells.cols; ++Col)
			{
				point_i32 Point = {Col, Row};
				u8 Value = GetAtU8(&Cells, Point);
				pixel_rgb Pixel = {Value, Value, Value};
				SetAtU8C3(&OutputCells, Point, Pixel);

				Value = GetAtU8(StabilizedImage, Point);
				Pixel = {Value, Value, Value};
				SetAtU8C3(&OutputOriginal, Point, Pixel);
			}
		}

		image CandidateCellLabels = ImageI32(&Cells);
		u32 CandidateCellCount = ConnectedComponents(&Cells, &CandidateCellLabels) - 1;

		// NOTE(alex): Tracking
		if (CandidateCellCount > 0)
		{
			candidate_cell* CandidateCells = (candidate_cell*)MemoryAllocate(CandidateCellCount * SizeOf(candidate_cell));
			ExtractCandidateCells(&Cells, &CandidateCellLabels, CandidateCells, CandidateCellCount);

			for (i32 CandidateCellIndex = 0; CandidateCellIndex < CandidateCellCount; ++CandidateCellIndex)
			{
				candidate_cell* CandidateCell = CandidateCells + CandidateCellIndex;
				CandidateCell->Scores = (f32*)MemoryAllocate(CellTrackerCount * SizeOf(f32));
				CandidateCell->ScoreCount = CellTrackerCount;
			}

			Sort(CandidateCells, candidate_cell, CandidateCellCount, CompareCandidateCellsByWeightedSize);

			// NOTE(alex): compute scores
			for (u32 CellTrackerIndex = 0; CellTrackerIndex < CellTrackerCount; ++CellTrackerIndex)
			{
				cell_tracker* CellTracker = CellTrackers + CellTrackerIndex;

				v2 PredictedPoint = CellTrackerPredict(CellTracker, ImageIndex);

				for (i32 CandidateCellIndex = 0; CandidateCellIndex < CandidateCellCount; ++CandidateCellIndex)
				{
					candidate_cell* CandidateCell = CandidateCells + CandidateCellIndex;
					
					v2 CenterToPrediction = PredictedPoint - CandidateCell->Center;

					// TODO(alex): angle this
					// TODO(alex): extract constants
					f32 DirectionScore = 0.f;
					if (CenterToPrediction.X > 0.f)
					{
						DirectionScore = 1.f;
					}
					else if (CenterToPrediction.X > -16.f)
					{
						DirectionScore = (16.f + CenterToPrediction.X) / 16.f;
					}

					f32 DistanceSqrToPrediction = Dot(CenterToPrediction, CenterToPrediction);
					f32 DistanceScore = (DistanceSqrToPrediction < Parameters->MaxEstimateDistanceSq) ? ((Parameters->MaxEstimateDistanceSq - DistanceSqrToPrediction) / Parameters->MaxEstimateDistanceSq) : 0.f;
					
					f32 LastSize = CellTrackerLastSize(CellTracker, ImageIndex);
					f32 SizeScore = 0.f;
					if (CandidateCell->WeightedSize >= Parameters->MinDetectionSize)
					{
						SizeScore = (CandidateCell->WeightedSize < LastSize) ? (LastSize / CandidateCell->WeightedSize) : 1.f;
					}

					f32 Score = DirectionScore * DistanceScore * SizeScore;

					CandidateCell->Scores[CellTrackerIndex] = Score;
				}
			}

			// NOTE(alex): sort rankings
			u32 CellRankingCount = CellTrackerCount * CandidateCellCount;
			cell_ranking* CellRankings = (cell_ranking*)MemoryAllocate(CellRankingCount * SizeOf(cell_ranking));
			for (u32 CellTrackerIndex = 0; CellTrackerIndex < CellTrackerCount; ++CellTrackerIndex)
			{
				cell_tracker* CellTracker = CellTrackers + CellTrackerIndex;
				
				for (i32 CandidateCellIndex = 0; CandidateCellIndex < CandidateCellCount; ++CandidateCellIndex)
				{
					candidate_cell* CandidateCell = CandidateCells + CandidateCellIndex;

					cell_ranking* CellRanking = CellRankings + CellTrackerIndex * CandidateCellCount + CandidateCellIndex;
					CellRanking->CellTrackerIndex = CellTrackerIndex;
					CellRanking->CandidateCellIndex = CandidateCellIndex;
					CellRanking->Score = CandidateCell->Scores[CellTrackerIndex];
				}
			}
			Sort(CellRankings, cell_ranking, CellRankingCount, CompareCellRanking);

			// NOTE(alex): select best ranking prediction and apply correction
			cell_ranking** SelectedCellRankings = (cell_ranking**)MemoryAllocate(CellTrackerCount * SizeOf(cell_ranking*));
			u32 SelectedCellRankingCount = 0;
			u32 SelectionDryRuns = 0;
			while ((SelectedCellRankingCount + SelectionDryRuns) < CellTrackerCount)
			{
				// NOTE(alex): select the top score
				cell_ranking* SelectedCellRanking = 0;
				for (u32 CellRankingIndex = 0; CellRankingIndex < CellRankingCount; ++CellRankingIndex)
				{
					cell_ranking* CellRanking = CellRankings + CellRankingIndex;

					b32 CellIndexUnused = true;
					for (u32 SelectedCellRankingIndex = 0; SelectedCellRankingIndex < SelectedCellRankingCount; ++SelectedCellRankingIndex)
					{
						if (SelectedCellRankings[SelectedCellRankingIndex]->CellTrackerIndex == CellRanking->CellTrackerIndex ||
							SelectedCellRankings[SelectedCellRankingIndex]->CandidateCellIndex == CellRanking->CandidateCellIndex)
						{
							CellIndexUnused = false;
							break;
						}
					}

					if (CellIndexUnused)
					{
						SelectedCellRanking = CellRanking;
						break;
					}
				}

				if (SelectedCellRanking != 0)
				{
					SelectedCellRankings[SelectedCellRankingCount++] = SelectedCellRanking;
					cell_tracker* CellTracker = CellTrackers + SelectedCellRanking->CellTrackerIndex;
					candidate_cell* CandidateCell = (SelectedCellRanking->Score > 0.f) ? CandidateCells + SelectedCellRanking->CandidateCellIndex : 0;
					CellTrackerCorrect(CellTracker, ImageIndex, CandidateCell);
				}
				else
				{
					++SelectionDryRuns;
				}
			}
			// NOTE(alex): correct the unmatched trackers
			for (u32 CellTrackerIndex = 0; CellTrackerIndex < CellTrackerCount; ++CellTrackerIndex)
			{
				cell_tracker* CellTracker = CellTrackers + CellTrackerIndex;
				if (CellTracker->Sizes[ImageIndex] == 0)
				{
					CellTrackerCorrect(CellTracker, ImageIndex, 0);
				}
			}

			// NOTE(alex): start tracking new cells
			if (CellTrackerCount < Parameters->MaxTrackedCells && ThresholdValue <= 85)
			{
				u32* UsedCandidateCells = (u32*)MemoryAllocate(Parameters->MaxTrackedCells * SizeOf(u32));;
				u32 UsedCandidateCellCount = 0;
				b32 OptionsNotExhausted = true;
				while (CellTrackerCount < Parameters->MaxTrackedCells && OptionsNotExhausted)
				{
					u32 CandidateCellIndex;
					for (CandidateCellIndex = 0; CandidateCellIndex < CandidateCellCount; ++CandidateCellIndex)
					{
						candidate_cell* CandidateCell = CandidateCells + CandidateCellIndex;

						if (CandidateCell->WeightedSize >= Parameters->MinTrackingSize)
						{
							b32 CellUntracked = true;
							for (u32 SelectedCellRankingIndex = 0; SelectedCellRankingIndex < SelectedCellRankingCount; ++SelectedCellRankingIndex)
							{
								if (SelectedCellRankings[SelectedCellRankingIndex]->CandidateCellIndex == CandidateCellIndex)
								{
									CellUntracked = false;
									break;
								}
							}

							if (CellUntracked)
							{
								b32 CellUnused = true;
								for (u32 UsedCandidateCellIndex = 0; UsedCandidateCellIndex < UsedCandidateCellCount; ++UsedCandidateCellIndex)
								{
									if (UsedCandidateCells[UsedCandidateCellIndex] == CandidateCellIndex)
									{
										CellUnused = false;
										break;
									}
								}

								if (CellUnused)
								{
									cell_tracker* CellTracker = CellTrackers + CellTrackerCount;
									CellTrackerInit(CellTracker, CandidateCell, ImageIndex, IMAGE_SET_SIZE);
									++CellTrackerCount;
									UsedCandidateCells[UsedCandidateCellCount++] = CandidateCellIndex;
									break;
								}
							}
						}
					}
					if (CandidateCellIndex >= CandidateCellCount - 1)
					{
						OptionsNotExhausted = false;
					}
				}
				MemoryFree(UsedCandidateCells);
			}
			MemoryFree(SelectedCellRankings);
			MemoryFree(CellRankings);

			for (u32 CellTrackerIndex = 0; CellTrackerIndex < CellTrackerCount; ++CellTrackerIndex)
			{
				cell_tracker* CellTracker = CellTrackers + CellTrackerIndex;
				cv::Scalar Color(HighlightColors[CellTrackerIndex].B, HighlightColors[CellTrackerIndex].G, HighlightColors[CellTrackerIndex].R);

				rect_i32 CellBounds = CellTracker->BoundingBoxes[ImageIndex];
				if (CellBounds.TopLeft.X)
				{
					cv::rectangle(OutputCells, {CellBounds.TopLeft.X - 1, CellBounds.TopLeft.Y - 1}, {CellBounds.BottomRight.X + 1, CellBounds.BottomRight.Y + 1}, Color);
					cv::rectangle(OutputOriginal, {CellBounds.TopLeft.X - 1, CellBounds.TopLeft.Y - 1}, {CellBounds.BottomRight.X + 1, CellBounds.BottomRight.Y + 1}, Color);
				}

				for (u32 StateIndex = 1; StateIndex < CellTracker->StateCount; ++StateIndex)
				{
					/*v2 PrevPred = CellTracker->PredictedPositions[StateIndex - 1];
					v2 CurrPred = CellTracker->PredictedPositions[StateIndex];

					if (PrevPred.X && CurrPred.X)
					{
						cv::line(OutputCells, {(i32)PrevPred.X, (i32)PrevPred.Y}, {(i32)CurrPred.X, (i32)CurrPred.Y}, Color);
					}*/

					v2 PrevEsti = CellTracker->EstimatedPositions[StateIndex - 1];
					v2 CurrEsti = CellTracker->EstimatedPositions[StateIndex];

					if (PrevEsti.X && CurrEsti.X)
					{
						cv::line(OutputCells, {(i32)PrevEsti.X, (i32)PrevEsti.Y}, {(i32)CurrEsti.X, (i32)CurrEsti.Y}, Color);
						cv::line(OutputOriginal, {(i32)PrevEsti.X, (i32)PrevEsti.Y}, {(i32)CurrEsti.X, (i32)CurrEsti.Y}, Color);
					}

					/*v2 PrevActu = CellTracker->ActualPositions[StateIndex - 1];
					v2 CurrActu = CellTracker->ActualPositions[StateIndex];

					if (PrevActu.X && CurrActu.X)
					{
						cv::line(OutputCells, {(i32)PrevActu.X, (i32)PrevActu.Y}, {(i32)CurrActu.X, (i32)CurrActu.Y}, Color);
					}*/
				}
			}

			ShowImage("Cells", &OutputCells);
			ShowImage("Original", &OutputOriginal);
			
			for (i32 CandidateCellIndex = 0; CandidateCellIndex < CandidateCellCount; ++CandidateCellIndex)
			{
				candidate_cell* CandidateCell = CandidateCells + CandidateCellIndex;
				MemoryFree(CandidateCell->Scores);
			}
			MemoryFree(CandidateCells);
		}
		else
		{
			for (u32 CellTrackerIndex = 0; CellTrackerIndex < CellTrackerCount; ++CellTrackerIndex)
			{
				cell_tracker* CellTracker = CellTrackers + CellTrackerIndex;
				CellTracker->PredictedPositions[ImageIndex] = CellTracker->PredictedPositions[ImageIndex - 1];
				CellTracker->EstimatedPositions[ImageIndex] = CellTracker->EstimatedPositions[ImageIndex - 1];
				CellTracker->ActualPositions[ImageIndex] = CellTracker->ActualPositions[ImageIndex - 1];
				CellTracker->Sizes[ImageIndex] = CellTracker->Sizes[ImageIndex - 1];
			}
		}

		char CellsName[256] = {};
		sprintf(CellsName, "data\\Output\\Cells_%d.png", ImageIndex);
		cv::imwrite(CellsName, OutputCells);
		sprintf(CellsName, "data\\Output\\Cells_Original_%d.png", ImageIndex);
		cv::imwrite(CellsName, OutputOriginal);

		if (NextImageIndex(ImageIndex) == 0)
		{
			Paused = true;
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
				if (NewImageIndex < ImageIndex)
				{
					Paused = true;
				}
				ImageIndex = NewImageIndex;

			} break;
			default: 
			{
				int a = KeyCode;
			} break;
		}
	}
	MemoryFree(CellTrackers);
}
