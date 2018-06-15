#include "HierarchyPathfinder.h"

namespace Hierarchy
{
	PathFinder::PathFinder(const Hierarchy* hierarchy)
		: mHierarchy(hierarchy),
		mClosedSet(hierarchy)
	{
	}

	PathFinder::IterationRes PathFinder::begin(const CellAndCorner& root)
	{
		Step step;
		step.mStepType = StepType::DIAG;
		step.mCornerIndex = root.mCorner;
		step.mCellKey = root.mCell;
		step.mClosedSetEdges = 0;
		step.mClosedSetCellKey = CellKey::invalidCellKey();
		step.mPoint = root.mCell.cornerGood(root.mCorner);
		step.mParentPoint = Point::invalidPoint();
		step.mTraversedCost = Cost(0, 0);
		validateStep(step);
		mOpenSet.push(step);

		return IterationRes::IN_PROGRESS;
	}

	PathFinder::IterationRes PathFinder::begin(Point startPoint, Point endPoint, DebugDraw* debugDraw)
	{
		mStartPoint = startPoint;
		mEndPoint = endPoint;

		DIDA_ASSERT(!"not implemented yet");

		return IterationRes::IN_PROGRESS;
	}

	PathFinder::IterationRes PathFinder::iteration(DebugDraw* debugDraw)
	{
		Step step;
		while(true)
		{
			if(mOpenSet.empty())
			{
				return IterationRes::UNREACHABLE;
			}

			step = mOpenSet.top();
			mOpenSet.pop();

			if(mClosedSet.tryAddPoint(step.mPoint, step.mParentPoint) &&
				!mClosedSet.pointTraversed(step.mCellKey, step.mPoint))
			{
				if(step.mClosedSetEdges)
				{
					mClosedSet.addEdges(step.mClosedSetCellKey, step.mClosedSetEdges);
				}

				if(debugDraw && step.mParentPoint != Point::invalidPoint())
				{
					debugDraw->drawLine(step.mParentPoint, step.mPoint);
				}

				switch(step.mStepType)
				{
				case StepType::DIAG:
					stepDiag(step);
					return IterationRes::IN_PROGRESS;

				case StepType::DIAG_OFF_GRID:
					stepDiagOffGrid(step);
					return IterationRes::IN_PROGRESS;
						
				case StepType::BEAM_X:
				case StepType::BEAM_Y:
					stepBeam(step);
					return IterationRes::IN_PROGRESS;
				}
			}
		}
	}

	void PathFinder::stepDiag(const Step& step)
	{
		switch(step.mCornerIndex)
		{
		case CornerIndex::MIN_X_MIN_Y:
			stepDiagTempl<CornerIndex::MIN_X_MIN_Y>(step);
			break;

		case CornerIndex::MAX_X_MIN_Y:
			stepDiagTempl<CornerIndex::MAX_X_MIN_Y>(step);
			break;

		case CornerIndex::MIN_X_MAX_Y:
			stepDiagTempl<CornerIndex::MIN_X_MAX_Y>(step);
			break;

		case CornerIndex::MAX_X_MAX_Y:
			stepDiagTempl<CornerIndex::MAX_X_MAX_Y>(step);
			break;
		}
	}

	template <CornerIndex cornerIndex>
	void PathFinder::stepDiagTempl(const Step& step)
	{
		DIDA_ASSERT(step.mStepType == StepType::DIAG);
		DIDA_ASSERT(mHierarchy->cellAt(step.mCellKey) == Cell::FULL);
		DIDA_ASSERT(step.mPoint == step.mCellKey.cornerGood(cornerIndex));

		{
			int8_t cornerX = (int8_t)cornerIndex & 1;
			int8_t cornerY = (int8_t)cornerIndex >> 1;

			CellKey toCellKey = mHierarchy->diagNextCellKey(step.mCellKey, cornerIndex);
			if(isFullCell(mHierarchy->cellAt(toCellKey)))
			{
				Point toPoint = step.mPoint;
				toPoint.mX += (1 - 2 * cornerX) << step.mCellKey.mLevel;
				toPoint.mY += (1 - 2 * cornerY) << step.mCellKey.mLevel;

				bool offGrid = toCellKey.cornerGood(cornerIndex) != toPoint;

				Step nextStep;
				nextStep.mStepType = offGrid ? StepType::DIAG_OFF_GRID : StepType::DIAG;
				nextStep.mCornerIndex = cornerIndex;
				nextStep.mCellKey = toCellKey;
				nextStep.mClosedSetEdges = 0xf;
				nextStep.mClosedSetCellKey = step.mCellKey;
				nextStep.mParentPoint = step.mPoint;
				nextStep.mPoint = toPoint;
				nextStep.mTraversedCost = step.mTraversedCost + Cost(0, 1 << step.mCellKey.mLevel);
				validateStep(nextStep);
				mOpenSet.push(nextStep);
			}
		}

		enqueueSideEdge<cornerIndex, 0>(step.mCellKey, step.mPoint, step.mTraversedCost);
		enqueueSideEdge<cornerIndex, 1>(step.mCellKey, step.mPoint, step.mTraversedCost);

		Point min = step.mCellKey.cornerGood(CornerIndex::MIN_X_MIN_Y);
		Point max = step.mCellKey.cornerGood(CornerIndex::MAX_X_MAX_Y);

		enqueueBeam<cornerIndex, 0>(step, step.mPoint, min.mY, max.mY);
		enqueueBeam<cornerIndex, 1>(step, step.mPoint, min.mX, max.mX);
	}

	void PathFinder::stepDiagOffGrid(const Step& step)
	{
		switch(step.mCornerIndex)
		{
		case CornerIndex::MIN_X_MIN_Y:
			stepDiagOffGridTempl<CornerIndex::MIN_X_MIN_Y>(step);
			break;

		case CornerIndex::MAX_X_MIN_Y:
			stepDiagOffGridTempl<CornerIndex::MAX_X_MIN_Y>(step);
			break;

		case CornerIndex::MIN_X_MAX_Y:
			stepDiagOffGridTempl<CornerIndex::MIN_X_MAX_Y>(step);
			break;

		case CornerIndex::MAX_X_MAX_Y:
			stepDiagOffGridTempl<CornerIndex::MAX_X_MAX_Y>(step);
			break;
		}
	}

	template <CornerIndex cornerIndex>
	void PathFinder::stepDiagOffGridTempl(const Step& step)
	{
		DIDA_ASSERT(step.mStepType == StepType::DIAG_OFF_GRID);
		DIDA_ASSERT(mHierarchy->cellAt(step.mCellKey) == Cell::FULL);
		DIDA_ASSERT(step.mPoint != step.mCellKey.cornerGood(cornerIndex));

		CornerConnectionInfo<cornerIndex> connectionInfo;
		connectionInfo.init(*mHierarchy, step.mPoint, step.mCellKey);

		{
			enqueueDiag<cornerIndex>(step.mCellKey, step.mPoint, step.mTraversedCost, 
				connectionInfo.mNextCellKey, connectionInfo.mDiagEndPt, 0);

			enqueueDiag<cornerIndex>(step.mCellKey, step.mPoint, step.mTraversedCost, 
				connectionInfo.mNextOnGridCellKey, connectionInfo.mOnGridPoint, 0);
		}

		if(connectionInfo.mXBeamTouchesSideEdge)
			enqueueSideEdge<cornerIndex, 0>(step.mCellKey, step.mPoint, step.mTraversedCost);

		if(connectionInfo.mYBeamTouchesSideEdge)
			enqueueSideEdge<cornerIndex, 1>(step.mCellKey, step.mPoint, step.mTraversedCost);

		enqueueBeam<cornerIndex, 0>(step, step.mPoint, connectionInfo.mXBeamMin, connectionInfo.mXBeamMax);
		enqueueBeam<cornerIndex, 1>(step, step.mPoint, connectionInfo.mYBeamMin, connectionInfo.mYBeamMax);
	}

	void PathFinder::stepBeam(const Step& step)
	{
		DIDA_ASSERT(step.mStepType == StepType::BEAM_X || step.mStepType == StepType::BEAM_Y);

		switch(step.mCornerIndex)
		{
		case CornerIndex::MIN_X_MIN_Y:
			if(step.mStepType == StepType::BEAM_X)
				stepBeamTempl<CornerIndex::MIN_X_MIN_Y, 0>(step);
			else
				stepBeamTempl<CornerIndex::MIN_X_MIN_Y, 1>(step);
			break;

		case CornerIndex::MAX_X_MIN_Y:
			if(step.mStepType == StepType::BEAM_X)
				stepBeamTempl<CornerIndex::MAX_X_MIN_Y, 0>(step);
			else
				stepBeamTempl<CornerIndex::MAX_X_MIN_Y, 1>(step);
			break;

		case CornerIndex::MIN_X_MAX_Y:
			if(step.mStepType == StepType::BEAM_X)
				stepBeamTempl<CornerIndex::MIN_X_MAX_Y, 0>(step);
			else
				stepBeamTempl<CornerIndex::MIN_X_MAX_Y, 1>(step);
			break;

		case CornerIndex::MAX_X_MAX_Y:
			if(step.mStepType == StepType::BEAM_X)
				stepBeamTempl<CornerIndex::MAX_X_MAX_Y, 0>(step);
			else
				stepBeamTempl<CornerIndex::MAX_X_MAX_Y, 1>(step);
			break;
		}
	}

	template <CornerIndex cornerIndex, int8_t axis>
	void PathFinder::stepBeamTempl(const Step& step)
	{
		constexpr int8_t perpAxis = axis ^ 1;
		constexpr int8_t cornerOnAxis = ((int8_t)cornerIndex >> axis) & 1;

		int16_t cellMin = step.mCellKey.mCoords[perpAxis] << step.mCellKey.mLevel;
		int16_t cellMax = cellMin + (1 << step.mCellKey.mLevel) - 1;

		if(cellMin == step.mBeamMin)
		{
			enqueueSideEdge<cornerIndex, axis>(step.mCellKey, step.mPoint, step.mTraversedCost);
		}

		if(cellMax == step.mBeamMax)
		{
			enqueueSideEdge<cornerIndex, axis>(step.mCellKey, step.mParentPoint, step.mTraversedCost);
		}

		enqueueBeam<cornerIndex, axis>(step, step.mParentPoint, step.mBeamMin, step.mBeamMax);
	}

	template <CornerIndex cornerIndex>
	void PathFinder::enqueueDiag(CellKey cellKey, Point parentPoint, Cost costToParent,
		CellKey toCellKey, Point toPoint, uint8_t closedSetEdges)
	{
		toCellKey = mHierarchy->topLevelCellContainingCorner(toCellKey, cornerIndex);
		if(isEmptyCell(mHierarchy->cellAt(toCellKey)))
		{
			return;
		}

		if(!mClosedSet.pointTraversed(toCellKey, toPoint))
		{
			bool offGrid = toCellKey.cornerGood(cornerIndex) != toPoint;

			Step nextStep;
			nextStep.mStepType = offGrid ? StepType::DIAG_OFF_GRID : StepType::DIAG;
			nextStep.mCornerIndex = cornerIndex;
			nextStep.mCellKey = toCellKey;
			nextStep.mClosedSetEdges = closedSetEdges;
			nextStep.mClosedSetCellKey = cellKey;
			nextStep.mParentPoint = parentPoint;
			nextStep.mPoint = toPoint;
			nextStep.mTraversedCost = costToParent + Cost::distance(parentPoint, toPoint);
			validateStep(nextStep);
			mOpenSet.push(nextStep);
		}
	}

	template <CornerIndex cornerIndex, int8_t axis>
	void PathFinder::enqueueBeam(const Step& step, Point parentPoint, int16_t beamMin, int16_t beamMax)
	{
		int8_t cornerX = (int8_t)cornerIndex & 1;
		int8_t cornerY = (int8_t)cornerIndex >> 1;
		int8_t cornerOnAxis = ((int8_t)cornerIndex >> axis) & 1;
		constexpr EdgeIndex beamEdge = edgeFromCorner(cornerIndex, axis);

		CellKey nextCellKey = step.mCellKey;
		nextCellKey.mCoords[axis] += 1 - 2 * cornerOnAxis;

		Cell nextCell = mHierarchy->cellAt(nextCellKey);
		if(nextCell == Cell::PARTIAL)
		{
			Hierarchy::BoundaryCellIterator<beamEdge, OnEdgeDir::TOWARDS_POSITIVE> it(
				mHierarchy, nextCellKey, beamMin);
			while(it.moveNext())
			{
				CellKey childCellKey = it.cell();
				int16_t childMin = childCellKey.mCoords[axis ^ 1] << childCellKey.mLevel;
				if(childMin > beamMax)
				{
					break;
				}

				int16_t childMax = childMin + (1 << childCellKey.mLevel) - 1;

				enqueueBeamCell<cornerIndex, axis>(step, parentPoint, childCellKey, 
					std::max(beamMin, childMin), std::min(beamMax, childMax));
			}
		}
		else if(isFullCell(nextCell))
		{
			while(isLevelUpCell(nextCell))
			{
				nextCellKey.mCoords >>= 1;
				nextCellKey.mLevel++;
				nextCell = mHierarchy->cellAt(nextCellKey);
			}

			enqueueBeamCell<cornerIndex, axis>(step, parentPoint, nextCellKey, beamMin, beamMax);
		}
	}

	template <CornerIndex cornerIndex, int8_t axis>
	void PathFinder::enqueueBeamCell(const Step& step, Point parentPoint, CellKey nextCellKey, int16_t beamMin, int16_t beamMax)
	{
		constexpr int8_t perpAxis = axis ^ 1;
		constexpr int8_t cornerOnPerpAxis = ((int8_t)cornerIndex >> perpAxis) & 1;

		int16_t cellMin = nextCellKey.mCoords[perpAxis] << nextCellKey.mLevel;
		int16_t cellMax = cellMin + (1 << nextCellKey.mLevel) - 1;

		DIDA_ASSERT(beamMin >= cellMin);
		DIDA_ASSERT(beamMax <= cellMax);

		Point point = nextCellKey.cornerGood(cornerIndex);
		if(cornerOnPerpAxis == 0)
			point[perpAxis] = beamMin;
		else
			point[perpAxis] = beamMax;

		Cell nextCell = mHierarchy->cellAt(nextCellKey);

		if(nextCell == Cell::FULL)
		{
			if(!mClosedSet.pointTraversed(nextCellKey, point))
			{
				Step nextStep;
				nextStep.mStepType = axis == 0 ? StepType::BEAM_X : StepType::BEAM_Y;
				nextStep.mCornerIndex = cornerIndex;
				nextStep.mCellKey = nextCellKey;
				nextStep.mClosedSetEdges = 0;
				nextStep.mClosedSetCellKey = step.mCellKey;
				nextStep.mPoint = point;
				nextStep.mParentPoint = parentPoint;
				nextStep.mBeamMin = beamMin;
				nextStep.mBeamMax = beamMax;
				nextStep.mTraversedCost = step.mTraversedCost + Cost::distance(step.mPoint, point);
				validateStep(nextStep);
				mOpenSet.push(nextStep);
			}
		}
	}

	template <CornerIndex cornerIndex, int8_t axis>
	void PathFinder::enqueueShore(const Step& step)
	{
		constexpr int8_t perpAxis = axis ^ 1;
		constexpr int8_t cornerOnAxis = ((int8_t)cornerIndex >> axis) & 1;
		constexpr int8_t cornerOnPerpAxis = ((int8_t)cornerIndex >> perpAxis) & 1;

		constexpr EdgeIndex traversedEdge = (EdgeIndex)((cornerOnPerpAxis << 1) + perpAxis);
		constexpr EdgeIndex oppTraversedEdge = (EdgeIndex)((int8_t)traversedEdge ^ 2);

		DIDA_ASSERT(cornerOnEdge(cornerIndex, traversedEdge));

		CellKey neighborCellKey = step.mCellKey;
		neighborCellKey.mCoords[perpAxis] += 2 * cornerOnPerpAxis - 1;

		if(!mHierarchy->edgeTraversable<oppTraversedEdge>(neighborCellKey))
			return;

		int16_t len = 1 << step.mCellKey.mLevel;

		Point point = step.mPoint;
		if(cornerOnAxis == 0)
			point[axis] += len;
		else
			point[axis] -= len;

		CellKey nextCellKey = step.mCellKey;
		nextCellKey.mCoords[axis] += 1 - 2 * cornerOnAxis;
		nextCellKey = mHierarchy->topLevelCellContainingCorner(nextCellKey, cornerIndex);

		if(!mClosedSet.pointTraversed(nextCellKey, point))
		{
			Step nextStep;
			nextStep.mStepType = (StepType)((int)StepType::CORNER_MIN_X_MIN_Y + (int)cornerIndex);
			nextStep.mCellKey = nextCellKey;
			nextStep.mClosedSetEdges = (uint8_t)(1 << (int)traversedEdge);
			nextStep.mClosedSetCellKey = step.mCellKey;
			nextStep.mPoint = point;
			nextStep.mParentPoint = step.mPoint;
			nextStep.mTraversedCost = step.mTraversedCost + Cost(len, 0);
			validateStep(nextStep);
			mOpenSet.push(nextStep);
		}
	}

	template <CornerIndex cornerIndex, int8_t beamAxis>
	void PathFinder::enqueueSideEdge(CellKey cellKey, Point parentPoint, Cost costToParent)
	{
		constexpr int8_t sideEdgeAxis = beamAxis ^ 1;
		constexpr int8_t sideEdgeSide = ((int8_t)cornerIndex >> sideEdgeAxis) & 1;
		constexpr OnEdgeDir beamDir = (((int8_t)cornerIndex >> beamAxis) & 1) ? OnEdgeDir::TOWARDS_NEGATIVE : OnEdgeDir::TOWARDS_POSITIVE;

		constexpr CornerIndex oppositeCorner = (CornerIndex)((int8_t)cornerIndex ^ (1 << sideEdgeAxis));
		
		CellKey neighborCellKey = cellKey;
		neighborCellKey.mCoords[sideEdgeAxis] += 2 * sideEdgeSide - 1;

		Cell neighborCell = mHierarchy->cellAt(neighborCellKey);

		bool prevEmpty = false;

		int16_t len = 0;
		if(neighborCell == Cell::PARTIAL)
		{
			Hierarchy::BoundaryCellIterator<edgeFromCorner(oppositeCorner, sideEdgeAxis),
				edgeDirFromCorner(oppositeCorner, sideEdgeAxis)> it(mHierarchy, neighborCellKey);
			while(it.moveNext())
			{
				CellKey diagCellKey = it.cell();
				Cell diagCell = mHierarchy->cellAt(diagCellKey);
				if(diagCell == Cell::FULL)
				{
					if(prevEmpty)
					{
						Point point = cellKey.cornerGood(cornerIndex);
						point[sideEdgeAxis] += 2 * sideEdgeSide - 1;
						if(beamDir == OnEdgeDir::TOWARDS_POSITIVE)
							point[beamAxis] += len;
						else
							point[beamAxis] -= len;

						if(!mClosedSet.pointTraversed(diagCellKey, point))
						{
							Step nextStep;
							nextStep.mStepType = StepType::DIAG;
							nextStep.mCornerIndex = oppositeCorner;
							nextStep.mCellKey = diagCellKey;
							nextStep.mClosedSetEdges = 0;
							nextStep.mPoint = point;
							nextStep.mParentPoint = parentPoint;
							nextStep.mTraversedCost = costToParent + Cost(len, 0);
							validateStep(nextStep);
							mOpenSet.push(nextStep);
						}

						prevEmpty = false;
					}
				}
				else
				{
					DIDA_ASSERT(diagCell == Cell::EMPTY);
					prevEmpty = true;
				}

				len += 1 << diagCellKey.mLevel;
			}
		}
		else
		{
			prevEmpty = isEmptyCell(neighborCell);
			len += 1 << cellKey.mLevel;
		}

		if(prevEmpty)
		{
			CellKey diagCellKey = neighborCellKey;
			if(beamDir == OnEdgeDir::TOWARDS_POSITIVE)
				diagCellKey.mCoords[beamAxis]++;
			else
				diagCellKey.mCoords[beamAxis]--;
			
			diagCellKey = mHierarchy->topLevelCellContainingCorner(diagCellKey, oppositeCorner);

			if(mHierarchy->cellAt(diagCellKey) == Cell::FULL)
			{
				Point point = cellKey.cornerGood(cornerIndex);
				point[sideEdgeAxis] += 2 * sideEdgeSide - 1;
				if(beamDir == OnEdgeDir::TOWARDS_POSITIVE)
					point[beamAxis] += len;
				else
					point[beamAxis] -= len;

				if(!mClosedSet.pointTraversed(diagCellKey, point))
				{
					bool offGrid = diagCellKey.cornerGood(oppositeCorner) != point;

					Step nextStep;
					nextStep.mStepType = offGrid ? StepType::DIAG_OFF_GRID : StepType::DIAG;
					nextStep.mCornerIndex = oppositeCorner;
					nextStep.mCellKey = diagCellKey;
					nextStep.mClosedSetEdges = 0;
					nextStep.mPoint = point;
					nextStep.mParentPoint = parentPoint;
					nextStep.mTraversedCost = costToParent + Cost(len, 0);
					validateStep(nextStep);
					mOpenSet.push(nextStep);
				}
			}
		}
	}

	template <CornerIndex cornerIndex>
	void CornerConnectionInfo<cornerIndex>::init(
		const Hierarchy& hierarchy, Point cornerPt, CellKey cellKey)
	{
		int8_t cornerX = (int8_t)cornerIndex & 1;
		int8_t cornerY = (int8_t)cornerIndex >> 1;

		mDiagStartPt = cornerPt;

		Point cellFromCornerPt = cellKey.cornerGood(cornerIndex);
		if(cellFromCornerPt != cornerPt)
		{
			Point cellToCornerPt = cellFromCornerPt;
			cellToCornerPt.mX += (1 - 2 * cornerX) << cellKey.mLevel;
			cellToCornerPt.mY += (1 - 2 * cornerY) << cellKey.mLevel;

			int16_t xDiff = cellToCornerPt.mX - mDiagStartPt.mX;
			int16_t yDiff = cellToCornerPt.mY - mDiagStartPt.mY;

			mNextCellKey = cellKey;
			if(cellFromCornerPt.mX == mDiagStartPt.mX)
			{
				mNextCellKey.mCoords.mY += 1 - 2 * cornerY;

				mDiagLen = std::abs(yDiff);
				if(xDiff > 0)
					mDiagEndPt.mX = mDiagStartPt.mX + mDiagLen;
				else
					mDiagEndPt.mX = mDiagStartPt.mX - mDiagLen;

				mDiagEndPt.mY = cellToCornerPt.mY;

				mNextCellKey = hierarchy.topLevelCellContainingEdgePoint<
					yEdgeFromCorner(cornerIndex), yEdgeDirFromCorner(cornerIndex)>(
					mNextCellKey, mDiagEndPt);

				mXBeamTouchesSideEdge = false;
				mYBeamTouchesSideEdge = true;
			}
			else
			{
				DIDA_ASSERT(cellFromCornerPt.mY == mDiagStartPt.mY);

				mNextCellKey.mCoords.mX += 1 - 2 * cornerX;

				mDiagEndPt.mX = cellToCornerPt.mX;

				mDiagLen = std::abs(xDiff);
				if(yDiff > 0)
					mDiagEndPt.mY = mDiagStartPt.mY + mDiagLen;
				else
					mDiagEndPt.mY = mDiagStartPt.mY - mDiagLen;

				mNextCellKey = hierarchy.topLevelCellContainingEdgePoint<
					xEdgeFromCorner(cornerIndex), xEdgeDirFromCorner(cornerIndex)>(
					mNextCellKey, mDiagEndPt);

				mXBeamTouchesSideEdge = true;
				mYBeamTouchesSideEdge = false;
			}
		}
		else
		{
			mNextCellKey = cellKey;
			mNextCellKey.mCoords.mX += 1 - 2 * cornerX;
			mNextCellKey.mCoords.mY += 1 - 2 * cornerY;

			mDiagEndPt = mNextCellKey.cornerGood(cornerIndex);
			mNextCellKey = hierarchy.topLevelCellContainingCorner(mNextCellKey, cornerIndex);

			mDiagLen = 1 << cellKey.mLevel;

			mXBeamTouchesSideEdge = true;
			mYBeamTouchesSideEdge = true;
		}

		mXBeamCellKey = cellKey;
		mXBeamCellKey.mCoords.mX += 1 - 2 * cornerX;
		if(mDiagStartPt.mY < mDiagEndPt.mY)
		{
			mXBeamMin = mDiagStartPt.mY;
			mXBeamMax = mDiagEndPt.mY - 1;
		}
		else
		{
			DIDA_ASSERT(mDiagStartPt.mY > mDiagEndPt.mY);
			mXBeamMin = mDiagEndPt.mY + 1;
			mXBeamMax = mDiagStartPt.mY;
		}

		mYBeamCellKey = cellKey;
		mYBeamCellKey.mCoords.mY += 1 - 2 * cornerY;
		if(mDiagStartPt.mX < mDiagEndPt.mX)
		{
			mYBeamMin = mDiagStartPt.mX;
			mYBeamMax = mDiagEndPt.mX - 1;
		}
		else
		{
			DIDA_ASSERT(mDiagStartPt.mX > mDiagEndPt.mX);
			mYBeamMin = mDiagEndPt.mX + 1;
			mYBeamMax = mDiagStartPt.mX;
		}

		mNextOnGridCellKey = cellKey;
		mNextOnGridCellKey.mCoords.mX += 1 - 2 * cornerX;
		mNextOnGridCellKey.mCoords.mY += 1 - 2 * cornerY;

		mOnGridPoint = mNextCellKey.cornerGood(cornerIndex);
		mNextOnGridCellKey = hierarchy.topLevelCellContainingCorner(mNextOnGridCellKey, cornerIndex);
	}

	void PathFinder::validateStep(const Step& step) const
	{
		int axis = (int)step.mStepType - (int)StepType::BEAM_X;
		int perpAxis = axis ^ 1;

		Point cellMin = step.mCellKey.cornerGood(CornerIndex::MIN_X_MIN_Y);
		Point cellMax = step.mCellKey.cornerGood(CornerIndex::MAX_X_MAX_Y);
		Point cornerPt = step.mCellKey.cornerGood(step.mCornerIndex);

		// step.mPoint must lie inside the cell.
		DIDA_ASSERT(step.mPoint.mX >= cellMin.mX && step.mPoint.mX <= cellMax.mX);
		DIDA_ASSERT(step.mPoint.mY >= cellMin.mY && step.mPoint.mY <= cellMax.mY);

		switch(step.mStepType)
		{
		case StepType::DIAG:
			DIDA_ASSERT(cornerPt == step.mPoint);
			break;

		case StepType::DIAG_OFF_GRID:
			DIDA_ASSERT(cornerPt != step.mPoint);
			break;

		case StepType::BEAM_X:
		case StepType::BEAM_Y:
			DIDA_ASSERT(step.mPoint[axis] == cornerPt[axis]);
			DIDA_ASSERT(step.mBeamMin >= cellMin[perpAxis]);
			DIDA_ASSERT(step.mBeamMax <= cellMax[perpAxis]);
			break;
		}
	}
}