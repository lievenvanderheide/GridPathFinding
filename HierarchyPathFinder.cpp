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
		step.mStepType = (StepType)((int)StepType::CORNER_MIN_X_MIN_Y + (int)root.mCorner);
		step.mCellKey = root.mCell;
		step.mClosedSetEdges = 0;
		step.mClosedSetCellKey = CellKey::invalidCellKey();
		step.mPoint = root.mCell.corner(root.mCorner);
		step.mParentPoint = Point::invalidPoint();
		step.mTraversedCost = Cost(0, 0);
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

				if(step.mStepType <= StepType::CORNER_MAX_X_MAX_Y)
				{	
					stepCorner(step);
					return IterationRes::IN_PROGRESS;
				}
				else if(step.mStepType <= StepType::BEAM_FROM_MAX_Y_SIDE_MAX_X)
				{	
					stepBeam(step);
					return IterationRes::IN_PROGRESS;
				}
			}
		}
	}

	void PathFinder::stepCorner(const Step& step)
	{
		DIDA_ASSERT(step.mStepType <= StepType::CORNER_MAX_X_MAX_Y);

		switch(step.mStepType)
		{
		case StepType::CORNER_MIN_X_MIN_Y:
			stepCornerTempl<CornerIndex::MIN_X_MIN_Y>(step);
			break;

		case StepType::CORNER_MAX_X_MIN_Y:
			stepCornerTempl<CornerIndex::MAX_X_MIN_Y>(step);
			break;

		case StepType::CORNER_MIN_X_MAX_Y:
			stepCornerTempl<CornerIndex::MIN_X_MAX_Y>(step);
			break;

		case StepType::CORNER_MAX_X_MAX_Y:
			stepCornerTempl<CornerIndex::MAX_X_MAX_Y>(step);
			break;
		}
	}

	template <CornerIndex cornerIndex>
	void PathFinder::stepCornerTempl(const Step& step)
	{
		DIDA_ASSERT(step.mStepType <= StepType::CORNER_MAX_X_MAX_Y);

		if(mHierarchy->cellAt(step.mCellKey) == Cell::FULL)
		{
			CornerConnectionInfo<cornerIndex> connectionInfo;
			connectionInfo.init(*mHierarchy, step.mPoint, step.mCellKey);

			enqueueDiag<cornerIndex>(step, connectionInfo);

			if(connectionInfo.mTouchesXSideEdge)
			{
				enqueueSideEdge<xEdgeFromCorner(cornerIndex), xEdgeDirFromCorner(cornerIndex)>(step);
			}

			if(connectionInfo.mTouchesYSideEdge)
			{
				enqueueSideEdge<yEdgeFromCorner(cornerIndex), yEdgeDirFromCorner(cornerIndex)>(step);
			}

			constexpr EdgeIndex beamXEdge = oppositeEdge(xEdgeFromCorner(cornerIndex));
			constexpr OnEdgeDir beamXEdgeDir = xEdgeDirFromCorner(cornerIndex);

			constexpr EdgeIndex beamYEdge = oppositeEdge(yEdgeFromCorner(cornerIndex));
			constexpr OnEdgeDir beamYEdgeDir = yEdgeDirFromCorner(cornerIndex);

			if(connectionInfo.mTouchesXSideEdge &&
				connectionInfo.mTouchesYSideEdge)
			{
				enqueueBeamFullEdge<beamXEdge, beamXEdgeDir>(step);
				enqueueBeamFullEdge<beamYEdge, beamYEdgeDir>(step);
			}
			else if(connectionInfo.mTouchesXSideEdge)
			{
			}
			else
			{
				DIDA_ASSERT(connectionInfo.mTouchesYSideEdge);
				DIDA_ASSERT(!"Not implemented yet");
			}
		}
		else
		{
			enqueueShore<cornerIndex, 0>(step);
			enqueueShore<cornerIndex, 1>(step);
		}
	}

	void PathFinder::stepBeam(const Step& step)
	{
		switch(step.mStepType)
		{
		case StepType::BEAM_FROM_MIN_X_SIDE_MIN_Y:
			stepBeamTempl<EdgeIndex::MAX_X, OnEdgeDir::TOWARDS_POSITIVE>(step);
			break;

		case StepType::BEAM_FROM_MIN_X_SIDE_MAX_Y:
			stepBeamTempl<EdgeIndex::MAX_X, OnEdgeDir::TOWARDS_NEGATIVE>(step);
			break;

		case StepType::BEAM_FROM_MIN_Y_SIDE_MIN_X:
			stepBeamTempl<EdgeIndex::MAX_Y, OnEdgeDir::TOWARDS_POSITIVE>(step);
			break;

		case StepType::BEAM_FROM_MIN_Y_SIDE_MAX_X:
			stepBeamTempl<EdgeIndex::MAX_Y, OnEdgeDir::TOWARDS_NEGATIVE>(step);
			break;

		case StepType::BEAM_FROM_MAX_X_SIDE_MIN_Y:
			stepBeamTempl<EdgeIndex::MIN_X, OnEdgeDir::TOWARDS_POSITIVE>(step);
			break;

		case StepType::BEAM_FROM_MAX_X_SIDE_MAX_Y:
			stepBeamTempl<EdgeIndex::MIN_X, OnEdgeDir::TOWARDS_NEGATIVE>(step);
			break;

		case StepType::BEAM_FROM_MAX_Y_SIDE_MIN_X:
			stepBeamTempl<EdgeIndex::MIN_Y, OnEdgeDir::TOWARDS_POSITIVE>(step);
			break;

		case StepType::BEAM_FROM_MAX_Y_SIDE_MAX_X:
			stepBeamTempl<EdgeIndex::MIN_Y, OnEdgeDir::TOWARDS_NEGATIVE>(step);
			break;

		default:
			DIDA_ASSERT(!"Not a beam step type.");
			break;
		}
	}

	template <EdgeIndex frontEdge, OnEdgeDir onFrontDir>
	void PathFinder::stepBeamTempl(const Step& step)
	{
		constexpr CornerIndex beamCorner = edgeStartCorner(oppositeEdge(frontEdge), onFrontDir);

		if(step.mCellKey.corner(beamCorner) == step.mPoint)
		{
			enqueueBeamFullEdge<frontEdge, onFrontDir>(step);
		}
		else
		{
			DIDA_ASSERT(!"Not implemented yet");
		}
	}

	template <CornerIndex cornerIndex>
	void PathFinder::enqueueDiag(const Step& step, const CornerConnectionInfo<cornerIndex>& connectionInfo)
	{
		if(!mClosedSet.pointTraversed(connectionInfo.mNextCellKey, connectionInfo.mDiagEndPt))
		{
			Step nextStep;
			nextStep.mStepType = (StepType)((int8_t)StepType::CORNER_MIN_X_MIN_Y + (int8_t)cornerIndex);
			nextStep.mCellKey = connectionInfo.mNextCellKey;
			nextStep.mClosedSetEdges = 0xf;
			nextStep.mClosedSetCellKey = step.mCellKey;
			nextStep.mParentPoint = connectionInfo.mDiagStartPt;
			nextStep.mPoint = connectionInfo.mDiagEndPt;
			nextStep.mTraversedCost = step.mTraversedCost + Cost(0, connectionInfo.mDiagLen);
			mOpenSet.push(nextStep);
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
			mOpenSet.push(nextStep);
		}
	}

	template <EdgeIndex sideEdge, OnEdgeDir sideDir>
	void PathFinder::enqueueSideEdge(const Step& step)
	{
		int8_t sideEdgeAxis = (int8_t)sideEdge & 1;
		int8_t sideEdgeSide = (int8_t)sideEdge >> 1;
		int8_t traverseAxis = sideEdgeAxis ^ 1;

		CornerIndex nextStepCorner = edgeStartCorner(oppositeEdge(sideEdge), sideDir);
		StepType nextStepType = (StepType)((int8_t)StepType::CORNER_MIN_X_MIN_Y + (int8_t)nextStepCorner);

		CellKey neighborCellKey = step.mCellKey;
		neighborCellKey.mCoords[sideEdgeAxis] += 2 * sideEdgeSide - 1;

		Cell neighborCell = mHierarchy->cellAt(neighborCellKey);

		bool prevEmpty = false;
		int16_t len = 0;
		if(neighborCell == Cell::PARTIAL)
		{
			Hierarchy::BoundaryCellIterator<oppositeEdge(sideEdge), sideDir> it(mHierarchy, neighborCellKey);
			while(it.moveNext())
			{
				CellKey diagCellKey = it.cell();
				Cell diagCell = mHierarchy->cellAt(diagCellKey);
				if(diagCell == Cell::FULL)
				{
					if(prevEmpty)
					{
						Point point = step.mPoint;
						if(sideDir == OnEdgeDir::TOWARDS_POSITIVE)
							point[traverseAxis] += len;
						else
							point[traverseAxis] -= len;

						if(!mClosedSet.pointTraversed(diagCellKey, point))
						{
							Step nextStep;
							nextStep.mStepType = nextStepType;
							nextStep.mCellKey = diagCellKey;
							nextStep.mClosedSetEdges = 0;
							nextStep.mPoint = point;
							nextStep.mParentPoint = step.mPoint;
							nextStep.mTraversedCost = step.mTraversedCost + Cost(len, 0);
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
			len += 1 << step.mCellKey.mLevel;
		}

		if(prevEmpty)
		{
			CellKey diagCellKey = neighborCellKey;
			if(sideDir == OnEdgeDir::TOWARDS_POSITIVE)
				diagCellKey.mCoords[traverseAxis]++;
			else
				diagCellKey.mCoords[traverseAxis]--;
			diagCellKey = mHierarchy->topLevelCellContainingCorner(diagCellKey, nextStepCorner);

			if(mHierarchy->cellAt(diagCellKey) == Cell::FULL)
			{
				Point point = step.mPoint;
				if(sideDir == OnEdgeDir::TOWARDS_POSITIVE)
					point[traverseAxis] += len;
				else
					point[traverseAxis] -= len;

				if(!mClosedSet.pointTraversed(diagCellKey, point))
				{
					Step nextStep;
					nextStep.mStepType = nextStepType;
					nextStep.mCellKey = diagCellKey;
					nextStep.mClosedSetEdges = 0;
					nextStep.mPoint = point;
					nextStep.mParentPoint = step.mPoint;
					nextStep.mTraversedCost = step.mTraversedCost + Cost(len, 0);
					mOpenSet.push(nextStep);
				}
			}
		}
	}

	template <EdgeIndex frontEdge, OnEdgeDir onFrontDir>
	void PathFinder::enqueueBeamFullEdge(const Step& step)
	{
		constexpr int8_t axis = (int8_t)frontEdge & 1;
		constexpr int8_t dir = (int8_t)frontEdge >> 1;
		constexpr CornerIndex beamCorner = edgeStartCorner(oppositeEdge(frontEdge), onFrontDir);

		CellKey neighborCellKey = step.mCellKey;
		neighborCellKey.mCoords[axis] += 2 * dir - 1;

		CellKey prevNeighborCellKey = mHierarchy->prevBoundaryCell<oppositeEdge(frontEdge), onFrontDir>(neighborCellKey);
		bool prevFull = mHierarchy->cellAt(prevNeighborCellKey) == Cell::FULL;

		Cell neighborCell = mHierarchy->cellAt(neighborCellKey);

		uint8_t closedSetEdges = 0;
		if((int8_t)beamCorner & 1)
			closedSetEdges |= (uint8_t)ClosedSet::EdgeFlags::MAX_X;
		else
			closedSetEdges |= (uint8_t)ClosedSet::EdgeFlags::MIN_X;

		if((int8_t)beamCorner >> 1)
			closedSetEdges |= (uint8_t)ClosedSet::EdgeFlags::MAX_Y;
		else
			closedSetEdges |= (uint8_t)ClosedSet::EdgeFlags::MIN_Y;

		if(isFullCell(neighborCell))
		{
			neighborCellKey = mHierarchy->topLevelCellContainingCorner(neighborCellKey, beamCorner);
			enqueueOnGridBeam<frontEdge, onFrontDir>(step, neighborCellKey, closedSetEdges);
		}
		else if(isEmptyCell(neighborCell))
		{
			if(prevFull)
			{
				DIDA_ASSERT(!"Not implemented yet");
			}
		}
		else if(neighborCell == Cell::PARTIAL)
		{
			Hierarchy::BoundaryCellIterator<oppositeEdge(frontEdge), onFrontDir> it(mHierarchy, neighborCellKey);
			while(it.moveNext())
			{
				Cell cell = mHierarchy->cellAt(it.cell());
				if(cell == Cell::FULL)
				{
					enqueueOnGridBeam<frontEdge, onFrontDir>(step, it.cell(), closedSetEdges);
					prevFull = true;
				}
				else if(prevFull)
				{
					DIDA_ASSERT(!"Not implemented yet");
					prevFull = false;
				}

				closedSetEdges = 0;
			}
		}
	}

	template <EdgeIndex frontEdge, OnEdgeDir onFrontDir>
	void PathFinder::enqueueOnGridBeam(const Step& step, CellKey cellKey, uint8_t closedSetEdges)
	{
		constexpr CornerIndex beamCorner = edgeStartCorner(oppositeEdge(frontEdge), onFrontDir);

		Point point = cellKey.corner(beamCorner);
		if(!mClosedSet.pointTraversed(cellKey, point))
		{
			Step nextStep;
			nextStep.mStepType = beamStepType(frontEdge, onFrontDir);
			nextStep.mCellKey = cellKey;
			nextStep.mClosedSetEdges = closedSetEdges;
			nextStep.mClosedSetCellKey = step.mCellKey;
			nextStep.mPoint = point;
			nextStep.mParentPoint = step.mPoint;
			nextStep.mTraversedCost = step.mTraversedCost + Cost::distance(step.mPoint, point);
			mOpenSet.push(nextStep);
		}
	}

#if 0
	template <EdgeIndex frontEdge, OnEdgeDir onFrontDir>
	void PathFinder::enqueueBeam(const Step& step, Point diagStartPt, Point diagEndPt)
	{
		int8_t axis = (int8_t)frontEdge & 1;
		int8_t dir = (int8_t)frontEdge >> 1;
		CornerIndex beamCorner = edgeStartCorner(oppositeEdge(frontEdge), onFrontDir);

		CellKey neighborCellKey = step.mCellKey;
		neighborCellKey.mCoords[axis] += 2 * dir - 1;

		Cell neighborCell = mHierarchy->cellAt(neighborCellKey);
		if(isFullCell(neighborCell))
		{
			neighborCellKey = mHierarchy->topLevelCellContainingCorner(neighborCellKey, beamCorner);
			enqueueBeamToNeighbor<frontEdge, onFrontDir>(step, diagStartPt, diagEndPt, neighborCellKey);
		}
		else if(neighborCell == Cell::PARTIAL)
		{
			Hierarchy::BoundaryCellIterator<oppositeEdge(frontEdge), onFrontDir> it(mHierarchy, neighborCellKey);
			while(it.moveNext())
			{
				enqueueBeamToNeighbor<frontEdge, onFrontDir>(step, diagStartPt, diagEndPt, it.cell());
			}
		}
	}
	
	template <EdgeIndex frontEdge, OnEdgeDir onFrontDir>
	void PathFinder::enqueueBeamToNeighbor(const Step& step, Point diagStartPt, Point diagEndPt, CellKey neighborCellKey)
	{
		int8_t axis = (int8_t)frontEdge & 1;
		int8_t dir = (int8_t)frontEdge >> 1;
		int8_t perpAxis = axis ^ 1;
		CornerIndex beamCorner = edgeStartCorner(oppositeEdge(frontEdge), onFrontDir);

		int16_t neighborMin = neighborCellKey.mCoords[perpAxis] << neighborCellKey.mLevel;
		int16_t neighborMax = (neighborCellKey.mCoords[perpAxis] + 1) << neighborCellKey.mLevel;
		if(onFrontDir == OnEdgeDir::TOWARDS_NEGATIVE)
			std::swap(neighborMin, neighborMax);

		if(onEdgeLt<onFrontDir>(neighborMax, diagStartPt[perpAxis]) ||
			onEdgeLt<onFrontDir>(diagEndPt[perpAxis], neighborMin))
		{
			return;
		}

		Point point = step.mPoint;
		point[axis] += (int16_t)(2 * dir - 1) << step.mCellKey.mLevel;

		uint8_t closedSetEdges = 0;
		if(onEdgeLt<onFrontDir>(neighborMin, diagStartPt[perpAxis]))
		{
			point[perpAxis] = step.mPoint[perpAxis];
		}
		else if(neighborMin == diagStartPt[perpAxis])
		{
			point[perpAxis] = step.mPoint[perpAxis];
			if(point == step.mCellKey.corner(edgeStartCorner(frontEdge, onFrontDir)))
			{
				closedSetEdges = 1 << (int)beamSideEdge(frontEdge, onFrontDir);
			}
		}
		else
		{
			point[perpAxis] = neighborMin;
		}

		if(mHierarchy->cellAt(neighborCellKey) == Cell::FULL)
		{
			if(!mClosedSet.pointTraversed(step.mCellKey, point))
			{
				Step nextStep;
				nextStep.mStepType = beamStepType(frontEdge, onFrontDir);
				nextStep.mCellKey = neighborCellKey;
				nextStep.mClosedSetEdges = closedSetEdges;
				nextStep.mClosedSetCellKey = step.mCellKey;
				nextStep.mPoint = point;
				nextStep.mParentPoint = step.mPoint;
				nextStep.mTraversedCost = step.mTraversedCost + Cost::distance(step.mPoint, point);
				mOpenSet.push(nextStep);
			}
		}
		else
		{
			DIDA_ASSERT(mHierarchy->cellAt(neighborCellKey) == Cell::EMPTY);
			DIDA_ASSERT(!"Not implemented yet");
		}
	}
#endif

	EdgeIndex PathFinder::beamSideEdge(EdgeIndex frontEdge, OnEdgeDir onFrontDir)
	{
		int8_t frontAxis = (int8_t)frontEdge & 1;
		int8_t edge = frontAxis ^ 1;
		if(onFrontDir == OnEdgeDir::TOWARDS_NEGATIVE)
			edge += 2;
		return (EdgeIndex)edge;
	}

	PathFinder::StepType PathFinder::beamStepType(EdgeIndex toEdge, OnEdgeDir dir)
	{
		int beamIdx = (int)oppositeEdge(toEdge) * 2;
		if(dir == OnEdgeDir::TOWARDS_NEGATIVE)
			beamIdx++;
		return (StepType)((int)StepType::BEAM_FROM_MIN_X_SIDE_MIN_Y + beamIdx);
	}

#if 0

	template <EdgeIndex edge, OnEdgeDir onEdgeDir> 
	void PathFinder::enqueueBeam(const Step& step, Point diagStartPt, Point diagEndPoint, uint8_t sideClosedSetEdges)
	{	
		int8_t edgeAxis = (int8_t)edge & 1;
		int8_t edgeSide = (int8_t)edge >> 1;
		int8_t perpAxis = edgeAxis ^ 1;

		StepType nextStepType = beamStepType(edge, step.mCorner);

		CellKey bigNeighborCellKey = step.mCellKey;
		bigNeighborCellKey.mCoords[edgeAxis] += 2 * edgeSide - 1;

		Point point = diagStartPt;
		point[edgeAxis] = (int16_t)(2 * edgeSide - 1) << step.mCellKey.mLevel;

		Hierarchy::BoundaryCellIterator<oppositeEdge(edge), onEdgeDir> it(
			mHierarchy, neighborCellKey, point[perpAxis]);

		DIDA_ON_DEBUG(bool hasFirstOne) = it.moveNext();
		DIDA_ASSERT(hasFirstOne);

		{
			CellKey neighborCellKey = it.cur();
			Cell neighborCell = mHierarchy->cellAt(neighborCellKey);

			int16_t cellBegin = neighborCellKey.mCoords[perpAxis];
			if(onEdgeDir == OnEdgeDir::TOWARDS_NEGATIVE)
				cellBegin++;
			cellBegin <<= neighborCellKey.mLevel;

			if(cellBegin == point[perpAxis])
			{
				CellKey prevNeighborCellKey = neighborCellKey;
				prevNeighborCellKey.mCoords[perpAxis] += onEdgeDir == OnEdgeDir::TOWARDS_POSITIVE ? -1 : 1;
				prevNeighborCellKey = mHierarchy->topLevelCellContainingCorner(
					prevNeighborCellKey, edgeStartCorner(edge, onEdgeDir));

				if(neighborCell == Cell::FULL ||
					(neighborCell == Cell::EMPTY && mHierarchy->cellAt(prevNeighborCellKey) == Cell::FULL))
				{
					if(!mHierarchy->isPointInSet(point))
					{
						Step nextStep;
						nextStep.mStepType = nextStepType;
						nextStep.mCellKey = neighborCellKey;
						nextStep.mClosedSetEdges = sideClosedSetEdges;
						nextStep.mClosedSetCellKey = step.mCellKey;
						nextStep.mPoint = point;
						nextStep.mParentPoint = step.mPoint;
						nextStep.mOffGrid = false;
						nextStep.mTraversedCost = step.mTraversedCost + Cost(1 << step.mCellKey.mLevel, 0);
						mOpenStep.push_back(nextStep);
					}
				}
			}
			else
			{
			}
		}


		bool prevEmpty = false;
		while(it.moveNext())
		{
			CellKey neighborCellKey = it.cur();
			Cell neighborCell = mHierarchy->cellAt(neighborCellKey);
			
			int16_t cellBegin = neighborCellKey.mCoords[perpAxis];
			if(onEdgeDir == OnEdgeDir::TOWARDS_NEGATIVE)
				cellBegin++;
			cellBegin <<= neighborCellKey.mLevel;

			if(cellBegin == point[perpAxis])
			{
				if(
					mHierarchy->isPointInSet(point))
				{
					Step nextStep;
					nextStep.mStepType = nextStepType;
					nextStep.mCellKey = neighborCellKey;
					nextStep.mClosedSetEdges = sideClosedSetEdges;
					nextStep.mClosedSetCellKey = step.mCellKey;
					nextStep.mPoint = point;
					nextStep.mParentPoint = step.mPoint;
					nextStep.mOffGrid = false;
					nextStep.mTraversedCost = step.mTraversedCost + Cost(1 << step.mCellKey.mLevel, 0);
					mOpenStep.push_back(nextStep);
				}
			}
			else
			{
				if(mHierarchy->isPointOnEdgeInSet(point))
				{
					Step nextStep;
					nextStep.mStepType = nextStepType;
					nextStep.mCellKey = neighborCellKey;
					nextStep.mClosedSetEdges = sideClosedSetEdges;
					nextStep.mClosedSetCellKey = step.mCellKey;
					nextStep.mPoint = point;
					nextStep.mParentPoint = step.mPoint;
					nextStep.mOffGrid = false;
					nextStep.mTraversedCost = step.mTraversedCost + Cost(1 << step.mCellKey.mLevel, 0);
					mOpenStep.push_back(nextStep);
				}
			}
		}
		
		bool touchesSideEdge = 
		/*CellKey prevNeighborCellKey = neighborCellKey;
		prevNeighborCellKey.mCoords[edgeAxis ^ 1] = onEdgeDir == OnEdgeDir::TOWARDS_POSITIVE ? -1 : 1;
		prevNeighborCellKey= mHierarchy->topLevelCellContainingCorner(edgeStartCorner(edge, onEdgeDir));*/

		Cell neighborCell = mHierarchy->cellAt(neighborCellKey);
		Cell prevNeighborCell = mHierarchy->cellAt(prevNeighborCellKey);

		

		if(isFullCell(neighborCell) ||
			(isEmptyCell(neighborCell) && isFullCell(prevNeighborCell)))
		{
			Point point = step.mPoint;
			point[edgeAxis] = neighborCellKey.mCoords[edgeAxis] + (edgeSide ^ 1)

			Step nextStep;
			nextStep.mStepType = nextStepType;
			nextStep.mCellKey = nextCellKey;
			nextStep.mClosedSetEdges = sideEdgeToClose;
			nextStep.mClosedSetCellKey = step.mCellKey;
			nextStep.mPoint;
			nextStep.mParentPoint = step.mPoint;

			bool mOffGrid;

			Cost mTraversedCost;

			nextStep.mStepType = nextStepType;
			nextStep.mCellKey = neighborCellKey;
			nextStep.mCorner = step.mCorner;
			nextStep.mParent = step.mCellKey.corner(step.mCorner);
			nextStep.mTraversedCost = step.mTraversedCost + Cost::distance(
				nextStep.mParent, nextStep.mCellKey.corner(beamEndCorner));
			mOpenSet.push(nextStep);
		}
		else if(nextCell == Cell::PARTIAL)
		{
			bool reverse = ((int8_t)step.mCorner & (2 >> edgeAxis)) != 0;
			Hierarchy::BoundaryCellIterator it(mHierarchy, neighborCellKey, oppositeEdge(edge), reverse);

			bool isFirst = true;
			while(it.moveNext())
			{
				CellKey childCellKey = it.cell();
				if(isFullCell(mHierarchy->cellAt(childCellKey)))
				{
					Step nextStep;
					nextStep.mStepType = nextStepType;
					nextStep.mCellKey = childCellKey;
					nextStep.mCorner = step.mCorner;
					nextStep.mParent = step.mCellKey.corner(step.mCorner);
					nextStep.mTraversedCost = step.mTraversedCost + Cost::distance(
						nextStep.mParent, nextStep.mCellKey.corner(beamEndCorner));
					mOpenSet.push(nextStep);

					if(debugDraw)
					{
						debugDraw->drawLine(nextStep.mParent, nextStep.mCellKey.corner(nextStep.mCorner));
					}
				}
			}
		}
		else
		{
			DIDA_ASSERT(isEmptyCell(nextCell));
		}
	}

#if 0
	void PathFinder::stepBeam(const Step& step, DebugDraw* debugDraw)
	{
		DIDA_ASSERT(!"Not implemented yet");

		/*DIDA_ASSERT(step.mStepType >= StepType::BEAM_TO_MIN_X_SIDE_MIN_Y);

		switch(step.mStepType)
		{
		case StepType::BEAM_TO_MIN_X_SIDE_MIN_Y:
			stepBeamFrontEdge(step, EdgeIndex::MIN_X, debugDraw);
			stepBeamSideEdge(step, EdgeIndex::MIN_Y, SideEdgeDir::TOWARDS_NEGATIVE, debugDraw);
			break;

		case StepType::BEAM_TO_MIN_X_SIDE_MAX_Y:
			stepBeamFrontEdge(step, EdgeIndex::MIN_X, debugDraw);
			stepBeamSideEdge(step, EdgeIndex::MAX_Y, SideEdgeDir::TOWARDS_NEGATIVE, debugDraw);
			break;

		case StepType::BEAM_TO_MIN_Y_SIDE_MIN_X:
			stepBeamFrontEdge(step, EdgeIndex::MIN_Y, debugDraw);
			stepBeamSideEdge(step, EdgeIndex::MIN_X, SideEdgeDir::TOWARDS_NEGATIVE, debugDraw);
			break;

		case StepType::BEAM_TO_MIN_Y_SIDE_MAX_X:
			stepBeamFrontEdge(step, EdgeIndex::MIN_X, debugDraw);
			stepBeamSideEdge(step, EdgeIndex::MAX_Y, SideEdgeDir::TOWARDS_NEGATIVE, debugDraw);
			break;

		case StepType::BEAM_TO_MAX_X_SIDE_MIN_Y:
			stepBeamFrontEdge(step, EdgeIndex::MAX_X, debugDraw);
			stepBeamSideEdge(step, EdgeIndex::MIN_Y, SideEdgeDir::TOWARDS_POSITIVE, debugDraw);
			break;

		case StepType::BEAM_TO_MAX_X_SIDE_MAX_Y:
			stepBeamFrontEdge(step, EdgeIndex::MAX_X, debugDraw);
			stepBeamSideEdge(step, EdgeIndex::MAX_Y, SideEdgeDir::TOWARDS_POSITIVE, debugDraw);
			break;

		case StepType::BEAM_TO_MAX_Y_SIDE_MIN_X:
			stepBeamFrontEdge(step, EdgeIndex::MAX_Y, debugDraw);
			stepBeamSideEdge(step, EdgeIndex::MIN_X, SideEdgeDir::TOWARDS_POSITIVE, debugDraw);
			break;

		case StepType::BEAM_TO_MAX_Y_SIDE_MAX_X:
			stepBeamFrontEdge(step, EdgeIndex::MAX_X, debugDraw);
			stepBeamSideEdge(step, EdgeIndex::MAX_Y, SideEdgeDir::TOWARDS_POSITIVE, debugDraw);
			break;
		}*/
	}

	template <CornerIndex fromCorner>
	void PathFinder::StepCornerContext<fromCorner>::enqueueNextDiag()
	{
		if(mPathFinder->mHierarchy->cellAt(mNextCellKey) != Cell::FULL)
		{
			return;
		}

		Point fromCornerPt = mNextCellKey.corner(fromCorner);
		Point toCornerPt = mNextCellKey.corner(oppositeCorner(fromCorner));

		mNextDiagOffGrid = fromCornerPt != mPoint;
		Cost cost;
		if(mNextDiagOffGrid)
		{
			

			cost = Cost::distance(mStepEndPoint, mNextDiagEndPoint);
		}
		else
		{
			mNextDiagTEndPoint = toCornerPt;

			if(mPathFinder->mClosedSet.isPointInClosedSet(mNextDiagEndPoint))
			{
				return;
			}

			cost = Cost(0, 1 << mNextCellKey.mLevel);
		}

		Step nextStep;
		nextStep.mStepType = (StepType)((uint8_t)StepType::BEAM_FROM_MIN_X_SIDE_MIN_Y + (uint8_t)fromCorner;
		nextStep.mCellKey = mNextCellKey;
		nextStep.mPoint = mStepEndPoint;
		nextStep.mParentPoint = mNextDiagEndPoint;
		nextStep.mOffGrid = mNextDiagOffGrid;
		nextStep.mTraversedCost = step.mTraversedCost + cost;
		mOpenSet.push(nextStep);
	}

	void PathFinder::stepDiag(const Step& step, CornerIndex fromCorner, DebugDraw* debugDraw)
	{
		if(mHierarchy->cellAt(nextCellKey) == Cell::EMPTY)
		{
			return;
		}
		
		bool nextOffGrid = nextCellKey.corner(fromCorner) != step.mPoint;
		Cell nextCell = mHierarchy->cellAt(nextCellKey);
		Point nextPoint;
		if(nextOffGrid)
		{	
			nextPoint = step.mPoint;
			if(nextPoint.mX == nextCellKey.mCoords.mX + (fromCornerX << nextCellKey.mLevel))
			{
				DIDA_ASSERT(!"Not implemented yet");
			}
			else
			{
				DIDA_ASSERT(nextPoint.mY == nextCellKey.mCoords.mY + (fromCornerX << nextCellKey.mLevel));
				DIDA_ASSERT(!"Not implemented yet");
			}

			if(mClosedSet.isPointOnEdgeInSet(nextPoint))
			{
				return;
			}
		}
		else
		{
			nextPoint = step.mPoint;
			nextPoint.mX += 1 << nextCellKey.mLevel;
			nextPoint.mY += 1 << nextCellKey.mLevel;

			if(mHierarchy->isPointInSet(nextPoint))
			{
				return;
			}
		}

		Step nextStep;
		nextStep.mStepType = (StepType)((int8_t)StepType::CORNER_MIN_X_MIN_Y + (int8_t)fromCorner);
		nextStep.mCellKey = nextCellKey;
		nextStep.mPoint = nextPoint;
		nextStep.mParentPoint = step.mPoint;
		nextStep.mOffGrid = nextOffGrid;
		nextStep.mTraversedCost = step.mTraversedCost + Cost(0, 1 << step.mCellKey.mLevel);
		mOpenSet.push(nextStep);

		if(debugDraw)
		{
			debugDraw->drawLine(nextStep.mParentPoint, nextStep.mPoint);
		}
	}

	void PathFinder::stepBeamFrontEdge(const Step& step, CellKey bigCellKey, EdgeIndex edge, DebugDraw* debugDraw)
	{	
		int8_t edgeAxis = (int8_t)edge & 1;
		int8_t edgeSide = (int8_t)edge >> 1;

		CellKey neighborCellKey = step.mCellKey;
		neighborCellKey.mCoords[edgeAxis] = (bigCellKey.mCoords[edgeAxis] + edgeSide) << 
			(bigCellKey.mLevel - neighborCellKey.mLevel);
		neighborCellKey.mCoords[edgeAxis] += edgeSide - 1;
		Cell nextCell = mHierarchy->cellAt(neighborCellKey);

		CornerIndex beamEndCorner = (CornerIndex)((int8_t)step.mCorner ^ (1 << edgeAxis));

		StepType nextStepType = beamStepType(edge, step.mCorner);
			
		if(isFullCell(nextCell))
		{
			Step nextStep;
			nextStep.mStepType = nextStepType;
			nextStep.mCellKey = neighborCellKey;
			nextStep.mCorner = step.mCorner;
			nextStep.mParent = step.mCellKey.corner(step.mCorner);
			nextStep.mTraversedCost = step.mTraversedCost + Cost::distance(
				nextStep.mParent, nextStep.mCellKey.corner(beamEndCorner));
			mOpenSet.push(nextStep);

			if(debugDraw)
			{
				debugDraw->drawLine(nextStep.mParent, nextStep.mCellKey.corner(nextStep.mCorner));
			}
		}
		else if(nextCell == Cell::PARTIAL)
		{
			bool reverse = ((int8_t)step.mCorner & (2 >> edgeAxis)) != 0;
			Hierarchy::BoundaryCellIterator it(mHierarchy, neighborCellKey, oppositeEdge(edge), reverse);

			bool isFirst = true;
			while(it.moveNext())
			{
				CellKey childCellKey = it.cell();
				if(isFullCell(mHierarchy->cellAt(childCellKey)))
				{
					Step nextStep;
					nextStep.mStepType = nextStepType;
					nextStep.mCellKey = childCellKey;
					nextStep.mCorner = step.mCorner;
					nextStep.mParent = step.mCellKey.corner(step.mCorner);
					nextStep.mTraversedCost = step.mTraversedCost + Cost::distance(
						nextStep.mParent, nextStep.mCellKey.corner(beamEndCorner));
					mOpenSet.push(nextStep);

					if(debugDraw)
					{
						debugDraw->drawLine(nextStep.mParent, nextStep.mCellKey.corner(nextStep.mCorner));
					}
				}
			}
		}
		else
		{
			DIDA_ASSERT(isEmptyCell(nextCell));
		}
	}

	EdgeIndex PathFinder::beamSideEdge(const Step& step)
	{
		const EdgeIndex table[] = 
		{
			EdgeIndex::MIN_Y, // BEAM_TO_MIN_X_SIDE_MIN_Y
			EdgeIndex::MAX_Y, // BEAM_TO_MIN_X_SIDE_MAX_Y
			EdgeIndex::MIN_X, // BEAM_TO_MIN_Y_SIDE_MIN_X
			EdgeIndex::MAX_X, // BEAM_TO_MIN_Y_SIDE_MAX_X
			EdgeIndex::MIN_Y, // BEAM_TO_MAX_X_SIDE_MIN_Y
			EdgeIndex::MAX_Y, // BEAM_TO_MAX_X_SIDE_MAX_Y
			EdgeIndex::MIN_X, // BEAM_TO_MAX_Y_SIDE_MIN_X
			EdgeIndex::MAX_X, // BEAM_TO_MAX_Y_SIDE_MAX_X
		};
		
		return table[(int)step.mStepType - (int)StepType::BEAM_TO_MIN_X_SIDE_MIN_Y];
	}

	PathFinder::StepType PathFinder::beamStepType(EdgeIndex toEdge, CornerIndex corner)
	{
		int8_t edgeAxis = (int8_t)toEdge & 1;
		int8_t edgeSide = (int8_t)toEdge >> 1;

		uint8_t ret = (int8_t)toEdge << 1;
		if(((int8_t)corner << edgeAxis) & 2)
		{
			ret++;
		}

		return (StepType)ret;
	}
#endif
#endif

	template <CornerIndex cornerIndex>
	void CornerConnectionInfo<cornerIndex>::init(
		const Hierarchy& hierarchy, Point cornerPt, CellKey cellKey)
	{
		int8_t cornerX = (int8_t)cornerIndex & 1;
		int8_t cornerY = (int8_t)cornerIndex >> 1;

		mDiagStartPt = cornerPt;

		Point cellFromCornerPt = cellKey.corner(cornerIndex);
		if(cellFromCornerPt != cornerPt)
		{	
			Point cellToCornerPt = cellKey.corner(oppositeCorner(cornerIndex));

			int16_t xDiff = cellToCornerPt.mY - cellFromCornerPt.mY;
			int16_t yDiff = cellToCornerPt.mX - cellFromCornerPt.mX;

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

				mTouchesXSideEdge = true;
				mTouchesYSideEdge = false;
			}
			else
			{
				DIDA_ASSERT(cellFromCornerPt.mY == mDiagStartPt.mY);

				mNextCellKey.mCoords.mX += 1 - 2 * cornerX;

				mDiagEndPt.mX = cellToCornerPt.mY;

				mDiagLen = std::abs(xDiff);
				if(yDiff > 0)
					mDiagEndPt.mY = mDiagStartPt.mY + mDiagLen;
				else
					mDiagEndPt.mY = mDiagStartPt.mY - mDiagLen;

				mNextCellKey = hierarchy.topLevelCellContainingEdgePoint<
					xEdgeFromCorner(cornerIndex), xEdgeDirFromCorner(cornerIndex)>(
					mNextCellKey, mDiagEndPt);
				
				mTouchesXSideEdge = false;
				mTouchesYSideEdge = true;
			}
		}
		else
		{
			mNextCellKey = cellKey;
			mNextCellKey.mCoords.mX += 1 - 2 * cornerX;
			mNextCellKey.mCoords.mY += 1 - 2 * cornerY;

			mDiagEndPt = mNextCellKey.corner(cornerIndex);
			mNextCellKey = hierarchy.topLevelCellContainingCorner(mNextCellKey, cornerIndex);

			mDiagLen = 1 << cellKey.mLevel;

			mTouchesXSideEdge = true;
			mTouchesYSideEdge = true;
		}
	}
}