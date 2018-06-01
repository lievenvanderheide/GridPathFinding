#include "HierarchyPathfinder.h"

namespace Hierarchy
{
	PathFinder::IterationRes PathFinder::begin(const CellAndCorner* roots, int numRoots)
	{
		DIDA_ASSERT(numRoots > 0);

		for(int i = 0; i < numRoots; i++)
		{
			Step step;
			step.mStepType = StepType::DIAGONAL;
			step.mCellKey = roots[i].mCell;
			step.mCorner = roots[i].mCorner;
			step.mParent = Point::invalidPoint();
			step.mTraversedCost = Cost(0, 0);
			mOpenSet.push(step);
		}

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

			Point pt = step.mCellKey.corner(step.mCorner);
			auto it = mClosedSet.find(pt);
			if(it == mClosedSet.end())
			{
				ClosedSetElem closedSetElem;
				closedSetElem.mParent = step.mParent;
				mClosedSet.insert(std::make_pair(pt, closedSetElem));
				break;
			}
		}

		if(step.mStepType == StepType::DIAGONAL)
		{
			stepDiagonal(step, debugDraw);
		}
		else
		{
			stepBeam(step, debugDraw);
		}

		return IterationRes::IN_PROGRESS;
	}

	void PathFinder::stepDiagonal(const Step& step, DebugDraw* debugDraw)
	{
		stepCornerToCorner(step, debugDraw);

		CellKey bigCellKey = step.mCellKey;
		while(mHierarchy->cellAt(bigCellKey) == Cell::LEVEL_UP_FULL)
		{
			bigCellKey.mCoords >>= 1;
			bigCellKey.mLevel++;
		}

		int corner = (int)step.mStepType;
		int cornerX = corner & 1;
		int cornerY = corner >> 1;

		switch(step.mCorner)
		{
		case CornerIndex::MIN_X_MIN_Y:
			stepBeamSideEdge(step, bigCellKey, EdgeIndex::MIN_X, SideEdgeDir::TOWARDS_POSITIVE, debugDraw);
			stepBeamSideEdge(step, bigCellKey, EdgeIndex::MIN_Y, SideEdgeDir::TOWARDS_POSITIVE, debugDraw);
			stepBeamFrontEdge(step, bigCellKey, EdgeIndex::MAX_X, debugDraw);
			stepBeamFrontEdge(step, bigCellKey, EdgeIndex::MAX_Y, debugDraw);
			break;

		case CornerIndex::MAX_X_MIN_Y:
			stepBeamSideEdge(step, bigCellKey, EdgeIndex::MAX_X, SideEdgeDir::TOWARDS_POSITIVE, debugDraw);
			stepBeamSideEdge(step, bigCellKey, EdgeIndex::MIN_Y, SideEdgeDir::TOWARDS_NEGATIVE, debugDraw);
			stepBeamFrontEdge(step, bigCellKey, EdgeIndex::MIN_X, debugDraw);
			stepBeamFrontEdge(step, bigCellKey, EdgeIndex::MAX_Y, debugDraw);
			break;

		case CornerIndex::MIN_X_MAX_Y:
			stepBeamSideEdge(step, bigCellKey, EdgeIndex::MIN_X, SideEdgeDir::TOWARDS_NEGATIVE, debugDraw);
			stepBeamSideEdge(step, bigCellKey, EdgeIndex::MAX_Y, SideEdgeDir::TOWARDS_POSITIVE, debugDraw);
			stepBeamFrontEdge(step, bigCellKey, EdgeIndex::MAX_X, debugDraw);
			stepBeamFrontEdge(step, bigCellKey, EdgeIndex::MIN_Y, debugDraw);
			break;

		case CornerIndex::MAX_X_MAX_Y:
			stepBeamSideEdge(step, bigCellKey, EdgeIndex::MAX_X, SideEdgeDir::TOWARDS_NEGATIVE, debugDraw);
			stepBeamSideEdge(step, bigCellKey, EdgeIndex::MAX_Y, SideEdgeDir::TOWARDS_NEGATIVE, debugDraw);
			stepBeamFrontEdge(step, bigCellKey, EdgeIndex::MIN_X, debugDraw);
			stepBeamFrontEdge(step, bigCellKey, EdgeIndex::MIN_Y, debugDraw);
			break;
		}
	}

	void PathFinder::stepBeam(const Step& step, DebugDraw* debugDraw)
	{
		CellKey bigCellKey = step.mCellKey;
		while(isLevelUpCell(mHierarchy->cellAt(bigCellKey)))
		{
			bigCellKey.mCoords >>= 1;
			bigCellKey.mLevel++;
		}

		switch(step.mStepType)
		{
		case StepType::BEAM_MIN_X:
			stepBeamFrontEdge(step, bigCellKey, EdgeIndex::MIN_X, debugDraw);
			if(step.mCorner == CornerIndex::MAX_X_MIN_Y)
			{
				stepBeamSideEdge(step, bigCellKey, EdgeIndex::MIN_Y, SideEdgeDir::TOWARDS_NEGATIVE, debugDraw);
			}
			else
			{
				DIDA_ASSERT(step.mCorner == CornerIndex::MAX_X_MAX_Y);
				stepBeamSideEdge(step, bigCellKey, EdgeIndex::MAX_Y, SideEdgeDir::TOWARDS_NEGATIVE, debugDraw);
			}
			break;

		case StepType::BEAM_MIN_Y:
			stepBeamFrontEdge(step, bigCellKey, EdgeIndex::MIN_Y, debugDraw);
			if(step.mCorner == CornerIndex::MIN_X_MAX_Y)
			{
				stepBeamSideEdge(step, bigCellKey, EdgeIndex::MIN_X, SideEdgeDir::TOWARDS_NEGATIVE, debugDraw);
			}
			else
			{
				DIDA_ASSERT(step.mCorner == CornerIndex::MAX_X_MAX_Y);
				stepBeamSideEdge(step, bigCellKey, EdgeIndex::MAX_X, SideEdgeDir::TOWARDS_NEGATIVE, debugDraw);
			}
			break;

		case StepType::BEAM_MAX_X:
			stepBeamFrontEdge(step, bigCellKey, EdgeIndex::MAX_X, debugDraw);
			if(step.mCorner == CornerIndex::MIN_X_MIN_Y)
			{
				stepBeamSideEdge(step, bigCellKey, EdgeIndex::MIN_Y, SideEdgeDir::TOWARDS_POSITIVE, debugDraw);
			}
			else
			{
				DIDA_ASSERT(step.mCorner == CornerIndex::MIN_X_MAX_Y);
				stepBeamSideEdge(step, bigCellKey, EdgeIndex::MAX_Y, SideEdgeDir::TOWARDS_POSITIVE, debugDraw);
			}
			break;

		case StepType::BEAM_MAX_Y:
			stepBeamFrontEdge(step, bigCellKey, EdgeIndex::MAX_Y, debugDraw);
			if(step.mCorner == CornerIndex::MIN_X_MIN_Y)
			{
				stepBeamSideEdge(step, bigCellKey, EdgeIndex::MIN_X, SideEdgeDir::TOWARDS_POSITIVE, debugDraw);
			}
			else
			{
				DIDA_ASSERT(step.mCorner == CornerIndex::MAX_X_MIN_Y);
				stepBeamSideEdge(step, bigCellKey, EdgeIndex::MAX_X, SideEdgeDir::TOWARDS_POSITIVE, debugDraw);
			}
			break;
		}
	}

	void PathFinder::stepCornerToCorner(const Step& step, DebugDraw* debugDraw)
	{
		int8_t corner = (int8_t)step.mCorner;
		int8_t cornerX = corner & 1;
		int8_t cornerY = corner >> 1;
		
		CellKey nextCellKey = step.mCellKey;
		nextCellKey.mCoords.mX += 1 - 2 * cornerX;
		nextCellKey.mCoords.mY += 1 - 2 * cornerY;
			
		Cell nextCell = mHierarchy->cellAt(nextCellKey);
		if(nextCell == Cell::LEVEL_UP_FULL)
		{
			// Loop while we can level up, and while leveling up leaves the
			// corner in the same place (which is the case when the corner's
			// cooredinates are even in the current level).
			while(nextCell == Cell::LEVEL_UP_FULL &&
				((nextCellKey.mCoords.mX + cornerX) | (nextCellKey.mCoords.mY + cornerY) & 1) == 0)
			{
				nextCellKey.mCoords >>= 1;
				nextCellKey.mLevel++;
				nextCell = mHierarchy->cellAt(nextCellKey);
			}
		}
		else
		{
			while(nextCell == Cell::PARTIAL)
			{
				nextCellKey.mCoords <<= 1;
				nextCellKey.mCoords.mX += cornerX;
				nextCellKey.mCoords.mY += cornerY;
				nextCellKey.mLevel--;
				nextCell = mHierarchy->cellAt(nextCellKey);
			}

			if(isEmptyCell(nextCell))
			{
				if(debugDraw)
				{
					debugDraw->drawLine(
						step.mCellKey.corner(step.mCorner),
						nextCellKey.corner(step.mCorner));
				}

				return;
			}
		}

		Step nextStep;
		nextStep.mStepType = StepType::DIAGONAL;
		nextStep.mCellKey = nextCellKey;
		nextStep.mCorner = step.mCorner;
		nextStep.mParent = step.mCellKey.corner(step.mCorner);
		nextStep.mTraversedCost = step.mTraversedCost + Cost(0, 1 << step.mCellKey.mLevel);
		mOpenSet.push(nextStep);

		if(debugDraw)
		{
			debugDraw->drawLine(nextStep.mParent, nextStep.mCellKey.corner(nextStep.mCorner));
		}
	}

	void PathFinder::stepBeamSideEdge(const Step& step, CellKey bigCellKey, EdgeIndex edge, SideEdgeDir dir, DebugDraw* debugDraw)
	{
		int8_t edgeAxis = (int8_t)edge & 1;
		int8_t edgeSide = (int8_t)edge >> 1;
		
		CellKey neighborCellKey = bigCellKey;
		neighborCellKey.mCoords[edgeAxis] += 2 * edgeSide - 1;

		int smallCoord = (step.mCellKey.mCoords[edgeAxis] + edgeSide) << step.mCellKey.mLevel;
		int bigCoord = (bigCellKey.mCoords[edgeAxis] + edgeSide) << bigCellKey.mLevel;
		if(smallCoord != bigCoord)
		{
			return;
		}

		Cell neighborCell = mHierarchy->cellAt(neighborCellKey);

		bool prevEmpty = false;
		int16_t len = 0;
		if(neighborCell == Cell::PARTIAL)
		{
			Hierarchy::BoundaryCellIterator it(mHierarchy, neighborCellKey, oppositeEdge(edge), dir == SideEdgeDir::TOWARDS_NEGATIVE);
			while(it.moveNext())
			{
				CellKey diagCellKey = it.cell();
				Cell diagCell = mHierarchy->cellAt(diagCellKey);
				if(diagCell == Cell::FULL)
				{
					if(prevEmpty)
					{
						Step nextStep;
						nextStep.mStepType = StepType::DIAGONAL;
						nextStep.mCellKey = diagCellKey;
						nextStep.mCorner = edgeStartCorner(oppositeEdge(edge), dir);
						nextStep.mParent = step.mCellKey.corner(step.mCorner);
						nextStep.mTraversedCost = step.mTraversedCost + Cost(len, 0);
						mOpenSet.push(nextStep);

						if(debugDraw)
						{
							debugDraw->drawLine(nextStep.mParent, nextStep.mCellKey.corner(nextStep.mCorner));
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
			len += 1 << bigCellKey.mLevel;
		}

		if(prevEmpty)
		{
			CellKey diagCellKey = neighborCellKey;
			diagCellKey.mCoords[edgeAxis ^ 1] += (int16_t)dir;
				
			Cell diagCell = mHierarchy->cellAt(diagCellKey);
			if(isFullCell(diagCell))
			{
				CornerIndex corner = edgeStartCorner(oppositeEdge(edge), dir);
				int8_t cornerX = (int8_t)corner & 1;
				int8_t cornerY = (int8_t)corner >> 1;

				while(diagCell == Cell::LEVEL_UP_FULL && 
					((diagCellKey.mCoords.mX + cornerX) | (diagCellKey.mCoords.mY + cornerY) & 1) == 0)
				{
					diagCellKey.mCoords >>= 1;
					diagCellKey.mLevel++;
				}

				Step nextStep;
				nextStep.mStepType = StepType::DIAGONAL;
				nextStep.mCellKey = diagCellKey;
				nextStep.mCorner = corner;
				nextStep.mParent = step.mCellKey.corner(step.mCorner);
				nextStep.mTraversedCost = step.mTraversedCost + Cost(len, 0);
				mOpenSet.push(nextStep);

				if(debugDraw)
				{
					debugDraw->drawLine(nextStep.mParent, nextStep.mCellKey.corner(nextStep.mCorner));
				}
			}
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
			
		if(isFullCell(nextCell))
		{
			Step nextStep;
			nextStep.mStepType = (StepType)((int)StepType::BEAM_MIN_X + (int)edge);
			nextStep.mCellKey = neighborCellKey;
			nextStep.mCorner = step.mCorner;
			nextStep.mParent = step.mCellKey.corner(step.mCorner);
			nextStep.mTraversedCost = step.mTraversedCost + Cost::distance(
				nextStep.mParent, nextStep.mCellKey.corner(nextStep.mCorner));
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
					nextStep.mStepType = (StepType)((int)StepType::BEAM_MIN_X + (int)edge);
					nextStep.mCellKey = childCellKey;
					nextStep.mCorner = step.mCorner;
					nextStep.mParent = step.mCellKey.corner(step.mCorner);
					nextStep.mTraversedCost = step.mTraversedCost + Cost::distance(
						nextStep.mParent, nextStep.mCellKey.corner(nextStep.mCorner));
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
}