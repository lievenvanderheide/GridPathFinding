#pragma once

#include <queue>
#include <map>
#include <set>

#include "Hierarchy.h"
#include "DebugDraw.h"

namespace Hierarchy
{
	struct CellAndCorner
	{
		CellKey mCell;
		CornerIndex mCorner;

		bool operator < (const CellAndCorner& b) const
		{
			if(mCell != b.mCell)
				return mCell < b.mCell;
			else
				return mCorner < b.mCorner;
		}
	};

	class PathFinder
	{
	public:
		enum class IterationRes
		{
			IN_PROGRESS,
			END_REACHED,
			UNREACHABLE,
		};

		enum class StepType : int8_t
		{
			DIAGONAL,
			BEAM_MIN_X,
			BEAM_MIN_Y,
			BEAM_MAX_X,
			BEAM_MAX_Y,
		};

		enum class ClosedSetMask : uint8_t
		{
			DIAG = 1,
			ANTI_DIAG = 2,
			BOUNDARY_MIN_X = 4,
			BOUNDARY_MIN_Y = 8,
			BOUNDARY_MAX_X = 16,
			BOUNDARY_MAX_Y = 32,
		};

		struct Step
		{
			StepType mStepType;
			CellKey mCellKey;
			CornerIndex mCorner;
			uint8_t mClosedSetMask;
			Point mParent;
			
			Cost mTraversedCost;

			bool operator < (const Step& b) const
			{
				return mTraversedCost > b.mTraversedCost;
			}
		};

		IterationRes begin(const CellAndCorner* roots, int numRoots);
		IterationRes begin(Point startPoint, Point endPoint, DebugDraw* debugDraw);

		IterationRes iteration(DebugDraw* debugDraw);

		void stepDiagonal(const Step& step, DebugDraw* debugDraw);
		void stepBeam(const Step& step, DebugDraw* debugDraw);

		void stepCornerToCorner(const Step& step, DebugDraw* debugDraw);

		void stepBeamFrontEdge(const Step& step, CellKey bigCellKey, EdgeIndex edge, DebugDraw* debugDraw);
		void stepBeamSideEdge(const Step& step, CellKey bigCellKey, EdgeIndex edge, SideEdgeDir dir, DebugDraw* debugDraw);
		
		Point mStartPoint;
		Point mEndPoint;

		CellKey mStartCellKey;
		CellKey mEndCellKey;

		std::priority_queue<Step> mOpenSet;

		struct ClosedSetElem
		{
			Point mParent;
			uint8_t mMask;
		};

		std::map<Point, ClosedSetElem> mClosedSet;

		const Hierarchy* mHierarchy;
	};
}