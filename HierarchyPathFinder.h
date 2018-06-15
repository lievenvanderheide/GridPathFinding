#pragma once

#include <queue>
#include <map>
#include <set>

#include "Hierarchy.h"
#include "ClosedSet.h"
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

	template <CornerIndex cornerIndex>
	struct CornerConnectionInfo
	{
		void init(const Hierarchy& hierarchy, Point cornerPt, CellKey cellKey);

		Point mDiagStartPt;
		Point mDiagEndPt;
		CellKey mNextCellKey;
		int16_t mDiagLen;

		CellKey mNextOnGridCellKey;
		Point mOnGridPoint;

		bool mXBeamTouchesSideEdge;
		bool mYBeamTouchesSideEdge;

		CellKey mXBeamCellKey;
		int16_t mXBeamMin;
		int16_t mXBeamMax;

		CellKey mYBeamCellKey;
		int16_t mYBeamMin;
		int16_t mYBeamMax;
	};

	class PathFinder
	{
	public:
		PathFinder(const Hierarchy* hierarchy);

		enum class IterationRes
		{
			IN_PROGRESS,
			END_REACHED,
			UNREACHABLE,
		};

		enum class StepType : uint8_t
		{
			DIAG,
			DIAG_OFF_GRID,
			BEAM_X,
			BEAM_Y,
		};

		struct Step
		{
			StepType mStepType : 3;
			CornerIndex mCornerIndex : 2;

			CellKey mCellKey;

			uint8_t mClosedSetEdges;
			CellKey mClosedSetCellKey;

			Point mPoint;
			Point mParentPoint;

			int16_t mBeamMin;
			int16_t mBeamMax;
			
			Cost mTraversedCost;

			bool operator < (const Step& b) const
			{
				return mTraversedCost > b.mTraversedCost;
			}
		};

		IterationRes begin(const CellAndCorner& root);
		IterationRes begin(Point startPoint, Point endPoint, DebugDraw* debugDraw);

		IterationRes iteration(DebugDraw* debugDraw);

	private:
		void stepDiag(const Step& step);

		template <CornerIndex cornerIndex>
		void stepDiagTempl(const Step& step);

		void stepDiagOffGrid(const Step& step);

		template <CornerIndex cornerIndex>
		void stepDiagOffGridTempl(const Step& step);

		void stepBeam(const Step& step);

		template <CornerIndex cornerIndex, int8_t axis>
		void stepBeamTempl(const Step& step);

		template <CornerIndex cornerIndex>
		void enqueueDiag(CellKey cellKey, Point parentPoint, Cost costToParent,
			CellKey toCellKey, Point toPoint, uint8_t closedSetEdges);

		template <CornerIndex cornerIndex, int8_t axis>
		void enqueueBeam(const Step& step, Point parentPoint, int16_t beamMin, int16_t beamMax);

		template <CornerIndex cornerIndex, int8_t axis>
		void enqueueBeamCell(const Step& step, Point parentPoint, CellKey nextCellKey, int16_t beamMin, int16_t beamMax);

		template <CornerIndex cornerIndex, int8_t beamAxis>
		void enqueueSideEdge(CellKey cellKey, Point parentPoint, Cost costToParent);

		template <CornerIndex cornerIndex, int8_t axis>
		void enqueueShore(const Step& step);

		void validateStep(const Step& step) const;
	
		Point mStartPoint;
		Point mEndPoint;

		CellKey mStartCellKey;
		CellKey mEndCellKey;

		std::priority_queue<Step> mOpenSet;

		ClosedSet mClosedSet;

		const Hierarchy* mHierarchy;
	};
}