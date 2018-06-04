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

		bool mTouchesXSideEdge;
		bool mTouchesYSideEdge;
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

		enum class StepType : int8_t
		{
			CORNER_MIN_X_MIN_Y,
			CORNER_MAX_X_MIN_Y,
			CORNER_MIN_X_MAX_Y,
			CORNER_MAX_X_MAX_Y,

			BEAM_FROM_MIN_X_SIDE_MIN_Y,
			BEAM_FROM_MIN_X_SIDE_MAX_Y,
			BEAM_FROM_MIN_Y_SIDE_MIN_X,
			BEAM_FROM_MIN_Y_SIDE_MAX_X,
			BEAM_FROM_MAX_X_SIDE_MIN_Y,
			BEAM_FROM_MAX_X_SIDE_MAX_Y,
			BEAM_FROM_MAX_Y_SIDE_MIN_X,
			BEAM_FROM_MAX_Y_SIDE_MAX_X,
		};

		struct Step
		{
			StepType mStepType;
			CellKey mCellKey;

			uint8_t mClosedSetEdges;
			CellKey mClosedSetCellKey;

			Point mPoint;
			Point mParentPoint;
			
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
		void stepCorner(const Step& step);

		template <CornerIndex cornerIndex>
		void stepCornerTempl(const Step& step);

		void stepBeam(const Step& step);

		template <EdgeIndex frontEdge, OnEdgeDir onFrontDir>
		void stepBeamTempl(const Step& step);

		template <CornerIndex cornerIndex>
		void enqueueDiag(const Step& step, const CornerConnectionInfo<cornerIndex>& connectionInfo);

		template <CornerIndex cornerIndex, int8_t axis>
		void enqueueShore(const Step& step);

		template <EdgeIndex edge, OnEdgeDir dir>
		void enqueueSideEdge(const Step& step);

		template <EdgeIndex frontEdge, OnEdgeDir onFrontDir>
		void enqueueBeamFullEdge(const Step& step);

		template <EdgeIndex frontEdge, OnEdgeDir onFrontDir>
		void enqueueOnGridBeam(const Step& step, CellKey cellKey, uint8_t closedSetEdges);

#if 0
		template <EdgeIndex frontEdge, OnEdgeDir onFrontDir>
		void enqueueBeam(const Step& step, Point diagStartPt, Point diagEndPt);

		template <EdgeIndex frontEdge, OnEdgeDir onFrontDir>
		void enqueueBeamToNeighbor(const Step& step, Point diagStartPt, Point diagEndPt, CellKey neighborCellKey);
#endif

		static EdgeIndex beamSideEdge(EdgeIndex frontEdge, OnEdgeDir onFrontDir);
		static StepType beamStepType(EdgeIndex toEdge, OnEdgeDir dir);
	
		Point mStartPoint;
		Point mEndPoint;

		CellKey mStartCellKey;
		CellKey mEndCellKey;

		std::priority_queue<Step> mOpenSet;

		ClosedSet mClosedSet;

		const Hierarchy* mHierarchy;
	};
}