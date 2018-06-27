#pragma once

#include <map>

#include "Hierarchy.h"

namespace Hierarchy
{
	class ClosedSet
	{
	public:
		ClosedSet(const Hierarchy* hierarchy);

		bool pointTraversed(CellKey cellKey, Point pt) const;
		
		bool tryAddPoint(Point pt, Point parentPt);

		enum class EdgeFlags : uint8_t
		{
			MIN_X = 1,
			MIN_Y = 2,
			MAX_X = 4,
			MAX_Y = 8,
			ALL = MIN_X | MIN_Y | MAX_X | MAX_Y,
		};

		void addEdges(CellKey cellKey, uint8_t edges);
		
	private:
		RefPtr<const Hierarchy> mHierarchy;
		
		std::map<CellKey, uint8_t> mTraversedEdges;
		std::map<Point, Point> mPointToParent;
	};
}