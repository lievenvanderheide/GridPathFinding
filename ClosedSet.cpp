#include "ClosedSet.h"

namespace Hierarchy
{
	ClosedSet::ClosedSet(const Hierarchy* hierarchy)
		: mHierarchy(hierarchy)
	{
	}

	bool ClosedSet::pointTraversed(CellKey cellKey, Point pt) const
	{
		Point min = cellKey.corner(CornerIndex::MIN_X_MIN_Y);
		Point max = cellKey.corner(CornerIndex::MAX_X_MAX_Y);

		auto it = mTraversedEdges.find(cellKey);
		uint8_t mask = it != mTraversedEdges.end() ? it->second : 0;

		if(pt.mX == min.mX)
		{
			if(mask & (uint8_t)EdgeFlags::MIN_X)
				return true;
		}
		else if(pt.mX == max.mX)
		{
			if(mask & (uint8_t)EdgeFlags::MAX_X)
				return true;
		}

		if(pt.mY == min.mY)
		{
			if(mask & (uint8_t)EdgeFlags::MIN_Y)
				return true;
		}
		else if(pt.mY == max.mY)
		{
			if(mask & (uint8_t)EdgeFlags::MAX_Y)
				return true;
		}

		return false;
	}

	bool ClosedSet::tryAddPoint(Point pt, Point parentPt)
	{
		auto it = mPointToParent.find(pt);
		if(it != mPointToParent.end())
		{
			return false;
		}

		mPointToParent.insert(std::make_pair(pt, parentPt));
		return true;
	}

	void ClosedSet::addEdges(CellKey cellKey, uint8_t edges)
	{
		auto it = mTraversedEdges.find(cellKey);
		if(it != mTraversedEdges.end())
		{
			it->second |= edges;
		}
		else
		{
			mTraversedEdges.insert(std::make_pair(cellKey, edges));
		}
	}
}