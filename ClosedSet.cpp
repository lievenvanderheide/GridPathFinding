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
			
			if(pointTraversedTestNeighbor<EdgeIndex::MAX_X>(cellKey, pt.mY))
				return true;
		}
		else if(pt.mX == max.mX)
		{
			if(mask & (uint8_t)EdgeFlags::MAX_X)
				return true;

			if(pointTraversedTestNeighbor<EdgeIndex::MIN_X>(cellKey, pt.mY))
				return true;
		}

		if(pt.mY == min.mY)
		{
			if(mask & (uint8_t)EdgeFlags::MIN_Y)
				return true;

			if(pointTraversedTestNeighbor<EdgeIndex::MAX_Y>(cellKey, pt.mX))
				return true;
		}
		else if(pt.mY == max.mY)
		{
			if(mask & (uint8_t)EdgeFlags::MAX_Y)
				return true;

			if(pointTraversedTestNeighbor<EdgeIndex::MIN_Y>(cellKey, pt.mX))
				return true;
		}

		return false;
	}

	template <EdgeIndex edge> 
	bool ClosedSet::pointTraversedTestNeighbor(CellKey neighborKey, int16_t coord) const
	{
		int8_t edgeAxis = (int8_t)edge & 1;
		int8_t perpAxis = edgeAxis ^ 1;

		Cell cell = mHierarchy->cellAt(neighborKey);
		if(cell == Cell::PARTIAL)
		{
			Hierarchy::BoundaryCellIterator<edge, OnEdgeDir::TOWARDS_POSITIVE> it(mHierarchy, neighborKey);
			while(it.moveNext())
			{
				CellKey cellKey = it.cell();
				int16_t cellMin = cellKey.mCoords[perpAxis] << cellKey.mLevel;
				int16_t cellMax = (cellKey.mCoords[perpAxis] + 1) << cellKey.mLevel;
				if(coord >= cellMin && coord <= cellMax)
				{
					auto it = mTraversedEdges.find(cellKey);
					uint8_t mask = it != mTraversedEdges.end() ? it->second : 0;
					if(mask & ((uint8_t)1 << (int)edge))
					{
						return true;
					}
				}
			}

			return false;
		}
		else
		{
			do
			{
				neighborKey.mCoords >>= 1;
				neighborKey.mLevel++;
			}
			while(isLevelUpCell(mHierarchy->cellAt(neighborKey)));

			auto it = mTraversedEdges.find(neighborKey);
			uint8_t mask = it != mTraversedEdges.end() ? it->second : 0;
			if(mask & ((uint8_t)1 << (int)edge))
			{
				return true;
			}

			return false;
		}
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