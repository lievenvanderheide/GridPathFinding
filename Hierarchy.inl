namespace Hierarchy
{
	Cell HierarchyLevel::cellAt(Point pt) const
	{
		DIDA_ASSERT(pt.mX >= 0 && pt.mX < mWidth &&
			pt.mY >= 0 && pt.mY < mHeight);
		return mCells[pt.mX + pt.mY * mWidth];
	}

	template <EdgeIndex edgeIndex, OnEdgeDir tieResolve>
	CellKey Hierarchy::topLevelCellContainingEdgePoint(CellKey cellKey, Point edgePoint) const
	{	
		int8_t edgeAxis = (int8_t)edgeIndex & 1;
		int8_t edgeSide = (int8_t)edgeIndex >> 1;
		int8_t perpAxis = edgeAxis ^ 1;

		if(cellAt(cellKey) == Cell::PARTIAL)
		{
			int16_t min = cellKey.mCoords[perpAxis] << cellKey.mLevel;
			int16_t max = min + (1 << cellKey.mLevel);

			while(cellAt(cellKey) == Cell::PARTIAL)
			{
				cellKey.mCoords <<= 1;

				cellKey.mCoords[edgeAxis] += edgeSide;

				int16_t mid = (min + max) / 2;
				if(edgePoint[perpAxis] < mid)
				{
					max = mid;
				}
				else
				{
					min = mid;
					cellKey.mCoords[perpAxis]++;
				}
				
				cellKey.mLevel--;
			}

			return cellKey;
		}
		else
		{
			while(isLevelUpCell(cellAt(cellKey)))
			{
				cellKey.mCoords >>= 1;
				cellKey.mLevel++;
			}

			return cellKey;
		}
	}

	template <EdgeIndex edge>
	bool Hierarchy::edgeTraversable(CellKey cellKey) const
	{
		Cell cell = cellAt(cellKey);
		if(cell == Cell::PARTIAL)
		{
			BoundaryCellIterator<edge, OnEdgeDir::TOWARDS_POSITIVE> it(this, cellKey);
			while(it.moveNext())
			{
				if(cellAt(it.cell()) == Cell::EMPTY)
				{
					return false;
				}
			}

			return true;
		}
		else
		{
			return isFullCell(cell);
		}
	}

	template <EdgeIndex edge, OnEdgeDir dir>
	Hierarchy::BoundaryCellIterator<edge, dir>::BoundaryCellIterator(
		const Hierarchy* hierarchy, CellKey cellKey)
	{
		DIDA_ASSERT(!isLevelUpCell(hierarchy->cellAt(cellKey)));

		mHierarchy = hierarchy;
		mStack[0] = cellKey;
		mStackHead = 1;
	}

	template <EdgeIndex edge, OnEdgeDir dir>
	Hierarchy::BoundaryCellIterator<edge, dir>::BoundaryCellIterator(
		const Hierarchy* hierarchy, CellKey cellKey, int16_t beginCoord)
	{
		static_assert(dir == OnEdgeDir::TOWARDS_POSITIVE);

		mHierarchy = hierarchy;

		int8_t normalAxis = (int8_t)edge & 1;
		int8_t towardsEdge = (int8_t)edge >> 1;
		int8_t parallelAxis = normalAxis ^ 1;

		int16_t cellMin = cellKey.mCoords[parallelAxis] << cellKey.mLevel;
		int16_t cellMax = cellMin + (1 << cellKey.mLevel);

		if(beginCoord < cellMin || 
			mHierarchy->cellAt(cellKey) != Cell::PARTIAL)
		{	
			mStack[0] = cellKey;
			mStackHead = 1;
		}
		else
		{
			DIDA_ASSERT(mHierarchy->cellAt(cellKey) == Cell::PARTIAL);

			mStackHead = 0;

			do
			{
				cellKey.mCoords <<= 1;
				cellKey.mLevel--;

				cellKey.mCoords[normalAxis] += towardsEdge;

				CellKey next = cellKey;
				next.mCoords[parallelAxis]++;

				int16_t mid = (cellMin + cellMax) / 2;
				if(beginCoord < mid)
				{
					cellMax = mid;
					mStack[mStackHead++] = next;
				}
				else
				{
					cellMin = mid;
					cellKey = next;
				}
			}
			while(mHierarchy->cellAt(cellKey) == Cell::PARTIAL);

			mStack[mStackHead++] = cellKey;
		}
	}

	template <EdgeIndex edge, OnEdgeDir dir>
	bool Hierarchy::BoundaryCellIterator<edge, dir>::moveNext()
	{
		int8_t normalAxis = (int8_t)edge & 1;
		int8_t towardsEdge = (int8_t)edge >> 1;
		int8_t parallelAxis = normalAxis ^ 1;

		if(mStackHead == 0)
		{
			return false;
		}

		mCur = mStack[--mStackHead];
		while(mHierarchy->cellAt(mCur) == Cell::PARTIAL)
		{
			mCur.mCoords <<= 1;
			mCur.mLevel--;

			mCur.mCoords[normalAxis] += towardsEdge;

			CellKey next = mCur;
			if(dir == OnEdgeDir::TOWARDS_POSITIVE)
				next.mCoords[parallelAxis]++;
			else
				mCur.mCoords[parallelAxis]++;

			mStack[mStackHead++] = next;
		}

		return true;
	}

	template <EdgeIndex edge, OnEdgeDir dir>
	CellKey Hierarchy::BoundaryCellIterator<edge, dir>::cell() const
	{
		return mCur;
	}

	template <EdgeIndex edge, OnEdgeDir dir>
	CellKey Hierarchy::prevBoundaryCell(CellKey cellKey) const
	{
		int8_t normalAxis = (int8_t)edge & 1;
		int8_t towardsEdge = (int8_t)edge >> 1;
		int8_t parallelAxis = normalAxis ^ 1;

		if(dir == OnEdgeDir::TOWARDS_POSITIVE)
			cellKey.mCoords[parallelAxis]--;
		else
			cellKey.mCoords[parallelAxis]++;

		Cell cell = cellAt(cellKey);
		if(isLevelUpCell(cell))
		{
			do
			{
				cellKey.mCoords >>= 1;
				cellKey.mLevel++;
				cell = cellAt(cellKey);
			}
			while(isLevelUpCell(cell));
		}
		else if(cell == Cell::PARTIAL)
		{
			do
			{
				cellKey.mCoords <<= 1;
				cellKey.mCoords[normalAxis] += towardsEdge;
				if(dir == OnEdgeDir::TOWARDS_POSITIVE)
					cellKey.mCoords[parallelAxis]++;
				cellKey.mLevel--;
				cell = cellAt(cellKey);
			}
			while(cell == Cell::PARTIAL);
		}

		return cellKey;
	}

	template <EdgeIndex edge, OnEdgeDir dir>
	CellKey Hierarchy::nextBoundaryCell(CellKey cellKey) const
	{
		return prevBoundaryCell<edge, oppositeDir(dir)>(cellKey);
	}
}