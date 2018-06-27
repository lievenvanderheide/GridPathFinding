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

	template <CornerIndex startCornerIndex, Axis2 axis>
	Hierarchy::BoundaryCellIterator<startCornerIndex, axis>::BoundaryCellIterator(
		const Hierarchy* hierarchy, CellKey cellKey)
	{
		DIDA_ASSERT(!isLevelUpCell(hierarchy->cellAt(cellKey)));

		mHierarchy = hierarchy;
		mStack[0] = cellKey;
		mStackHead = 1;
	}

	template <CornerIndex startCornerIndex, Axis2 axis>
	Hierarchy::BoundaryCellIterator<startCornerIndex, axis>::BoundaryCellIterator(
		const Hierarchy* hierarchy, CellKey cellKey, int16_t beginCoord)
	{
		mHierarchy = hierarchy;

		constexpr Axis2 normalAxis = otherAxis(axis);
		constexpr int8_t towardsEdge = cornerOnAxis(startCornerIndex, normalAxis);
		constexpr bool towardsPositive = cornerOnAxis(startCornerIndex, axis) == 0;

		int16_t cellMin = cellKey.mCoords[axis] << cellKey.mLevel;
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
				if(towardsPositive)
					next.mCoords[axis]++;
				else
					cellKey.mCoords[axis]++;

				int16_t mid = (cellMin + cellMax) / 2;
				if(beginCoord < mid)
				{
					cellMax = mid;
					if(towardsPositive)
						mStack[mStackHead++] = next;
					else
						cellKey = next;
				}
				else
				{
					cellMin = mid;
					if(towardsPositive)
						cellKey = next;
					else
						mStack[mStackHead++] = next;
				}
			}
			while(mHierarchy->cellAt(cellKey) == Cell::PARTIAL);

			mStack[mStackHead++] = cellKey;
		}
	}

	template <CornerIndex startCornerIndex, Axis2 axis>
	bool Hierarchy::BoundaryCellIterator<startCornerIndex, axis>::moveNext()
	{
		constexpr Axis2 normalAxis = otherAxis(axis);
		constexpr int8_t towardsEdge = cornerOnAxis(startCornerIndex, normalAxis);
		constexpr bool towardsPositive = cornerOnAxis(startCornerIndex, axis) == 0;

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
			if(towardsPositive)
				next.mCoords[axis]++;
			else
				mCur.mCoords[axis]++;

			mStack[mStackHead++] = next;
		}

		return true;
	}

	template <CornerIndex startCornerIndex, Axis2 axis>
	CellKey Hierarchy::BoundaryCellIterator<startCornerIndex, axis>::cell() const
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