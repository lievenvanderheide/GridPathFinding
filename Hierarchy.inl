namespace Hierarchy
{
	Hierarchy::BoundaryCellIterator::BoundaryCellIterator(const Hierarchy* hierarchy, CellKey cellKey, EdgeIndex edge, bool reverse)
	{
		DIDA_ASSERT(!isLevelUpCell(hierarchy->cellAt(cellKey)));

		mHierarchy = hierarchy;
		mStack[0] = cellKey;
		mStackHead = 1;
		mNormalAxis = (int8_t)edge & 1;
		mTowardsEdge = (int8_t)edge >> 1;
		mParallelAxis = mNormalAxis ^ 1;
		mParallelDir = reverse ? 0 : 1;
	}

	bool Hierarchy::BoundaryCellIterator::moveNext()
	{
		if(mStackHead == 0)
		{
			return false;
		}

		mCur = mStack[--mStackHead];
		while(mHierarchy->cellAt(mCur) == Cell::PARTIAL)
		{
			mCur.mCoords <<= 1;
			mCur.mLevel--;

			mCur.mCoords[mNormalAxis] += mTowardsEdge;

			CellKey next = mCur;
			next.mCoords[mParallelAxis] += mParallelDir;
			mStack[mStackHead++] = next;

			mCur.mCoords[mParallelAxis] += mParallelDir ^ 1;
		}

		return true;
	}

	CellKey Hierarchy::BoundaryCellIterator::cell() const
	{
		return mCur;
	}
}