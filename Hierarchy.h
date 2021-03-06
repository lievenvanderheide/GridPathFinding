#pragma once

#include <vector>
#include <QPainter>

#include "Utils.h"
#include "Obj.h"

namespace Hierarchy
{
	enum class Cell : uint8_t
	{
		EMPTY = 1,
		FULL = 2,
		PARTIAL = 4,

		LEVEL_UP_MASK = 8,
		LEVEL_UP_EMPTY = LEVEL_UP_MASK | EMPTY,
		LEVEL_UP_FULL = LEVEL_UP_MASK | FULL,
	};

	enum class CornerIndex : uint8_t
	{
		MIN_X_MIN_Y = 0,
		MAX_X_MIN_Y = 1,
		MIN_X_MAX_Y = 2,
		MAX_X_MAX_Y = 3,
	};

	enum class EdgeIndex : uint8_t
	{
		MIN_X = 0,
		MIN_Y = 1,
		MAX_X = 2,
		MAX_Y = 3,
	};

	enum class OnEdgeDir : int8_t
	{
		TOWARDS_NEGATIVE = -1,
		TOWARDS_POSITIVE = 1,
	};

	static constexpr inline bool isEmptyCell(Cell cell)
	{
		return ((uint8_t)cell & (uint8_t)Cell::EMPTY) != 0;
	}

	static constexpr inline bool isFullCell(Cell cell)
	{
		return ((uint8_t)cell & (uint8_t)Cell::FULL) != 0;
	}

	static constexpr inline bool isLevelUpCell(Cell cell)
	{
		return ((uint8_t)cell & (uint8_t)Cell::LEVEL_UP_MASK) != 0;
	}

	static constexpr inline int8_t cornerOnAxis(CornerIndex cornerIndex, Axis2 axis)
	{
		return ((int8_t)cornerIndex >> (int8_t)axis) & 1;
	}

	static constexpr EdgeIndex xEdgeFromCorner(CornerIndex corner)
	{
		if((int8_t)corner & 1)
			return EdgeIndex::MAX_X;
		else
			return EdgeIndex::MIN_X;
	}

	static constexpr OnEdgeDir xEdgeDirFromCorner(CornerIndex corner)
	{
		if((int8_t)corner & 2)
			return OnEdgeDir::TOWARDS_NEGATIVE;
		else
			return OnEdgeDir::TOWARDS_POSITIVE;
	}

	static constexpr EdgeIndex yEdgeFromCorner(CornerIndex corner)
	{
		if((int8_t)corner & 2)
			return EdgeIndex::MAX_Y;
		else
			return EdgeIndex::MIN_Y;
	}

	static constexpr OnEdgeDir yEdgeDirFromCorner(CornerIndex corner)
	{
		if((int8_t)corner & 1)
			return OnEdgeDir::TOWARDS_NEGATIVE;
		else
			return OnEdgeDir::TOWARDS_POSITIVE;
	}

	struct CellKey
	{
		Point mCoords;
		uint8_t mLevel;

		CellKey() { }
		CellKey(Point cell, uint8_t level)
			: mCoords(cell),
			mLevel(level)
		{
		}

		bool operator == (const CellKey& b) const
		{
			return mCoords == b.mCoords && mLevel == b.mLevel;
		}

		bool operator != (const CellKey& b) const
		{
			return mCoords != b.mCoords || mLevel != b.mLevel;
		}

		bool operator < (const CellKey& b) const
		{
			if(mCoords != b.mCoords)
				return mCoords < b.mCoords;
			else
				return mLevel < b.mLevel;
		}

		Point corner(CornerIndex corner) const
		{
			int8_t cornerX = (int8_t)corner & 1;
			int8_t cornerY = (int8_t)corner >> 1;
			return Point(
				((mCoords.mX + cornerX) << mLevel) - cornerX,
				((mCoords.mY + cornerY) << mLevel) - cornerY);
		}

		static CellKey invalidCellKey()
		{
			CellKey ret;
			ret.mCoords = Point::invalidPoint();
			ret.mLevel = -1;
			return ret;
		}
	};

	class HierarchyLevel
	{
		friend class Hierarchy;

	public:
		void initLevel0(int width, int height, const uint8_t* elevation);
		void initLevel0(int width, int height, const uint8_t* elevation, const uint8_t* overrides);
		void initWithLowerLevel(HierarchyLevel& deeperLevel);

		inline Cell cellAt(Point pt) const;

		void rotate90DegCcw();

	private:
		int mWidth;
		int mHeight;
		std::vector<Cell> mCells;
	};

	class Hierarchy : public Obj
	{
	public:
		Hierarchy(int width, int height, const uint8_t* elevation);

		int numLevels() const
		{
			return (int)mLevels.size();
		}

		int width() const { return mWidth; }
		int height() const { return mHeight; }

		Cell cellAt(CellKey cellKey) const
		{
			return mLevels[cellKey.mLevel].cellAt(cellKey.mCoords);
		}

		CellKey topLevelCellContainingPoint(Point pt) const;
		CellKey topLevelCellContainingCorner(CellKey cellKey, CornerIndex cornerIndex) const;
		
		template <EdgeIndex edgeIndex, OnEdgeDir tieResolve>
		CellKey topLevelCellContainingEdgePoint(CellKey cellKey, Point edgePoint) const;
		
		CellKey cellKeyAdjToCorner(CellKey cornerCellKey, CornerIndex cornerIndex) const;

		CellKey diagNextCellKey(CellKey cellKey, CornerIndex cornerIndex) const;

		template <CornerIndex startCornerIndex, Axis2 axis> 
		class BoundaryCellIterator
		{
		public:
			inline BoundaryCellIterator(const Hierarchy* hierarchy, CellKey cellKey);
			inline BoundaryCellIterator(const Hierarchy* hierarchy, CellKey cellKey, int16_t beginCoord);

			inline bool moveNext();

			inline CellKey cell() const;

		private:
			CellKey mCur;
			CellKey mStack[16];
			int8_t mStackHead;
			
			const Hierarchy* mHierarchy;
		};

		template <EdgeIndex edge, OnEdgeDir dir>
		CellKey prevBoundaryCell(CellKey cellKey) const;

		template <EdgeIndex edge, OnEdgeDir dir>
		CellKey nextBoundaryCell(CellKey cellKey) const;

		void drawLevel0AsBase(QPainter& painter, const QRect& rect) const;
		void drawLevel(QPainter& painter, int levelIndex, const QRect& rect) const;
		void drawLevelWithPalette(QPainter& painter, int levelIndex, const QRect& rect, const QVector<QRgb>& palette) const;

		void rotate90DegCcw();
		
	private:
		std::vector<HierarchyLevel> mLevels;
		int mWidth;
		int mHeight;
	};
}

#include "Hierarchy.inl"