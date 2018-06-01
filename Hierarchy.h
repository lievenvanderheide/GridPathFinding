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

	enum class Direction : uint8_t
	{
		MIN_X,
		MIN_Y,
		MAX_X,
		MAX_Y,

		CORNER_0,
		CORNER_1, 
		CORNER_2,
		CORNER_3,
	};

	enum class SideEdgeDir : int8_t
	{
		TOWARDS_NEGATIVE = -1,
		TOWARDS_POSITIVE = 1,
	};

	static inline bool isEmptyCell(Cell cell)
	{
		return ((uint8_t)cell & (uint8_t)Cell::EMPTY) != 0;
	}

	static inline bool isFullCell(Cell cell)
	{
		return ((uint8_t)cell & (uint8_t)Cell::FULL) != 0;
	}

	static inline bool isLevelUpCell(Cell cell)
	{
		return ((uint8_t)cell & (uint8_t)Cell::LEVEL_UP_MASK) != 0;
	}

	static CornerIndex oppositeCorner(CornerIndex cornerIndex)
	{
		return (CornerIndex)((uint8_t)cornerIndex ^ 3);
	}

	static EdgeIndex oppositeEdge(EdgeIndex edgeIndex)
	{
		return (EdgeIndex)((uint8_t)edgeIndex ^ 2);
	}

	static inline bool cornerOnEdge(CornerIndex cornerIndex, EdgeIndex edgeIndex)
	{
		uint8_t edgeAxis = (uint8_t)edgeIndex & 1;
		uint8_t edgeSign = (uint8_t)edgeIndex >> 1;
		return ((uint8_t)cornerIndex >> edgeAxis) == edgeSign;
	}

	static inline CornerIndex edgeMin(EdgeIndex edgeIndex)
	{
		uint8_t edgeAxis = (uint8_t)edgeIndex & 1;
		uint8_t edgeSign = (uint8_t)edgeIndex >> 1;
		return (CornerIndex)(edgeSign << edgeAxis);
	}

	static inline bool isStraightDirection(Direction direction)
	{
		return direction <= Direction::MAX_Y;
	}

	static inline CornerIndex edgeStartCorner(EdgeIndex edge, SideEdgeDir dir)
	{
		int8_t edgeAxis = (int8_t)edge & 1;
		int8_t edgeSide = (int8_t)edge >> 1;

		int8_t minCorner = edgeSide << edgeAxis;
		if(dir == SideEdgeDir::TOWARDS_NEGATIVE)
			return (CornerIndex)(minCorner + (2 >> edgeAxis));
		else
			return (CornerIndex)minCorner;
	}

	static inline CornerIndex edgeEndCorner(EdgeIndex edge, SideEdgeDir dir)
	{
		int8_t edgeAxis = (int8_t)edge & 1;
		int8_t edgeSide = (int8_t)edge >> 1;

		int8_t minCorner = edgeSide << edgeAxis;
		if(dir == SideEdgeDir::TOWARDS_POSITIVE)
			return (CornerIndex)(minCorner + (2 >> edgeAxis));
		else
			return (CornerIndex)minCorner;
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

		Point topLeftCorner() const
		{
			return mCoords << mLevel;
		}

		Point corner(CornerIndex corner) const
		{
			int8_t cornerX = (int8_t)corner & 1;
			int8_t cornerY = (int8_t)corner >> 1;
			return Point(
				(mCoords.mX + cornerX) << mLevel,
				(mCoords.mY + cornerY) << mLevel);
		}

		void edgeCorners(EdgeIndex edge, Point& aOut, Point& bOut) const
		{
			int8_t axis = (int8_t)edge & 1;
			int8_t side = (int8_t)edge >> 1;

			aOut = mCoords;
			aOut[axis] += side;

			bOut = aOut;
			aOut[axis ^ 1]++;

			aOut <<= mLevel;
			bOut <<= mLevel;
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

		Cell cellAt(Point pt) const
		{
			DIDA_ASSERT(pt.mX >= 0 && pt.mX < mWidth &&
				pt.mY >= 0 && pt.mY < mHeight);
			return mCells[pt.mX + pt.mY * mWidth];
		}

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

		CellKey cellContainingPoint(Point pt) const;
		
		CellKey cellKeyAdjToCorner(CellKey cornerCellKey, CornerIndex cornerIndex) const;

		Cell cellAt(CellKey cellKey) const
		{
			return mLevels[cellKey.mLevel].cellAt(cellKey.mCoords);
		}

		class BoundaryCellIterator
		{
		public:
			inline BoundaryCellIterator(const Hierarchy* hierarchy, CellKey cellKey, EdgeIndex edge, bool reverse = false);

			inline bool moveNext();

			inline CellKey cell() const;

		private:
			CellKey mCur;
			CellKey mStack[16];
			
			int8_t mStackHead;
			int8_t mParallelAxis;
			int8_t mParallelDir;
			int8_t mNormalAxis;
			int8_t mTowardsEdge;
			
			const Hierarchy* mHierarchy;
		};

		void drawLevel0AsBase(QPainter& painter, const QRect& rect) const;
		void drawLevel(QPainter& painter, int levelIndex, const QRect& rect) const;
		void drawLevelWithPalette(QPainter& painter, int levelIndex, const QRect& rect, const QVector<QRgb>& palette) const;
		
	private:
		std::vector<HierarchyLevel> mLevels;
		int mWidth;
		int mHeight;
	};
}

#include "Hierarchy.inl"