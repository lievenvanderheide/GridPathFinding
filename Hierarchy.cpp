#include "pch.h"
#include "Hierarchy.h"

namespace Hierarchy
{
	void HierarchyLevel::initLevel0(int width, int height, const uint8_t* elevation)
	{
		mWidth = width;
		mHeight = height;
		int size = width * height;
		
		mCells.resize(size);
		const uint8_t* curElevation = elevation;
		Cell* curCell = mCells.data();
		for(int i = 0; i < size; i++)
		{
			if(*curElevation != 0)
				*curCell = Cell::FULL;
			else
				*curCell = Cell::EMPTY;

			curElevation++;
			curCell++;
		}
	}

	void HierarchyLevel::initLevel0(int width, int height, const uint8_t* elevation, const uint8_t* overrides)
	{
		mWidth = width;
		mHeight = height;
		int size = width * height;

		mCells.resize(size);
		const uint8_t* curElevation = elevation;
		const uint8_t* curOverrides = overrides;
		Cell* curCell = mCells.data();
		for(int i = 0; i < size; i++)
		{
			if(*curOverrides == 0 && *curElevation != 0)
				*curCell = Cell::FULL;
			else
				*curCell = Cell::EMPTY;

			curElevation++;
			curOverrides++;
			curCell++;
		}
	}

	static Cell mergeCells(const Cell cells[4])
	{
		uint8_t merged = 0;
		for(int i = 0; i < 4; i++)
		{
			merged |= (uint8_t)cells[i];
		}

		if(merged == (uint8_t)Cell::EMPTY)
			return Cell::EMPTY;
		else if(merged == (uint8_t)Cell::FULL)
			return Cell::FULL;
		else
			return Cell::PARTIAL;
	}

	void HierarchyLevel::initWithLowerLevel(HierarchyLevel& srcLevel)
	{
		mWidth = (srcLevel.mWidth + 1) / 2;
		mHeight = (srcLevel.mHeight + 1) / 2;
		int size = mWidth * mHeight;

		int roundedDownWidth = srcLevel.mWidth / 2;
		int roundedDownHeight = srcLevel.mHeight / 2;

		mCells.resize(size);
		Cell* curCell = mCells.data();
		Cell* srcCellsOrig = srcLevel.mCells.data();
		for(int y = 0; y < roundedDownHeight; y++)
		{
			for(int x = 0; x < roundedDownWidth; x++)
			{
				Cell* srcCells[] = 
				{
					srcCellsOrig,
					(srcCellsOrig + 1),
					(srcCellsOrig + srcLevel.mWidth),
					(srcCellsOrig + srcLevel.mWidth + 1),
				};

				uint8_t merged = 0;
				for(Cell* srcCell : srcCells)
					merged |= (uint8_t)*srcCell;

				if(merged == (uint8_t)Cell::EMPTY)
				{
					*curCell = Cell::EMPTY;

					for(Cell* srcCell : srcCells)
						*srcCell = Cell::LEVEL_UP_EMPTY;
				}
				else if(merged == (uint8_t)Cell::FULL)
				{
					*curCell = Cell::FULL;

					for(Cell* srcCell : srcCells)
						*srcCell = Cell::LEVEL_UP_FULL;
				}
				else
				{
					*curCell = Cell::PARTIAL;
				}
				
				curCell++;
				srcCellsOrig += 2;
			}

			if(roundedDownWidth != mWidth)
			{
				Cell* srcCells[] = 
				{
					srcCellsOrig,
					(srcCellsOrig + srcLevel.mWidth),
				};

				uint8_t merged = (uint8_t)Cell::EMPTY;
				for(Cell* srcCell : srcCells)
					merged |= (uint8_t)*srcCell;
				
				if(merged == (uint8_t)Cell::EMPTY)
				{
					*curCell = Cell::EMPTY;
					for(Cell* srcCell : srcCells)
						*srcCell = Cell::LEVEL_UP_EMPTY;
				}
				else
				{
					*curCell = Cell::PARTIAL;
				}

				curCell++;
				srcCellsOrig++;
			}

			srcCellsOrig += srcLevel.mWidth;
		}

		if(roundedDownHeight != mHeight)
		{
			for(int x = 0; x < roundedDownWidth; x++)
			{
				Cell* srcCells[] = 
				{
					srcCellsOrig,
					srcCellsOrig + 1,
				};

				uint8_t merged = (uint8_t)Cell::EMPTY;
				for(Cell* srcCell : srcCells)
					merged |= (uint8_t)*srcCell;

				if(merged == (uint8_t)Cell::EMPTY)
				{
					*curCell = Cell::EMPTY;
					for(Cell* srcCell : srcCells)
						*srcCell = Cell::LEVEL_UP_EMPTY;
				}
				else
				{
					*curCell = Cell::PARTIAL;
				}

				curCell++;
				srcCellsOrig += 2;
			}

			if(roundedDownWidth != mWidth)
			{
				if(*srcCellsOrig == Cell::EMPTY)
				{
					*curCell = Cell::EMPTY;
					*srcCellsOrig = Cell::LEVEL_UP_EMPTY;
				}
				else
				{
					*curCell = Cell::PARTIAL;
				}
			}
		}
	}

	void HierarchyLevel::rotate90DegCcw()
	{
		std::vector<Cell> rotatedCells;
		rotatedCells.resize(mWidth * mHeight);

		const Cell* src = mCells.data();
		for(int y = 0; y < mHeight; y++)
		{
			Cell* dest = rotatedCells.data() + mHeight - y - 1;
			for(int x = 0; x < mWidth; x++)
			{
				*dest = *(src++);
				dest += mHeight;
			}
		}

		mCells = std::move(rotatedCells);
		std::swap(mWidth, mHeight);
	}

	Hierarchy::Hierarchy(int width, int height, const uint8_t* elevation)
		: mWidth(width),
		mHeight(height)
	{
		DIDA_ASSERT(width > 0 && height > 0);

		unsigned long numLevels;
		if(width > height)
		{
			_BitScanReverse(&numLevels, 2 * width - 1);
		}
		else
		{
			_BitScanReverse(&numLevels, 2 * height - 1);
		}
		numLevels++;

		DIDA_ON_DEBUG(int fullSize = 1 << (numLevels - 1));
		DIDA_ASSERT(width <= fullSize && width);
		DIDA_ASSERT(height <= fullSize);
		DIDA_ASSERT(fullSize < 2 * std::max(width, height));

		mLevels.resize(numLevels);
		mLevels[0].initLevel0(width, height, elevation);
		for(int i = 1; i < numLevels; i++)
		{
			mLevels[i].initWithLowerLevel(mLevels[i - 1]);
		}
	}

	CellKey Hierarchy::topLevelCellContainingPoint(Point pt) const
	{
		CellKey ret;
		ret.mCoords = pt;
		ret.mLevel = 0;

		while(isLevelUpCell(cellAt(ret)))
		{
			ret.mCoords >>= 1;
			ret.mLevel++;
		}

		return ret;
	}

	CellKey Hierarchy::topLevelCellContainingCorner(CellKey cellKey, CornerIndex cornerIndex) const
	{
		int8_t cornerX = (int8_t)cornerIndex & 1;
		int8_t cornerY = (int8_t)cornerIndex >> 1;

		if(cellAt(cellKey) == Cell::PARTIAL)
		{
			while(cellAt(cellKey) == Cell::PARTIAL)
			{
				cellKey.mCoords <<= 1;
				cellKey.mCoords.mX += cornerX;
				cellKey.mCoords.mY += cornerY;
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

	CellKey Hierarchy::cellKeyAdjToCorner(CellKey cornerCellKey, CornerIndex corner) const
	{
		int8_t cornerX = (int8_t)corner & 1;
		int8_t cornerY = (int8_t)corner >> 1;

		CellKey ret = cornerCellKey;
		ret.mCoords.mX += cornerX - 1;
		ret.mCoords.mY += cornerY - 1;

		Cell cell = cellAt(ret);
		if(cell == Cell::PARTIAL)
		{
			do
			{
				ret.mCoords.mX = 2 * ret.mCoords.mX + (1 - cornerX);
				ret.mCoords.mY = 2 * ret.mCoords.mY + (1 - cornerY);
				ret.mLevel--;
			}
			while(cellAt(ret) == Cell::PARTIAL);
		}
		else if(isLevelUpCell(cell))
		{
			do
			{
				ret.mCoords >>= 1;
				ret.mLevel++;
			}
			while(isLevelUpCell(cellAt(ret)));
		}
		
		return ret;
	}

	CellKey Hierarchy::diagNextCellKey(CellKey cellKey, CornerIndex cornerIndex) const
	{
		int8_t cornerX = (int8_t)cornerIndex & 1;
		int8_t cornerY = (int8_t)cornerIndex >> 1;

		cellKey.mCoords.mX += 1 - 2 * cornerX;
		cellKey.mCoords.mY += 1 - 2 * cornerY;
		return topLevelCellContainingCorner(cellKey, cornerIndex);
	}

	void Hierarchy::drawLevel0AsBase(QPainter& painter, const QRect& rect) const
	{
		QVector<QRgb> palette(256, 0);
		palette[(int)Cell::EMPTY] = qRgb(128, 128, 128);
		palette[(int)Cell::FULL] = qRgb(255, 255, 255);
		palette[(int)Cell::PARTIAL] = qRgb(255, 255, 255);
		palette[(int)Cell::LEVEL_UP_EMPTY] = qRgb(128, 128, 128);
		palette[(int)Cell::LEVEL_UP_FULL] = qRgb(255, 255, 255);
		drawLevelWithPalette(painter, 0, rect, palette);
	}

	void Hierarchy::drawLevel(QPainter& painter, int levelIndex, const QRect& rect) const
	{
		QVector<QRgb> palette(256, 0);
		palette[(int)Cell::EMPTY] = qRgb(32, 128, 255);
		palette[(int)Cell::FULL] = qRgb(128, 255, 32);
		palette[(int)Cell::PARTIAL] = qRgb(185, 122, 87);
		palette[(int)Cell::LEVEL_UP_EMPTY] = qRgb(53, 160, 198);
		palette[(int)Cell::LEVEL_UP_FULL] = qRgb(64, 196, 16);
		drawLevelWithPalette(painter, levelIndex, rect, palette);
	}	

	void Hierarchy::drawLevelWithPalette(QPainter& painter, int levelIndex, const QRect& rect, const QVector<QRgb>& palette) const
	{
		const HierarchyLevel& level = mLevels[levelIndex];

		QRectF srcRect(0, 0,
			(float)mWidth / (float)(1 << levelIndex),
			(float)mHeight / (float)(1 << levelIndex));

		if(level.mWidth % 4 == 0)
		{
			// Properly aligned, so we can draw the mCells data immediately.
			QImage image((uchar*)level.mCells.data(), level.mWidth, level.mHeight, QImage::Format_Indexed8);
			image.setColorTable(palette);

			painter.drawImage(rect, image, srcRect);
		}
		else
		{
			// Scanline width isn't a multiple of 4, so create of the data which
			// does have this property.
			QImage image(level.mWidth, level.mHeight, QImage::Format_Indexed8);

			const Cell* src = level.mCells.data();
			for(int y = 0; y < level.mHeight; y++)
			{
				std::copy(src, src + level.mWidth, (Cell*)image.scanLine(y));
				src += level.mWidth;
			}

			image.setColorTable(palette);
			painter.drawImage(rect, image, srcRect);
		}
	}

	void Hierarchy::rotate90DegCcw()
	{	
		mLevels[0].rotate90DegCcw();

		int levelWidth = mWidth;
		int levelHeight = mHeight;
		int level = 1;

		while(((levelWidth | levelHeight) & 1) == 0)
		{
			levelWidth /= 2;
			levelHeight /= 2;
			mLevels[level].rotate90DegCcw();
			level++;
		}

		if(level < mLevels.size())
		{
			for(Cell& cell : mLevels[level - 1].mCells)
				cell = (Cell)((uint8_t)cell & ~(uint8_t)Cell::LEVEL_UP_MASK);

			do
			{
				mLevels[level].initWithLowerLevel(mLevels[level - 1]);
				level++;
			}
			while(level < mLevels.size());
		}

		std::swap(mWidth, mHeight);
	}
}
