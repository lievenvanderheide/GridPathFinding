#include "pch.h"
#include "TestCase.h"

namespace Hierarchy
{
	TestCase::TestCase(Hierarchy* hierarchy, const CellAndCorner* roots, int numRoots)
		: mHierarchy(hierarchy),
		mRoots(roots, roots + numRoots)
	{
	}

	TestCase::TestCase(const TestCase& src)
		: mRoots(src.mRoots)
	{
		mHierarchy.setNew(new Hierarchy(*src.mHierarchy));
	}

	TestCase::~TestCase()
	{
	}

	RefPtr<TestCase> TestCase::loadFromFile(const char* fileName)
	{
		QImage image(fileName);
		if(image.isNull())
		{
			return nullptr;
		}

		int imageWidth = image.width();
		int imageHeight = image.height();
		
		RefPtr<TestCase> ret;
		ret.setNew(new TestCase());

		{
			std::vector<uint8_t> elevation;
			elevation.resize(imageWidth * imageHeight);

			int i = 0;
			for(int y = 0; y < imageHeight; y++)
			{
				for(int x = 0; x < imageWidth; x++)
				{
					if(image.pixel(x, y) == qRgb(0, 0, 0))
						elevation[i] = 0;
					else
						elevation[i] = 255;
					i++;
				}
			}
			
			ret->mHierarchy.setNew(new Hierarchy(imageWidth, imageHeight, elevation.data()));
		}
		
		for(int y = 0; y < imageHeight; y++)
		{	
			for(int x = 0; x < imageWidth; x++)
			{
				QRgb color = image.pixel(x, y);
				switch(color)
				{
				case qRgb(255, 0, 0):
					if(!ret->tryAddRoot(Point(x, y), CornerIndex::MIN_X_MIN_Y))
						return nullptr;
					break;

				case qRgb(0, 255, 0):
					if(!ret->tryAddRoot(Point(x, y), CornerIndex::MAX_X_MIN_Y))
						return nullptr;
					break;

				case qRgb(0, 0, 255):
					if(!ret->tryAddRoot(Point(x, y), CornerIndex::MIN_X_MAX_Y))
						return nullptr;
					break;

				case qRgb(255, 255, 0):
					if(!ret->tryAddRoot(Point(x, y), CornerIndex::MAX_X_MAX_Y))
						return nullptr;
					break;
				}
			}
		}

		if(ret->mRoots.empty())
		{
			return nullptr;
		}

		return ret;
	}

	bool TestCase::tryAddRoot(Point pt, CornerIndex corner)
	{
		CellKey cellKey = mHierarchy->topLevelCellContainingPoint(pt);

		Point expectedPt = cellKey.mCoords << cellKey.mLevel;
		if((int8_t)corner & 1)
			expectedPt.mX += (1 << cellKey.mLevel) - 1;
		if((int8_t)corner >> 1)
			expectedPt.mY += (1 << cellKey.mLevel) - 1;

		if(pt == expectedPt)
		{
			CellAndCorner root;
			root.mCell = cellKey;
			root.mCorner = corner;
			mRoots.push_back(root);
			return true;
		}
		else
		{
			return false;
		}
	}

	static CornerIndex rotateCorner90DegCcw(CornerIndex cornerIndex)
	{
		const CornerIndex table[] =
		{
			CornerIndex::MAX_X_MIN_Y,
			CornerIndex::MAX_X_MAX_Y,
			CornerIndex::MIN_X_MIN_Y,
			CornerIndex::MIN_X_MAX_Y,
		};

		return table[(int)cornerIndex];
	}

	void TestCase::rotate90DegCcw()
	{
		mHierarchy->rotate90DegCcw();

		int rotatedWidth = mHierarchy->width();
		int rotatedHeight = mHierarchy->height();
		for(CellAndCorner& root : mRoots)
		{
			CellKey level0CellKey = root.mCell;
			int8_t cornerX = (int8_t)root.mCorner & 1;
			int8_t cornerY = (int8_t)root.mCorner >> 1;
			while(level0CellKey.mLevel != 0)
			{
				level0CellKey.mCoords <<= 1;
				level0CellKey.mCoords.mX += cornerX;
				level0CellKey.mCoords.mY += cornerY;
				level0CellKey.mLevel--;
			}

			int16_t tmp = level0CellKey.mCoords.mX;
			level0CellKey.mCoords.mX = rotatedWidth - level0CellKey.mCoords.mY - 1;
			level0CellKey.mCoords.mY = tmp;

			root.mCorner = rotateCorner90DegCcw(root.mCorner);
			root.mCell = mHierarchy->topLevelCellContainingCorner(level0CellKey, root.mCorner);
		}
	}
}