#include "pch.h"
#include "TestCase.h"

namespace Hierarchy
{
	TestCase::TestCase(const Hierarchy* hierarchy, const CellAndCorner* roots, int numRoots)
		: mHierarchy(hierarchy),
		mRoots(roots, roots + numRoots)
	{
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

			QImage grayScaleImage = image.convertToFormat(QImage::Format_Grayscale8);
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
		CellKey cellKey = mHierarchy->cellContainingPoint(pt);

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
}