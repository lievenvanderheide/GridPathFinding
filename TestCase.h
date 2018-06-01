#pragma once

#include "Obj.h"
#include "Hierarchy.h"
#include "HierarchyPathFinder.h"

namespace Hierarchy
{
	class TestCase : public Obj
	{
	public:
		TestCase(const Hierarchy* hierarchy, const CellAndCorner* roots, int numRoots);
		virtual ~TestCase();

		static RefPtr<TestCase> loadFromFile(const char* fileName);

		const Hierarchy* hierarchy() const { return mHierarchy; }
		const CellAndCorner* rootsBegin() const { return mRoots.data(); }
		int numRoots() const { return (int)mRoots.size(); }

	private:
		TestCase() { }

		bool tryAddRoot(Point pt, CornerIndex corner);

		RefPtr<const Hierarchy> mHierarchy;
		std::vector<CellAndCorner> mRoots;
	};
}