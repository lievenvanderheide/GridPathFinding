#pragma once

#include "Obj.h"
#include "Hierarchy.h"
#include "HierarchyPathFinder.h"

namespace Hierarchy
{
	class TestCase : public Obj
	{
	public:
		TestCase(Hierarchy* hierarchy, const CellAndCorner* roots, int numRoots);
		TestCase(const TestCase& src);
		virtual ~TestCase();

		static RefPtr<TestCase> loadFromFile(const char* fileName);

		const Hierarchy* hierarchy() const { return mHierarchy; }
		const CellAndCorner* rootsBegin() const { return mRoots.data(); }
		int numRoots() const { return (int)mRoots.size(); }

		void rotate90DegCcw();

	private:
		TestCase() { }

		bool tryAddRoot(Point pt, CornerIndex corner);

		RefPtr<Hierarchy> mHierarchy;
		std::vector<CellAndCorner> mRoots;
	};
}