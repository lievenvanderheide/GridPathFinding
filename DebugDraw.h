#pragma once

#include <QPainter>

#include "Utils.h"
#include "Hierarchy.h"

namespace Hierarchy
{
	class DebugDraw
	{
	public:
		DebugDraw(const Hierarchy* hierarchy);

		void drawLine(Point from, Point to);

		void drawToPainter(QPainter& painter, float scale) const;

	private:
		const Hierarchy* mHierarchy;

		struct Line
		{
			Point mFrom;
			Point mTo;
		};
		std::vector<Line> mLines;

		struct Beam
		{
			Point mMin;
			Point mMax;
		};
		std::vector<Beam> mBeams;
	};
}