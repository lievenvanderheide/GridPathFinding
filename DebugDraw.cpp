#include "pch.h"
#include "DebugDraw.h"

namespace Hierarchy
{
	DebugDraw::DebugDraw(const Hierarchy* hierarchy)
		: mHierarchy(hierarchy)
	{
	}

	void DebugDraw::drawLine(Point from, Point to)
	{
		Line line;
		line.mFrom = from;
		line.mTo = to;
		mLines.push_back(line);
	}

	void DebugDraw::drawToPainter(QPainter& painter, float scale) const
	{
		painter.setPen(QPen(QColor(255, 0, 0)));

		for(const Line& line : mLines)
		{
			QLineF qtLine(
				(line.mFrom.mX + .5f) * scale,
				(line.mFrom.mY + .5f) * scale,
				(line.mTo.mX + .5f) * scale,
				(line.mTo.mY + .5f) * scale);
			painter.drawLine(qtLine);
		}

		for(const Beam& beam : mBeams)
		{
			QRectF rect(
				beam.mMin.mX * scale, beam.mMin.mY * scale,
				beam.mMax.mX * scale, beam.mMax.mY * scale);
			painter.fillRect(rect, QColor(255, 240, 0, 128));
		}
	}
}