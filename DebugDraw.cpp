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

	void DebugDraw::drawBeam(CellKey cellKey, Direction direction)
	{
		Beam beam;
		beam.mMin = cellKey.corner(CornerIndex::MIN_X_MIN_Y);
		beam.mMax = cellKey.corner(CornerIndex::MAX_X_MAX_Y);
		mBeams.push_back(beam);
	}

	void DebugDraw::drawToPainter(QPainter& painter, float scale) const
	{
		painter.setPen(QPen(QColor(255, 0, 0)));

		for(const Line& line : mLines)
		{
			QLineF qtLine(line.mFrom.mX * scale, line.mFrom.mY * scale,
				line.mTo.mX * scale, line.mTo.mY * scale);
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