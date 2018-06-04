#pragma once

#include <QWidget>

#include "Hierarchy.h"
#include "HierarchyPathFinder.h"
#include "TestCase.h"
#include "DebugDraw.h"

class QSlider;
class QLabel;
class QScrollArea;

class HierarchyArea;

class HierarchyView : public QWidget
{
	Q_OBJECT

public:
	HierarchyView(const Hierarchy::TestCase* testCase);

private Q_SLOTS:
	void onLevelChanged(int value);
	void onCellSelected(Point cell);
	void onRotate90Deg();

private:
	QSlider* mLevelSlider;
	QLabel* mLevelIndexLabel;
	QLabel* mSelectedCellLabel;
	QScrollArea* mScrollArea;
	HierarchyArea* mHierarchyArea;
};

class HierarchyArea : public QWidget
{
	Q_OBJECT

public:
	HierarchyArea(const Hierarchy::TestCase* testCase);

	void setSelectedLevel(int selectedLevel);

	const Hierarchy::TestCase* testCase() const { return mTestCase; }

Q_SIGNALS:
	void cellSelected(Point pt);

protected:
	virtual void paintEvent(QPaintEvent* evnt) override;
	virtual void mousePressEvent(QMouseEvent* evnt) override;
	virtual void keyPressEvent(QKeyEvent *evnt) override;

private:
	RefPtr<const Hierarchy::TestCase> mTestCase;

	int mSelectedLevel;
	Point mSelectedCell;

	Hierarchy::PathFinder mPathFinder;
	Hierarchy::PathFinder::IterationRes mLastIterationRes;

	Hierarchy::DebugDraw mDebugDraw;
	int mScale;
};