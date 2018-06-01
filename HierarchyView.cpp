#include "pch.h"
#include "HierarchyView.h"

HierarchyView::HierarchyView(const Hierarchy::TestCase* testCase)
{
	QVBoxLayout* vLayout = new QVBoxLayout();
	vLayout->setMargin(0);
	setLayout(vLayout);

	QHBoxLayout* topBarLayout = new QHBoxLayout();
	topBarLayout->setMargin(4);
	vLayout->addLayout(topBarLayout);

	topBarLayout->addWidget(new QLabel("Level:"));

	mLevelSlider = new QSlider(Qt::Horizontal);
	mLevelSlider->setValue(0);
	mLevelSlider->setTickInterval(1);
	mLevelSlider->setMinimum(0);
	mLevelSlider->setMaximum(testCase->hierarchy()->numLevels() - 1);
	mLevelSlider->setMinimumWidth(200);
	QObject::connect(mLevelSlider, &QSlider::valueChanged,
		this, &HierarchyView::onLevelChanged);
	topBarLayout->addWidget(mLevelSlider);

	mLevelIndexLabel = new QLabel("0");
	topBarLayout->addWidget(mLevelIndexLabel);

	topBarLayout->addSpacing(32);

	topBarLayout->addWidget(new QLabel("Selected cell:"));
	
	mSelectedCellLabel = new QLabel("<none>");
	topBarLayout->addWidget(mSelectedCellLabel);

	topBarLayout->addStretch(1);

	QScrollArea* scrollArea = new QScrollArea();
	vLayout->addWidget(scrollArea);

	mHierarchyArea = new HierarchyArea(testCase);
	QObject::connect(mHierarchyArea, &HierarchyArea::cellSelected,
		this, &HierarchyView::onCellSelected);
	scrollArea->setWidget(mHierarchyArea);
	scrollArea->setAlignment(Qt::AlignCenter);
	scrollArea->ensureVisible(0, 2048, 0, 0);
}

void HierarchyView::onLevelChanged(int value)
{
	mLevelIndexLabel->setText(QString::number(value));
	mHierarchyArea->setSelectedLevel(value);

	mSelectedCellLabel->setText("<none>");
}

void HierarchyView::onCellSelected(Point cell)
{
	char txt[32];
	sprintf(txt, "%d, %d", cell.mX, cell.mY);
	mSelectedCellLabel->setText(txt);
}

HierarchyArea::HierarchyArea(const Hierarchy::TestCase* testCase)
	: mTestCase(testCase),
	mDebugDraw(testCase->hierarchy())
{
	setFocusPolicy(Qt::StrongFocus);

	const Hierarchy::Hierarchy* hierarchy = testCase->hierarchy();

	mScale = 1;
	int width = hierarchy->width();
	int height = hierarchy->height();
	while(width < 512 && height < 512)
	{
		width *= 2;
		height *= 2;
		mScale *= 2;
	}

	setMinimumSize(width, height);

	mSelectedLevel = 0;
	mSelectedCell = Point::invalidPoint();

	mPathFinder.mHierarchy = hierarchy;
	mLastIterationRes = mPathFinder.begin(mTestCase->rootsBegin(), mTestCase->numRoots());
}

void HierarchyArea::setSelectedLevel(int selectedLevel)
{
	mSelectedLevel = selectedLevel;
	mSelectedCell = Point::invalidPoint();
	update();
}

void HierarchyArea::paintEvent(QPaintEvent* evnt)
{
	QPainter painter(this);

	QRect rect(0, 0, width(), height());
	
	const Hierarchy::Hierarchy* hierarchy = mTestCase->hierarchy();
	hierarchy->drawLevel0AsBase(painter, rect);

	painter.setOpacity(.7f);
	hierarchy->drawLevel(painter, mSelectedLevel, rect);

	mDebugDraw.drawToPainter(painter, mScale);

	if(mSelectedCell != Point::invalidPoint())
	{
		painter.setPen(QPen(QColor(255, 0, 0)));

		int cellSize = 1 << mSelectedLevel;
		painter.drawRect(QRect(
			mSelectedCell.mX * cellSize, 
			mSelectedCell.mY * cellSize,
			cellSize - 1, cellSize - 1));
	}
}

void HierarchyArea::mousePressEvent(QMouseEvent* evnt)
{
	QPoint pos = evnt->pos();

	int cellSize = 1 << mSelectedLevel;

	update(QRect(
		mSelectedCell.mX * cellSize, 
		mSelectedCell.mY * cellSize,
		cellSize, cellSize));

	mSelectedCell.mX = pos.x() / cellSize;
	mSelectedCell.mY = pos.y() / cellSize;

	update(QRect(
		mSelectedCell.mX * cellSize, 
		mSelectedCell.mY * cellSize,
		cellSize, cellSize));

	cellSelected(mSelectedCell);
}

void HierarchyArea::keyPressEvent(QKeyEvent *evnt)
{
	if(evnt->key() == Qt::Key_Right &&
		mLastIterationRes == Hierarchy::PathFinder::IterationRes::IN_PROGRESS)
	{
		mLastIterationRes = mPathFinder.iteration(&mDebugDraw);
		update();
	}
}