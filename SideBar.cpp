#include "SideBar.h"

SideBar::SideBar(QWidget *parent)
	: QListWidget(parent),
	mCurTestCase(TestCase::NONE)
{
	mDefaultFont = font();
	mSelectedFont = font();
	mSelectedFont.setBold(true);
	mSelectedFont.setItalic(true);

	const char* testCaseNames[] = 
	{
		"Terrain",
		"Corner to corner 0",
		"Corner to corner 1",
		"Corner to corner 2",
		"Corner to corner 3",
		"Shore 0",
		"Shore 1",
		"Shore 2",
		"Shore 3",
		"Side Corners 0",
		"Side Corners 1",
		"Side Corners 2",
		"Side Corners 3",
		"T-Vertices",
		"Beam 1",
	};

	for(const char* testCaseName : testCaseNames)
	{
		QListWidgetItem* item = new QListWidgetItem(testCaseName);
		item->setFont(mDefaultFont);
		addItem(item);
	}

	setSelectionMode(QAbstractItemView::NoSelection);
	QObject::connect(this, &QListWidget::itemDoubleClicked, 
		this, &SideBar::onListItemSelected);
}

SideBar::~SideBar()
{
}

void SideBar::onListItemSelected(QListWidgetItem* selectedItem)
{
	if(mCurTestCase != TestCase::NONE)
	{
		QListWidgetItem* widgetItem = item((int)mCurTestCase);
		widgetItem->setFont(mDefaultFont);
	}

	selectedItem->setFont(mSelectedFont);
	mCurTestCase = (TestCase)row(selectedItem);
	testCaseSelected(mCurTestCase);
}