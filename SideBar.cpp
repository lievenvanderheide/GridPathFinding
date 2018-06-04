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
		"Corner to corner",
		"Shore",
		"Side Corners",
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