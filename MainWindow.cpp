#include "pch.h"
#include "MainWindow.h"
#include "HierarchyView.h"
#include "SideBar.h"

MainWindow::MainWindow()
{
	QFrame* grayArea = new QFrame();
	grayArea->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
	grayArea->setStyleSheet("background-color:gray;");
	setCentralWidget(grayArea);
	
	QDockWidget* leftDockWidget = new QDockWidget();
	addDockWidget(Qt::LeftDockWidgetArea, leftDockWidget);

	mSideBar = new SideBar();
	leftDockWidget->setWidget(mSideBar);
	QObject::connect(mSideBar, &SideBar::testCaseSelected,
		this, &MainWindow::onTestCaseSelected);
}

QSize MainWindow::sizeHint() const
{
	return QSize(1200, 800);
}

void MainWindow::onTestCaseSelected(SideBar::TestCase testCase)
{
	switch(testCase)
	{
	case SideBar::TestCase::NONE:
		{
			QFrame* grayArea = new QFrame();
			grayArea->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
			grayArea->setStyleSheet("background-color:gray;");
			setCentralWidget(grayArea);
		}
		break;

	case SideBar::TestCase::CORNER_TO_CORNER_0:
		showTestCaseFromFile(":TestCases/CornerToCorner0.png");
		break;

	case SideBar::TestCase::CORNER_TO_CORNER_1:
		showTestCaseFromFile(":TestCases/CornerToCorner1.png");
		break;

	case SideBar::TestCase::CORNER_TO_CORNER_2:
		showTestCaseFromFile(":TestCases/CornerToCorner2.png");
		break;

	case SideBar::TestCase::CORNER_TO_CORNER_3:
		showTestCaseFromFile(":TestCases/CornerToCorner3.png");
		break;

	case SideBar::TestCase::SHORE_0:
		showTestCaseFromFile(":TestCases/Shore0.png");
		break;

	case SideBar::TestCase::SHORE_1:
		showTestCaseFromFile(":TestCases/Shore1.png");
		break;

	case SideBar::TestCase::SHORE_2:
		showTestCaseFromFile(":TestCases/Shore2.png");
		break;

	case SideBar::TestCase::SHORE_3:
		showTestCaseFromFile(":TestCases/Shore3.png");
		break;

	case SideBar::TestCase::SIDE_CORNERS_0:
		showTestCaseFromFile(":TestCases/SideCorners0.png");
		break;

	case SideBar::TestCase::SIDE_CORNERS_1:
		showTestCaseFromFile(":TestCases/SideCorners1.png");
		break;

	case SideBar::TestCase::SIDE_CORNERS_2:
		showTestCaseFromFile(":TestCases/SideCorners2.png");
		break;

	case SideBar::TestCase::SIDE_CORNERS_3:
		showTestCaseFromFile(":TestCases/SideCorners3.png");
		break;

	case SideBar::TestCase::T_VERTICES:
		showTestCaseFromFile(":TestCases/TVertices.png");
		break;

	case SideBar::TestCase::BEAM_1:
		showTestCaseFromFile(":TestCases/Beam1.png");
		break;

	default:
		DIDA_ASSERT(!"not implemented yet");
		break;
	}
}

void MainWindow::showTestCaseFromFile(const char* fileName)
{
	RefPtr<Hierarchy::TestCase> testCase = Hierarchy::TestCase::loadFromFile(fileName);

	HierarchyView* hierarchyView = new HierarchyView(testCase);
	setCentralWidget(hierarchyView);
}