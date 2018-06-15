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
	case SideBar::TestCase::TERRAIN:
		{
			QFrame* grayArea = new QFrame();
			grayArea->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
			grayArea->setStyleSheet("background-color:gray;");
			setCentralWidget(grayArea);
		}
		break;

	case SideBar::TestCase::CORNER_TO_CORNER:
		showTestCaseFromFile(":TestCases/CornerToCorner.png");
		break;

	case SideBar::TestCase::CORNER_TO_OFF_GRID_DIAG:
		showTestCaseFromFile(":TestCases/CornerToOffGridDiag.png");
		break;

	case SideBar::TestCase::SHORE:
		showTestCaseFromFile(":TestCases/Shore.png");
		break;

	case SideBar::TestCase::SIDE_CORNERS:
		showTestCaseFromFile(":TestCases/SideCorners.png");
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
	if(testCase)
	{
		HierarchyView* hierarchyView = new HierarchyView(testCase);
		setCentralWidget(hierarchyView);
	}
	else
	{
		QString errorMsg = QString("Failed to load %1.").arg(fileName);

		QLabel* errorLabel = new QLabel(errorMsg);
		errorLabel->setAlignment(Qt::AlignCenter);
		errorLabel->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
		errorLabel->setStyleSheet("background-color:gray;");
		setCentralWidget(errorLabel);
	}
}