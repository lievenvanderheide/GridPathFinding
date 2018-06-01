#pragma once

#include <QMainWindow>

#include "SideBar.h"
#include "Hierarchy.h"

class Terrain;

class QListWidgetItem;

class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	MainWindow();

	virtual QSize sizeHint() const override;

private Q_SLOTS:
	void onTestCaseSelected(SideBar::TestCase testCase);

private:
	void showTestCaseFromFile(const char* fileName);

	SideBar* mSideBar;
};
