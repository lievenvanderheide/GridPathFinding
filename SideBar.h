#pragma once

#include <QListWidget>

class SideBar : public QListWidget
{
	Q_OBJECT

public:
	enum class TestCase
	{
		NONE = -1,
		TERRAIN = 0,

		CORNER_TO_CORNER = 1,
		CORNER_TO_OFF_GRID_DIAG = 2,
		SHORE = 3,
		SIDE_CORNERS = 4,
		T_VERTICES = 5,
		BEAM_1 = 6,
	};

	SideBar(QWidget *parent = nullptr);
	~SideBar();

Q_SIGNALS:
	void testCaseSelected(TestCase testCase);

private Q_SLOTS:
	void onListItemSelected(QListWidgetItem* item);

private:
	QFont mDefaultFont;
	QFont mSelectedFont;

	TestCase mCurTestCase;
};
