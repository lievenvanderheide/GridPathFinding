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

		CORNER_TO_CORNER_0 = 1,
		CORNER_TO_CORNER_1 = 2,
		CORNER_TO_CORNER_2 = 3,
		CORNER_TO_CORNER_3 = 4,
		SHORE_0 = 5,
		SHORE_1 = 6,
		SHORE_2 = 7,
		SHORE_3 = 8,
		SIDE_CORNERS_0 = 9,
		SIDE_CORNERS_1 = 10,
		SIDE_CORNERS_2 = 11,
		SIDE_CORNERS_3 = 12,
		T_VERTICES = 13,
		BEAM_1 = 14,
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
