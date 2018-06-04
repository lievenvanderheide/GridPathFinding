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
		SHORE = 2,
		SIDE_CORNERS = 3,
		T_VERTICES = 4,
		BEAM_1 = 5,
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
