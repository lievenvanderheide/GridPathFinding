#pragma once

#include <cmath>
#include <cstdint>

#ifdef NDEBUG
#define DIDA_ASSERT(cond)
#define DIDA_ON_DEBUG(s)
#else
#define DIDA_DEBUG

#define DIDA_ASSERT(cond) if(!(cond)) \
	{ \
		printf("Assertion failure: %s\n", #cond); \
		__debugbreak(); \
	}

#define DIDA_ON_DEBUG(s) s
#endif

static const float SQRT_2 = 1.41421356237309504880f;

enum class Axis2 : uint8_t
{
	X = 0,
	Y = 1,
};

constexpr Axis2 otherAxis(Axis2 axis)
{
	return (Axis2)((uint8_t)axis ^ 1);
}

struct Point
{
	int16_t mX;
	int16_t mY;

	Point() { }
	Point(int16_t x, int16_t y)
		: mX(x),
		mY(y)
	{
	}

	int16_t operator [] (int i) const
	{
		return (&mX)[i];
	}

	int16_t& operator [] (int i)
	{
		return (&mX)[i];
	}

	int16_t operator [] (Axis2 axis) const
	{
		return (&mX)[(int)axis];
	}

	int16_t& operator [] (Axis2 axis)
	{
		return (&mX)[(int)axis];
	}

	bool operator == (Point b) const
	{
		return mX == b.mX && mY == b.mY;
	}

	bool operator != (Point b) const
	{
		return mX != b.mX || mY != b.mY;
	}

	bool operator < (Point b) const
	{
		if(mX != b.mX)
			return mX < b.mX;
		else
			return mY < b.mY;
	}

	Point operator << (int n) const
	{
		return Point(
			mX << n,
			mY << n);
	}

	Point operator >> (int n) const
	{
		return Point(
			mX >> n,
			mY >> n);
	}

	Point& operator <<= (int n)
	{
		mX <<= n;
		mY <<= n;
		return *this;
	}

	Point& operator >>= (int n)
	{
		mX >>= n;
		mY >>= n;
		return *this;
	}

	static const Point invalidPoint()
	{
		return Point(-1, -1);
	}
};

class Cost
{
public:
	Cost() { }
	Cost(int straight, int diag)
		: mStraight(straight),
		mDiag(diag)
	{
	}

	Cost operator + (Cost b) const
	{
		return Cost(
			mStraight + b.mStraight,
			mDiag + b.mDiag);
	}

	Cost& operator += (Cost b)
	{
		mStraight += b.mStraight;
		mDiag += b.mDiag;
		return *this;
	}

	static Cost distance(Point a, Point b)
	{
		int16_t xDiff = std::abs(a.mX - b.mX);
		int16_t yDiff = std::abs(a.mY - b.mY);
		if(xDiff < yDiff)
		{
			return Cost(
				yDiff - xDiff,
				xDiff);
		}
		else
		{
			return Cost(
				xDiff - yDiff,
				yDiff);
		}
	}

	static Cost length(int16_t xDiff, int16_t yDiff)
	{
		DIDA_ASSERT(xDiff >= 0 && yDiff >= 0);

		if(xDiff < yDiff)
		{
			return Cost(
				yDiff - xDiff,
				xDiff);
		}
		else
		{
			return Cost(
				xDiff - yDiff,
				yDiff);
		}
	}

	bool operator == (Cost b) const
	{
		return mStraight == b.mStraight && mDiag == b.mDiag;
	}

	bool operator < (Cost b) const
	{
		return cmp(b) < 0;
	}

	bool operator > (Cost b) const
	{
		return cmp(b) > 0;
	}

	int64_t cmp(Cost b) const
	{
		float aFloat = mStraight + SQRT_2 * mDiag;
		float bFloat = b.mStraight + SQRT_2 * b.mDiag;
		if(aFloat > bFloat)
			return 1;
		else if(aFloat < bFloat)
			return -1;
		else
			return 0;

		int straightDiff = mStraight - b.mStraight;
		int diagDiff = b.mDiag - mDiag;
		if((straightDiff ^ diagDiff) < 0)
		{
			return straightDiff < diagDiff;
		}
		else
		{
			int64_t a = (int64_t)straightDiff * (int64_t)straightDiff;
			int64_t b = 2 * (int64_t)diagDiff * (int64_t)diagDiff;
			if(straightDiff >= 0)
			{
				return a - b;
			}
			else
			{
				return a - b;
			}
		}
	}

	static Cost maxCost()
	{
		return Cost(INT_MAX, INT_MAX);
	}

private:
	int mStraight;
	int mDiag;
};

// Shorted distance, when only allowed to move horizontally, vertically and
// diagonally. Named after the street pattern of San Francisco.
static inline float sanFranDistance(Point a, Point b)
{
	int16_t xDiff = std::abs(a.mX - b.mX);
	int16_t yDiff = std::abs(a.mY - b.mY);
	if(xDiff < yDiff)
		return (float)xDiff * SQRT_2 + (float)(yDiff - xDiff);
	else
		return (float)yDiff * SQRT_2 + (float)(xDiff - yDiff);
}

static inline bool isPow2(int i)
{
	return (i & (i - 1)) == 0;
}
