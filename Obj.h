#pragma once

#include <atomic>

class Obj
{
public:
	Obj()
		: mRefCount(1)
	{
	}

	Obj(const Obj& src)
		: mRefCount(1)
	{
	}

	virtual ~Obj()
	{
	}

	void addRef() const
	{
		mRefCount++;
	}

	void release() const
	{
		if(--mRefCount == 0)
		{
			delete this;
		}
	}

private:
	mutable std::atomic<uint32_t> mRefCount;
};

template <class T>
class RefPtr
{
public:
	RefPtr()
		: mPtr(nullptr)
	{
	}

	RefPtr(const RefPtr& src)
	{
		mPtr = src.mPtr;
		mPtr->addRef();
	}

	RefPtr(RefPtr&& src)
	{
		mPtr = src.mPtr;
		src.mPtr = nullptr;
	}

	RefPtr(T* p)
	{
		mPtr = p;
		if(mPtr)
		{
			mPtr->addRef();
		}
	}

	~RefPtr()
	{
		if(mPtr)
		{
			mPtr->release();
		}
	}

	RefPtr<T>& operator = (T* ptr)
	{
		if(mPtr)
		{
			mPtr->release();
		}

		mPtr = ptr;

		if(mPtr)
		{
			mPtr->addRef();
		}

		return *this;
	}

	RefPtr<T>& operator = (const RefPtr<T>& src)
	{
		if(src.mPtr)
		{
			src.mPtr->addRef();
		}

		if(mPtr)
		{
			mPtr->release();
		}

		mPtr = src.mPtr;

		return *this;
	}

	RefPtr<T>& operator = (RefPtr<T>&& src)
	{
		if(mPtr)
		{
			mPtr->release();
		}

		mPtr = src.mPtr;
		src.mPtr = nullptr;

		return *this;
	}

	static RefPtr<T> fromNew(T* ptr)
	{
		RefPtr<T> ret;
		ret.setNew(ptr);
		return ret;
	}

	void setNew(T* ptr)
	{
		if(mPtr)
		{
			mPtr->release();
		}

		mPtr = ptr;
	}

	operator T* () { return mPtr; }
	operator const T* () const { return mPtr; }

	T* ptr() { return mPtr; }
	const T* constPtr() const { return mPtr; }

	T* operator -> () { return mPtr; }
	const T* operator -> () const { return mPtr; }

private:
	T* mPtr;
};
