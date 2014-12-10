#ifndef STTRANSFORM4_H
#define STTRANSFORM4_H

#include "stForward.h"
#include <iostream>

class STTransform4
{
public:

	// Constructors
	STTransform4();
	STTransform4(const STTransform4& t);
    STTransform4(float a00, float a01, float a02, float a03,
        float a10, float a11, float a12, float a13,
        float a20, float a21, float a22, float a23,
        float a30, float a31, float a32, float a33);

	// Assignment
	STTransform4& operator = (const STTransform4& t);

	// Element access
	inline const float* operator [] (int row) const
	{
		return _Entries[row];
	}
	inline float* operator [] (int row)
	{
		return _Entries[row];
	}

	// Factory methods for constructing useful transformations
	static STTransform4 Identity();
	static STTransform4 Translation(float tx, float ty, float tz);
	static STTransform4 Scaling(float sx, float sy, float sz);
	static STTransform4 Rotation(float rx, float ry, float rz);

	// In-place arithmetic operators
	STTransform4& operator += (const STTransform4& t);
	STTransform4& operator -= (const STTransform4& t);
	STTransform4& operator *= (const STTransform4& t);
	STTransform4& operator *= (float s);
	STTransform4& operator /= (float s);

	// linear algebra: determinant, transpose, inverse (copy from matt's code)
	// Linear Algebra
    STTransform4 Transpose() const;
    STTransform4 Inverse() const;

    float columnnMagnitude(int col) const;

	friend std::ostream& operator<<(std::ostream& out,const STTransform4& t)
	{
		for(int i=0;i<4;i++)out<<"["<<t._Entries[i][0]<<", "<<t._Entries[i][1]<<", "<<t._Entries[i][2]<<", "<<t._Entries[i][3]<<"]"<<std::endl;return out;
	}

private:
	float _Entries[4][4];
};

// Non in-place arithmetic operators
STTransform4 operator * (const STTransform4& left, const STTransform4& right);
STTransform4 operator * (const STTransform4& left, float right);
STTransform4 operator * (float left, const STTransform4& right);
STTransform4 operator / (const STTransform4& left, float right);
STTransform4 operator + (const STTransform4& left, const STTransform4& right);
STTransform4 operator - (const STTransform4& left, const STTransform4& right);
STVector3 operator * (const STTransform4& left, const STVector3& right);
STPoint3 operator * (const STTransform4& left, const STPoint3& right);


#endif

