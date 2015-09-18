// stForward.h
#ifndef __STFORWARD_H__
#define __STFORWARD_H__

#define USE_EIGEN 1

#if USE_EIGEN

#include <Eigen/Dense>

typedef Eigen::Matrix4f STTransform4;
typedef Eigen::Vector4f STPoint3;
typedef Eigen::Vector4f STVector3;

#else

class STTransform4;
struct STPoint3;
struct STVector3;

#endif

struct STColor3f;
struct STColor4f;
struct STColor4ub;
class STImage;
//class STTransform4;
struct STPoint2;
//struct STPoint3;
class STShape;
class STTexture;
class STTimer;
struct STVector2;
//struct STVector3;

#endif  // __STFORWARD_H__
