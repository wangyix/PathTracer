// STVector3.cpp
#include "STVector3.h"

#if USE_EIGEN

#else

/*const STVector3 STVector3::Zero(0.0f, 0.0f, 0.0f);
const STVector3 STVector3::eX(1.0f, 0.0f, 0.0f);
const STVector3 STVector3::eY(0.0f, 1.0f, 0.0f);
const STVector3 STVector3::eZ(0.0f, 0.0f, 1.0f);*/

STVector3 STVector3::Zero() {
    return STVector3(0.f, 0.f, 0.f);
}

#endif
