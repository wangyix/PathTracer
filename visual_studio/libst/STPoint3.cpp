// STPoint3.cpp
#include "STPoint3.h"

#if USE_EIGEN

#else

//const STPoint3 STPoint3::Origin(0.0f, 0.0f, 0.0f);
STPoint3 STPoint3::Zero() {
    return STPoint3(0.f, 0.f, 0.f);
}

#endif
