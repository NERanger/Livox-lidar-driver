#pragma once

#include <cstdint>

namespace livox{

namespace utils{

inline bool IsVec3Zero(int32_t x, int32_t y, int32_t z){return (x == 0 && y == 0 && z == 0);}
inline bool IsVec3fInBound(float x, float y, float z, float bound){return !(x > bound || y > bound|| z > bound || x < -bound || y < -bound || z < -bound);}

}

}