#include "math.hpp"

using namespace stir;

vec2 TEST_vec2(vec2 a) {
    return vec2(1.0f, 2.0f) + a;
}

vec3 TEST_vec3(vec3 a, vec3 b) {
    return cross(a, b);
}

#if 1
vec3 TEST_quat(float an, vec3 ax, quat q) {
    return rot(q, ax);
//    return from_axis_angle(ax, an);
    //return mul(quat(1,2,3,4), q);
}

// matrix TEST_matrix(matrix a, matrix b) {
//     return mul(a, b);
//     // return inverse_tr(a);
// }

// vec4 TEST_vec4(vec4 a, vec4 b) {
//     return vec4(dot(a, b));
// }
#endif
