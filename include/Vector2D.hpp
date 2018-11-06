#ifndef _VECTORS_H
#define _VECTORS_H

#include <cmath>

typedef double VecType;

struct Vector2D{
    VecType x;
    VecType y;

    //multiply vector by scalar
    Vector2D operator*(const VecType& v) {
        return Vector2D(x * v, y * v);
    }

    //divide vector by scalar
    Vector2D operator/(const VecType& v){
        return Vector2D(x / v, x / v);
    }

    //unary vector
    Vector2D operator~(){
        return this / !this;
    }

    //vector length
    Vector2D operator!(){
        return sqrt(pow(x,2), pow(y,2));
    }
};

#endif
