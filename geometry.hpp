#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include <cmath>
#include <iostream>

template <typename T, typename F>
struct Vec2
{
    union
    {
        struct
        {
            T u, v;
        };
        T raw[2];
    };
    Vec2() : u(0), v(0) {}
    Vec2(T _u, T _v) : u(_u), v(_v) {}
    inline bool operator==(const Vec2<T, F> &V) const { return u == V.u && v == V.v; }
    inline Vec2<T, F> operator+(const Vec2<T, F> &V) const { return Vec2<T, F>(u + V.u, v + V.v); }
    inline Vec2<T, F> operator-(const Vec2<T, F> &V) const { return Vec2<T, F>(u - V.u, v - V.v); }
    inline Vec2<T, F> operator*(F scalar) const { return Vec2<T, F>((T)(u * scalar), (T)(v * scalar)); }
    inline T dot_product(const Vec2<T, F> &V) const { return u * V.u + v * V.v; }
    T norm() const { return std::sqrt(u * u + v * v); }
    friend std::ostream &operator<<(std::ostream &s, Vec2<T, F> &V);
};

template <typename T, typename F>
struct Vec3
{
    union
    {
        struct
        {
            T x, y, z;
        };
        T raw[3];
    };
    Vec3() : x(0), y(0), z(0) {}
    Vec3(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}
    inline bool operator==(const Vec3<T, F> &v) const { return x == v.x && y == v.y && z == v.z; }
    inline Vec3<T, F> operator+(const Vec3<T, F> &v) const { return Vec3<T, F>(x + v.x, y + v.y, z + v.z); }
    inline Vec3<T, F> operator-(const Vec3<T, F> &v) const { return Vec3<T, F>(x - v.x, y - v.y, z - v.z); }
    inline Vec3<T, F> operator*(F scalar) const { return Vec3<T, F>((T)(x * scalar), (T)(y * scalar), (T)(z * scalar)); }
    inline T dot_product(const Vec3<T, F> &v) const { return x * v.x + y * v.y + z * v.z; }
    inline Vec3<T, F> cross_product(const Vec3<T, F> &v) const { return Vec3<T, F>(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x); }
    T norm() const { return std::sqrt(x * x + y * y + z * z); }
    Vec3<T, F> &normalized()
    {
        *this = *this * (1. / norm());
        return *this;
    }
    friend std::ostream &operator<<(std::ostream &s, Vec3<T, F> &v);
};

typedef Vec2<float, float> Vec2f;
typedef Vec2<int, float> Vec2i;
typedef Vec3<float, float> Vec3f;
typedef Vec3<int, float> Vec3i;

template <typename T, typename F>
std::ostream &operator<<(std::ostream &s, Vec2<T, F> &V)
{
    s << "(" << V.x << ", " << V.y << ")\n";
    return s;
}

template <typename T, typename F>
std::ostream &operator<<(std::ostream &s, Vec3<T, F> &v)
{
    s << "(" << v.x << ", " << v.y << ", " << v.z << ")\n";
    return s;
}

#endif //__GEOMETRY_H__