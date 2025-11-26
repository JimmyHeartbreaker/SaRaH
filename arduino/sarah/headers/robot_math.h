#ifndef ROBOT_MATH_H
#define ROBOT_MATH_H

#include <cmath>
#include <numeric>

const float M_2PI = (M_PI*2.0f);
const float M_PIF = float(M_PI);
const float M_PI_F = float(M_PI);
const float INV_TWO_PI = (1.0f / M_2PI);
const float oneDegree = 1 * float(M_PI / 180);

typedef struct Point2D
{
    float X;
    float Y;
} __attribute__((packed)) Point2D;

static inline float size(const Point2D& p)
{
    return p.Y*p.Y + p.X*p.X;
}

static inline float mag(const Point2D& p)
{
    return pow(p.Y*p.Y + p.X*p.X, 0.5);
}

static inline float mag(float y, float x)
{
    return pow(y*y + x*x, 0.5);
}
static inline Point2D div(const Point2D& a, float b)
{
    float oneOverB = 1/b;

    return { a.X *oneOverB,a.Y*oneOverB};
}
static inline Point2D div(const Point2D& a,const Point2D& b)
{
    return { b.X!= 0 ? a.X /b.X : 0, b.Y!= 0 ? a.Y / b.Y : 0};
}
static inline Point2D div(const Point2D& a, unsigned long b)
{
    float inv = 1.0f / static_cast<float>(b);
    return { a.X *inv,a.Y*inv};
}
static inline float cross_scalar(const Point2D& a,const Point2D& b)
{
    return  a.X * b.Y - a.Y * b.X;
}


static inline float dot(const Point2D& a,const Point2D& b)
{
    return a.X * b.X + a.Y * b.Y;
}
static inline float det(const Point2D& a,const Point2D& b)
{
    return a.X * b.Y - a.Y * b.X;
}
static inline float angleBetween(const Point2D& a,const Point2D& b)
{
    return atan2(det(a,b),dot(a,b));
}

static inline Point2D normal(const Point2D& v)
{
    return {v.Y,-v.X};
}
static inline Point2D normal_ccw(const Point2D& v)
{
    return {-v.Y,v.X};
}
static inline Point2D normalize(const Point2D& v)
{
    return div(v,mag(v));
}
static inline Point2D toPoint2D(float angle, float dist)
{
    return { dist *  sinf(angle), dist * cosf(angle)};
}
static unsigned short  toIndex(float angle,int nPoints)
{
	return  (unsigned short )(std::round((angle/M_2PI ) * nPoints) + nPoints )%nPoints;
}

static unsigned short  toIndexFloor(float angle,int nPoints)
{
	return  (unsigned short )(std::floor((angle/M_2PI ) * nPoints) + nPoints )%nPoints;
}
static inline float toAngle(unsigned short index,int nPoints)
{
    return index* M_2PI / nPoints ;
}
static inline float toAngle(const Point2D& point)
{
    return atan2f(point.X, point.Y);
}
static inline Point2D add(const Point2D& a, const Point2D& b)
{
    return { a.X + b.X,a.Y+b.Y};
}

static inline Point2D rotate(const Point2D& a, float angle)
{
    return { a.X * cosf(angle) + a.Y * sinf(angle),-a.X * sinf(angle) + a.Y * cosf(angle)};
}

static inline Point2D rotate(const Point2D& a, float cos_f,float sin_f)
{
    return { a.X * cos_f + a.Y * sin_f,-a.X * sin_f + a.Y * cos_f};
}

static inline Point2D sub(const Point2D& a,const Point2D& b)
{
    return { a.X - b.X,a.Y-b.Y};
}
static inline Point2D mul(const Point2D& a, float b)
{

    return { a.X *b,a.Y*b};
}



static float standardDeviation(const float data[], int size) {
    if (size <= 0) return 0.0;

    // Compute mean
    float mean = std::accumulate(data, data + size, 0.0) / size;

    // Compute variance
    float variance = 0.0;
    for (int i = 0; i < size; ++i) {
        float diff = data[i] - mean;
        variance += diff * diff;
    }
    variance /= size; // or (size - 1) for sample stddev

    return std::sqrt(variance);
}

static float standardDeviation_special(const float data[], int size) {
    if (size <= 0) return 0.0;

    // Compute mean
    float mean = std::accumulate(data, data + size, 0.0) / size;

    // Compute variance
    float variance = 0.0;
    for (int i = 0; i < size; ++i) {
        float diff = fabs(data[i] - mean);
        variance += diff * diff;
    }
    variance /= size; // or (size - 1) for sample stddev

    float std = std::sqrt(variance);
    float cv = std/mean;

    return 1 / ( 1+ cv );
}
#endif