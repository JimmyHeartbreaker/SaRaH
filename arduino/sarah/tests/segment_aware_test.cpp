#include <iostream>
#include <cmath>
#include <vector>
#include <cstdlib>
#include <iomanip>
#include "..\pid_controller.h"

#define M_PI_F 3.14159265358979323846f





static inline float pointToSegmentDistSq(const Point2D &p, const Point2D &q, const Point2D &r, Point2D &closest) {
    float vx = q.X - p.X, vy = q.Y - p.Y;
    float wx = r.X - p.X, wy = r.Y - p.Y;
    float vlen2 = vx*vx + vy*vy;
    float t = (vlen2 > 1e-8f) ? (vx*wx + vy*wy) / vlen2 : 0.0f;
    t = fminf(fmaxf(t, 0.0f), 1.0f);
    closest.X = p.X + vx * t;
    closest.Y = p.Y + vy * t;
    float dx = r.X - closest.X;
    float dy = r.Y - closest.Y;
    return dx*dx + dy*dy;
}



// --- TEST ---
int main() {
    const float trueRadius = 2000.0f; // 2 m wall
    const float noiseRange = 80.0f;   // ±80 mm noise
    std::vector<ArcNode> dst(N_POINTS);
    std::vector<LidarScanNormalMeasureRaw> nodes(N_POINTS);

    // Initialize dst as a perfect circular wall
    for (int i = 0; i < N_POINTS; ++i) {
        float angle = i * (M_2PI / N_POINTS);
        dst[i].angle = angle;
        dst[i].dist = trueRadius;
        dst[i].point = toPoint2D(angle, trueRadius);
    }

    // Create noisy scan around that wall
    for (int i = 0; i < N_POINTS; ++i) {
        float angle = i * (M_2PI / N_POINTS);
        float noise = ((rand() % 2001) / 1000.0f - 1.0f) * noiseRange; // [-80, +80]
        float dist = trueRadius + noise;
        nodes[i].angle_z_q6 = (int)(angle * 180.0f / M_PI_F * 128.0f);
        nodes[i].dist_mm_q2 = (int)(dist * 4.0f);
    }

    // Run update
    float diff = read_scan(nodes.data(), dst.data(), N_POINTS, 0.5f, true);

    // Evaluate result
    float mean = 0, stddev = 0;
    for (int i = 0; i < N_POINTS; ++i) {
        mean += dst[i].dist;
    }
    mean /= N_POINTS;
    for (int i = 0; i < N_POINTS; ++i) {
        stddev += powf(dst[i].dist - mean, 2);
    }
    stddev = sqrtf(stddev / N_POINTS);

    std::cout << "Total diff: " << diff << "\n";
    std::cout << "Mean distance: " << mean << " mm\n";
    std::cout << "Std deviation: " << std::fixed << std::setprecision(2) << stddev << " mm\n\n";

    std::cout << "Sample updated nodes:\n";
    for (int i = 0; i < 10; ++i) {
        std::cout << "Angle " << std::setw(3) << i
                  << ": dist=" << std::setw(8) << std::fixed << std::setprecision(2) << dst[i].dist
                  << " (was " << trueRadius << ")\n";
    }
}
