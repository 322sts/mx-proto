#include "Terrain.h"
#include <algorithm>
#include <limits>
#include <numbers>

namespace mx {

Terrain::Terrain(double x0, double dx, std::vector<double> heights)
    : x0_(x0), dx_(dx), h_(std::move(heights)) {}

// Segment i (entre les échantillons i et i+1) et position t dans [0,1].
void Terrain::segment(double x, int& i, double& t) const {
    const double u = clampd((x - x0_) / dx_, 0.0, double(h_.size() - 1) - 1e-9);
    i = int(u);
    t = u - i;
}

// Catmull-Rom : passe par les échantillons, tangente = différence centrée -> C1.
double Terrain::height(double x) const {
    int i; double t;
    segment(x, i, t);
    const int n = int(h_.size());
    const double p0 = h_[std::max(i - 1, 0)], p1 = h_[i], p2 = h_[i + 1], p3 = h_[std::min(i + 2, n - 1)];
    return 0.5 * (2 * p1 + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t +
                  (-p0 + 3 * p1 - 3 * p2 + p3) * t * t * t);
}

double Terrain::slope(double x) const {
    int i; double t;
    segment(x, i, t);
    const int n = int(h_.size());
    const double p0 = h_[std::max(i - 1, 0)], p1 = h_[i], p2 = h_[i + 1], p3 = h_[std::min(i + 2, n - 1)];
    const double dhdt = 0.5 * ((-p0 + p2) + 2 * (2 * p0 - 5 * p1 + 4 * p2 - p3) * t +
                               3 * (-p0 + 3 * p1 - 3 * p2 + p3) * t * t);
    return dhdt / dx_;
}

Vec2 Terrain::normal(double x) const {
    const double s = slope(x);
    return Vec2{-s, 1.0} / std::sqrt(1 + s * s);
}

Vec2 Terrain::closestPoint(Vec2 p, double radius) const {
    auto dist2 = [&](double x) { const double dy = height(x) - p.y, dx = x - p.x; return dx * dx + dy * dy; };
    // Balayage grossier (1 cm) puis affinage par recherche ternaire autour du meilleur.
    const double step = 0.01;
    double best = p.x, bestD2 = std::numeric_limits<double>::max();
    for (double x = p.x - radius; x <= p.x + radius; x += step) {
        const double d2 = dist2(x);
        if (d2 < bestD2) { bestD2 = d2; best = x; }
    }
    double lo = best - step, hi = best + step;
    for (int k = 0; k < 20; ++k) {
        const double m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
        if (dist2(m1) < dist2(m2)) hi = m2; else lo = m1;
    }
    const double x = 0.5 * (lo + hi);
    return {x, height(x)};
}

Terrain Terrain::makeTestTrack() {
    auto s01 = [](double x, double a, double b) { return clampd((x - a) / (b - a), 0, 1); };
    auto smooth = [](double s) { return s * s * (3 - 2 * s); };
    // Forme brute de la piste (m). Les arêtes vives sont arrondies ensuite par un flou.
    auto shape = [&](double x) -> double {
        // Petit saut : tremplin concave (lèvre ~27°), dos raide, puis bosse de réception.
        if (x >= 60 && x < 64) { const double s = s01(x, 60, 64); return 1.0 * s * s; }
        if (x >= 64 && x < 65.5) return 1.0 * (1 - smooth(s01(x, 64, 65.5)));
        if (x >= 71 && x < 74) return 1.2 * smooth(s01(x, 71, 74));
        if (x >= 74 && x < 79) return 1.2 * (1 - s01(x, 74, 79)); // réception ~13°
        // Grosse table : tremplin 2,5 m (lèvre ~32°), plateau 15 m, réception ~20°.
        if (x >= 110 && x < 118) { const double s = s01(x, 110, 118); return 2.5 * s * s; }
        if (x >= 118 && x < 133) return 2.5;
        if (x >= 133 && x < 140) return 2.5 * (1 - s01(x, 133, 140));
        // Whoops : 17 bosses de 60 cm espacées de 3,5 m.
        if (x >= 170 && x < 229.5) return 0.3 * (1 - std::cos(2 * std::numbers::pi * (x - 170) / 3.5));
        // Montée raide (jusqu'à ~37°), plateau, descente (~28° max).
        if (x >= 250 && x < 264) return 7.0 * smooth(s01(x, 250, 264));
        if (x >= 264 && x < 280) return 7.0;
        if (x >= 280 && x < 300) return 7.0 * (1 - smooth(s01(x, 280, 300)));
        return 0.0;
    };
    const double x0 = -20, dx = 0.05, x1 = 420;
    std::vector<double> h;
    for (double x = x0; x <= x1 + 1e-9; x += dx) h.push_back(shape(x));
    // Flou [1/4 1/2 1/4] répété : arrondit lèvres et cassures (~10 cm de rayon), comme la terre tassée.
    for (int pass = 0; pass < 8; ++pass) {
        std::vector<double> o = h;
        for (size_t i = 1; i + 1 < h.size(); ++i) o[i] = 0.25 * h[i - 1] + 0.5 * h[i] + 0.25 * h[i + 1];
        h.swap(o);
    }
    return Terrain(x0, dx, std::move(h));
}

} // namespace mx
