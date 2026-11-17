#include "Terrain.h"
#include <algorithm>
#include <numbers>

namespace mx {

Terrain::Terrain(double x0, double z0, double cell, int nx, int nz, std::vector<float> heights)
    : x0_(x0), z0_(z0), cell_(cell), nx_(nx), nz_(nz), h_(std::move(heights)) {}

float Terrain::at(int ix, int iz) const {
    ix = std::clamp(ix, 0, nx_ - 1);
    iz = std::clamp(iz, 0, nz_ - 1);
    return h_[size_t(iz) * nx_ + ix];
}

namespace {
// Catmull-Rom 1D sur 4 échantillons, valeur et dérivée par rapport à t.
void catmull(double p0, double p1, double p2, double p3, double t, double& v, double& d) {
    const double a = -p0 + 3 * p1 - 3 * p2 + p3, b = 2 * p0 - 5 * p1 + 4 * p2 - p3, c = -p0 + p2;
    v = 0.5 * (2 * p1 + c * t + b * t * t + a * t * t * t);
    d = 0.5 * (c + 2 * b * t + 3 * a * t * t);
}
} // namespace

double Terrain::sample(double x, double z, double& dhdx, double& dhdz) const {
    const double u = clampd((x - x0_) / cell_, 0.0, nx_ - 1 - 1e-9);
    const double w = clampd((z - z0_) / cell_, 0.0, nz_ - 1 - 1e-9);
    const int ix = int(u), iz = int(w);
    const double tx = u - ix, tz = w - iz;
    // Bicubique séparable : 4 lignes en x, puis une passe en z (sur les valeurs et sur les pentes).
    double row[4], rowDx[4];
    for (int j = 0; j < 4; ++j)
        catmull(at(ix - 1, iz - 1 + j), at(ix, iz - 1 + j), at(ix + 1, iz - 1 + j), at(ix + 2, iz - 1 + j), tx, row[j], rowDx[j]);
    double h, dz, dx, unused;
    catmull(row[0], row[1], row[2], row[3], tz, h, dz);
    catmull(rowDx[0], rowDx[1], rowDx[2], rowDx[3], tz, dx, unused);
    dhdx = dx / cell_;
    dhdz = dz / cell_;
    return h;
}

double Terrain::height(double x, double z) const {
    double dx, dz;
    return sample(x, z, dx, dz);
}

Vec3 Terrain::normal(double x, double z) const {
    double dx, dz;
    sample(x, z, dx, dz);
    return normalize(Vec3{-dx, 1.0, -dz});
}

Terrain Terrain::makeTestTrack() {
    auto s01 = [](double x, double a, double b) { return clampd((x - a) / (b - a), 0, 1); };
    auto smooth = [](double s) { return s * s * (3 - 2 * s); };
    // Profil de la piste le long de x (m). Les arêtes vives sont arrondies ensuite par un flou.
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
    // Profil fin (5 cm) + flou [1/4 1/2 1/4] répété : lèvres et cassures arrondies (~10 cm), comme la terre tassée.
    const double px0 = -40, pdx = 0.05;
    std::vector<double> prof;
    for (double x = px0; x <= 440 + 1e-9; x += pdx) prof.push_back(shape(x));
    for (int pass = 0; pass < 8; ++pass) {
        std::vector<double> o = prof;
        for (size_t i = 1; i + 1 < prof.size(); ++i) o[i] = 0.25 * prof[i - 1] + 0.5 * prof[i] + 0.25 * prof[i + 1];
        prof.swap(o);
    }
    auto profile = [&](double x) {
        const double u = clampd((x - px0) / pdx, 0, double(prof.size() - 1) - 1e-9);
        const size_t i = size_t(u);
        return prof[i] + (prof[i + 1] - prof[i]) * (u - double(i));
    };
    // Largeur : pleine hauteur sur |z| <= 5 m, raccord en S jusqu'au plat à |z| = 9 m (flancs ~30° max sur la montée).
    auto widthWeight = [&](double z) { return 1 - smooth(s01(std::abs(z), 5, 9)); };

    const double x0 = -30, z0 = -40, cell = 0.1;
    const int nx = int((430 - x0) / cell) + 1, nz = int((40 - z0) / cell) + 1;
    std::vector<float> h(size_t(nx) * nz);
    std::vector<double> px(nx);
    for (int ix = 0; ix < nx; ++ix) px[ix] = profile(x0 + ix * cell);
    for (int iz = 0; iz < nz; ++iz) {
        const double w = widthWeight(z0 + iz * cell);
        for (int ix = 0; ix < nx; ++ix) h[size_t(iz) * nx + ix] = float(px[ix] * w);
    }
    return Terrain(x0, z0, cell, nx, nz, std::move(h));
}

Terrain Terrain::makeFlat() {
    const int nx = 1001, nz = 201; // 500 m x 100 m, mailles de 50 cm
    return Terrain(-50, -50, 0.5, nx, nz, std::vector<float>(size_t(nx) * nz, 0.0f));
}

} // namespace mx
