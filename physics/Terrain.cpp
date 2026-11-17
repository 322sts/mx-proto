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

const Spawn& Terrain::spawn(const std::string& name) const {
    for (const Spawn& s : spawns_)
        if (s.name == name) return s;
    return spawns_.front();
}

namespace {
// Catmull-Rom 1D sur 4 échantillons, valeur et dérivée par rapport à t.
void catmull(double p0, double p1, double p2, double p3, double t, double& v, double& d) {
    const double a = -p0 + 3 * p1 - 3 * p2 + p3, b = 2 * p0 - 5 * p1 + 4 * p2 - p3, c = -p0 + p2;
    v = 0.5 * (2 * p1 + c * t + b * t * t + a * t * t * t);
    d = 0.5 * (c + 2 * b * t + 3 * a * t * t);
}
double s01(double x, double a, double b) { return clampd((x - a) / (b - a), 0, 1); }
double smooth(double s) { return s * s * (3 - 2 * s); }
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
    // ---- 1. Ligne médiane : "tortue" qui enchaîne lignes droites et arcs (virages à droite = +).
    // Rectangle aux coins arrondis de rayons différents + chicane ; les deux dernières droites
    // sont calculées pour que la boucle se referme exactement sur le départ (0, 0, cap +x).
    constexpr double ds = 0.1;
    std::vector<TrackPoint> path;
    std::vector<double> bermH; // hauteur de berm (côté extérieur) à chaque point
    double x = 0, z = 0, th = 0, s = 0;
    auto push = [&](double curv, double berm, double along = 0) {
        path.push_back({x, z, th, s + along, curv});
        bermH.push_back(berm);
    };
    auto straight = [&](double len) {
        const int n = int(std::round(len / ds));
        for (int i = 0; i < n; ++i) {
            push(0, 0);
            x += std::cos(th) * ds;
            z += std::sin(th) * ds;
            s += ds;
        }
        x += std::cos(th) * (len - n * ds); // reste < ds
        z += std::sin(th) * (len - n * ds);
        s += len - n * ds;
    };
    // Arc de rayon r, angle a (rad), sens turn (+1 droite, -1 gauche) ; berm de hauteur H côté
    // extérieur, qui monte et descend sur les 6 premiers / derniers mètres.
    auto arc = [&](double r, double a, int turn, double H) {
        const double len = r * a, cx = x - std::sin(th) * r * turn, cz = z + std::cos(th) * r * turn, th0 = th;
        const int n = int(std::round(len / ds));
        for (int i = 0; i < n; ++i) {
            const double l = i * len / n;
            th = th0 + turn * l / r;
            x = cx + std::sin(th) * r * turn;
            z = cz - std::cos(th) * r * turn;
            push(turn / r, H * smooth(clampd(std::min(l, len - l) / 6.0, 0, 1)), l);
        }
        th = th0 + turn * a;
        x = cx + std::sin(th) * r * turn;
        z = cz - std::cos(th) * r * turn;
        s += len;
    };
    const double q = std::numbers::pi / 2;
    straight(113);            // S1 : départ, double, table          (cap +x)
    const double c1 = s;
    arc(12, q, +1, 1.4);      // berm serré à droite
    const double s2 = s;
    straight(38);             // S2 : rouleaux                         (cap +z)
    arc(20, q, +1, 0.0);      // grand virage à plat
    straight(30);             // S3a                                    (cap -x)
    arc(12, q / 2, -1, 0.0);  // chicane gauche...
    arc(12, q / 2, +1, 0.0);  // ...droite
    const double s3b = s;
    straight(x - (-15));      // S3b : whoops + petit saut, jusqu'à x = -15
    arc(10, q, +1, 1.6);      // épingle relevée
    const double s4 = s;
    straight(z - 25);         // S4 : petite table, jusqu'à z = 25   (cap -z)
    arc(25, q, +1, 1.0);      // grande courbe relevée, retour au départ
    const double L = s;
    (void)c1;

    // ---- 2. Profil le long du circuit (sauts, rouleaux, whoops), puis arrondi des arêtes.
    auto shape = [&](double t) -> double {
        // Double : tremplin concave (lèvre ~30°), réception en pente douce.
        if (t >= 38 && t < 42) { const double u = s01(t, 38, 42); return 1.3 * u * u; }
        if (t >= 42 && t < 43.5) return 1.3 * (1 - smooth(s01(t, 42, 43.5)));
        if (t >= 52 && t < 55) return 1.4 * smooth(s01(t, 52, 55));
        if (t >= 55 && t < 62) return 1.4 * (1 - s01(t, 55, 62));
        // Table : 2 m, plateau 12 m.
        if (t >= 74 && t < 80) { const double u = s01(t, 74, 80); return 2.0 * u * u; }
        if (t >= 80 && t < 92) return 2.0;
        if (t >= 92 && t < 99) return 2.0 * (1 - s01(t, 92, 99));
        // Rouleaux : 3 bosses de 50 cm.
        for (double c : {s2 + 8, s2 + 17, s2 + 26})
            if (std::abs(t - c) < 2.5) return 0.25 * (1 + std::cos(std::numbers::pi * (t - c) / 2.5));
        // Whoops : 12 bosses de 50 cm espacées de 3,5 m, puis petit saut.
        if (t >= s3b + 5 && t < s3b + 47) return 0.25 * (1 - std::cos(2 * std::numbers::pi * (t - s3b - 5) / 3.5));
        if (t >= s3b + 55 && t < s3b + 58.5) { const double u = s01(t, s3b + 55, s3b + 58.5); return 0.9 * u * u; }
        if (t >= s3b + 58.5 && t < s3b + 60) return 0.9 * (1 - smooth(s01(t, s3b + 58.5, s3b + 60)));
        if (t >= s3b + 64 && t < s3b + 66) return 1.0 * smooth(s01(t, s3b + 64, s3b + 66));
        if (t >= s3b + 66 && t < s3b + 72) return 1.0 * (1 - s01(t, s3b + 66, s3b + 72));
        // Petite table dans S4.
        if (t >= s4 + 10 && t < s4 + 15) { const double u = s01(t, s4 + 10, s4 + 15); return 1.2 * u * u; }
        if (t >= s4 + 15 && t < s4 + 22) return 1.2;
        if (t >= s4 + 22 && t < s4 + 28) return 1.2 * (1 - s01(t, s4 + 22, s4 + 28));
        return 0.0;
    };
    const double pds = 0.05;
    std::vector<double> prof;
    for (double t = 0; t <= L + 1e-9; t += pds) prof.push_back(shape(t));
    // Flou [1/4 1/2 1/4] répété : lèvres et cassures arrondies (~10 cm), comme la terre tassée.
    for (int pass = 0; pass < 8; ++pass) {
        std::vector<double> o = prof;
        for (size_t i = 1; i + 1 < prof.size(); ++i) o[i] = 0.25 * prof[i - 1] + 0.5 * prof[i] + 0.25 * prof[i + 1];
        prof.swap(o);
    }
    auto profile = [&](double t) {
        const double u = clampd(t / pds, 0, double(prof.size() - 1) - 1e-9);
        const size_t i = size_t(u);
        return prof[i] + (prof[i + 1] - prof[i]) * (u - double(i));
    };
    auto berm = [&](double t, int& side) {
        const size_t i = std::min(size_t(std::max(t, 0.0) / ds), path.size() - 1);
        side = path[i].curvature > 0 ? -1 : 1; // virage à droite : extérieur à gauche (d < 0)
        return bermH[i];
    };

    // ---- 3. Grille : pour chaque maille, point le plus proche de la ligne médiane (abscisse s,
    // écart latéral d, + = à droite), puis hauteur = profil (pleine largeur, flancs en S) + berm.
    const double x0 = -60, z0 = -40, cell = 0.1, R = 9.0; // R : portée de la piste (piste + flancs)
    const int nx = int(220 / cell) + 1, nz = int(150 / cell) + 1;
    std::vector<float> best(size_t(nx) * nz, float(R * R)), sv(best.size()), dv(best.size());
    const size_t stride = 3; // segments de 30 cm
    for (size_t i = 0; i < path.size(); i += stride) {
        const TrackPoint& a = path[i];
        const TrackPoint& b = path[(i + stride) % path.size()];
        const double ex = b.x - a.x, ez = b.z - a.z, len = std::hypot(ex, ez);
        if (len < 1e-9) continue;
        const double ux = ex / len, uz = ez / len;
        const int ix0 = std::max(0, int((std::min(a.x, b.x) - R - x0) / cell)), ix1 = std::min(nx - 1, int((std::max(a.x, b.x) + R - x0) / cell) + 1);
        const int iz0 = std::max(0, int((std::min(a.z, b.z) - R - z0) / cell)), iz1 = std::min(nz - 1, int((std::max(a.z, b.z) + R - z0) / cell) + 1);
        for (int iz = iz0; iz <= iz1; ++iz) {
            for (int ix = ix0; ix <= ix1; ++ix) {
                const double px = x0 + ix * cell - a.x, pz = z0 + iz * cell - a.z;
                const double t = clampd(px * ux + pz * uz, 0, len);
                const double qx = px - ux * t, qz = pz - uz * t, d2 = qx * qx + qz * qz;
                const size_t k = size_t(iz) * nx + ix;
                if (d2 < best[k]) {
                    best[k] = float(d2);
                    sv[k] = float(a.s + t);
                    dv[k] = float(std::copysign(std::sqrt(d2), -uz * qx + ux * qz)); // côté droit = (-uz, ux)
                }
            }
        }
    }
    std::vector<float> h(best.size(), 0.0f);
    for (size_t k = 0; k < h.size(); ++k) {
        if (best[k] >= float(R * R)) continue;
        const double t = sv[k], d = dv[k];
        int side;
        const double bh = berm(t, side), u = d * side; // u > 0 : côté extérieur du virage
        // Berm : la piste se relève du centre jusqu'au bord extérieur, crête de 1 m, puis retombe.
        const double bermShape = u <= 0 ? 0 : (u < 5.6 ? smooth(s01(u, 0, 4.6)) : 1 - smooth(s01(u, 5.6, 8.6)));
        h[k] = float(profile(t) * (1 - smooth(s01(std::abs(d), kTrackHalfWidth, 7.0))) + bh * bermShape);
    }

    Terrain terrain(x0, z0, cell, nx, nz, std::move(h));
    terrain.path_ = std::move(path);
    auto at = [&](const char* name, double t) {
        const TrackPoint& p = terrain.path_[std::min(size_t(t / ds), terrain.path_.size() - 1)];
        terrain.spawns_.push_back({name, p.x, p.z, -p.heading});
    };
    at("Départ", 4);
    at("Double", 22);
    at("Table", 60);
    at("Berm", 100);
    at("Virage à plat", s2 + 30);
    at("Whoops", s3b - 6);
    return terrain;
}

Terrain Terrain::makeFlat() {
    const int nx = 1001, nz = 201; // 500 m x 100 m, mailles de 50 cm
    Terrain t(-50, -50, 0.5, nx, nz, std::vector<float>(size_t(nx) * nz, 0.0f));
    t.spawns_.push_back({"Départ", 0, 0, 0});
    return t;
}

} // namespace mx
