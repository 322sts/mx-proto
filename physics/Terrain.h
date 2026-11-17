#pragma once
// Sol = carte de hauteurs y = h(x, z) sur une grille régulière, interpolée en bicubique
// Catmull-Rom : hauteur ET pente continues dans les deux directions, donc aucune "marche"
// sous les pneus.
#include "Math3D.h"
#include <vector>

namespace mx {

class Terrain {
public:
    Terrain(double x0, double z0, double cell, int nx, int nz, std::vector<float> heights);

    double height(double x, double z) const;
    // Hauteur et pentes dh/dx, dh/dz en une seule évaluation.
    double sample(double x, double z, double& dhdx, double& dhdz) const;
    Vec3 normal(double x, double z) const; // unitaire, sortant du sol

    double xMin() const { return x0_; }
    double xMax() const { return x0_ + cell_ * (nx_ - 1); }
    double zMin() const { return z0_; }
    double zMax() const { return z0_ + cell_ * (nz_ - 1); }

    // Piste de test : bande de 10 m de large le long de x (élan, petit saut, grosse table,
    // whoops, montée, descente) au milieu d'une grande zone plate pour tourner.
    static Terrain makeTestTrack();
    static Terrain makeFlat();
    // Positions de remise à zéro sur la piste (m, en x ; z = 0, cap vers +x).
    static constexpr double kStartX = 5.0, kJump1X = 30.0, kJump2X = 82.0, kWhoopsX = 145.0;

private:
    double x0_, z0_, cell_;
    int nx_, nz_;
    std::vector<float> h_; // h_[iz * nx_ + ix]
    float at(int ix, int iz) const;
};

} // namespace mx
