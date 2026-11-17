#pragma once
// Sol = carte de hauteurs y = h(x, z) sur une grille régulière, interpolée en bicubique
// Catmull-Rom : hauteur ET pente continues dans les deux directions, donc aucune "marche"
// sous les pneus.
#include "Math3D.h"
#include <string>
#include <vector>

namespace mx {

// Point de la ligne médiane du circuit (pour le décor : piquets, bottes de paille).
struct TrackPoint {
    double x, z;    // m
    double heading; // rad : direction (cos, 0, sin) ; + = vers la droite (+z)
    double s;       // m   abscisse le long du circuit
    double curvature; // 1/m (+ = virage à droite)
};

// Point de départ nommé : position au sol et cap (lacet autour de +y, + = tourné vers la gauche).
struct Spawn {
    std::string name;
    double x, z, yaw;
};

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

    const std::vector<TrackPoint>& trackPath() const { return path_; }
    const std::vector<Spawn>& spawns() const { return spawns_; }
    const Spawn& spawn(const std::string& name) const;
    static constexpr double kTrackHalfWidth = 4.0; // m, largeur de piste roulable = 8 m

    // Circuit de test en boucle (~420 m) : double, table, virages relevés (berms) de rayons
    // différents, virage à plat, chicane, rouleaux, whoops, au milieu d'un grand pré plat.
    static Terrain makeTestTrack();
    static Terrain makeFlat();

private:
    double x0_, z0_, cell_;
    int nx_, nz_;
    std::vector<float> h_; // h_[iz * nx_ + ix]
    std::vector<TrackPoint> path_;
    std::vector<Spawn> spawns_;
    float at(int ix, int iz) const;
};

} // namespace mx
