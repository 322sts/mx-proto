#pragma once
// Profil de hauteur vu de côté y = h(x), échantillonné tous les 5 cm et interpolé par
// spline de Catmull-Rom : hauteur ET pente continues, donc aucune "marche" sous les pneus.
#include "Math2D.h"
#include <vector>

namespace mx {

class Terrain {
public:
    Terrain(double x0, double dx, std::vector<double> heights);

    double height(double x) const;
    double slope(double x) const; // dh/dx
    Vec2 normal(double x) const;  // unitaire, sortant du sol
    // Point du profil le plus proche de p, cherché dans |x - p.x| <= radius.
    Vec2 closestPoint(Vec2 p, double radius) const;

    double xMin() const { return x0_; }
    double xMax() const { return x0_ + dx_ * double(h_.size() - 1); }

    // Piste de test : élan, petit saut, grosse table, whoops, montée raide, descente.
    static Terrain makeTestTrack();
    // Positions de remise à zéro sur la piste (m).
    static constexpr double kStartX = 5.0, kJump1X = 30.0, kJump2X = 82.0, kWhoopsX = 145.0;

private:
    double x0_, dx_;
    std::vector<double> h_;
    void segment(double x, int& i, double& t) const;
};

} // namespace mx
