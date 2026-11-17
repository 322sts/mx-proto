#pragma once
// Maths 2D minimales du prototype vu de côté (x vers l'avant, y vers le haut,
// angles en rad, positifs dans le sens trigo = cabrage).
// Pour la 3D, on remplacera Vec2/RigidBody2D ; le reste du code n'utilise que cette API.
#include <cmath>

namespace mx {

struct Vec2 {
    double x = 0, y = 0;
    Vec2 operator+(Vec2 o) const { return {x + o.x, y + o.y}; }
    Vec2 operator-(Vec2 o) const { return {x - o.x, y - o.y}; }
    Vec2 operator-() const { return {-x, -y}; }
    Vec2 operator*(double s) const { return {x * s, y * s}; }
    Vec2 operator/(double s) const { return {x / s, y / s}; }
    Vec2& operator+=(Vec2 o) { x += o.x; y += o.y; return *this; }
    Vec2& operator-=(Vec2 o) { x -= o.x; y -= o.y; return *this; }
};
inline Vec2 operator*(double s, Vec2 v) { return v * s; }
inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; } // composante z de a x b
inline Vec2 perp(Vec2 a) { return {-a.y, a.x}; }                      // z x a (rotation de +90°)
inline double length(Vec2 a) { return std::sqrt(dot(a, a)); }
inline Vec2 rotate(Vec2 v, double ang) {
    const double c = std::cos(ang), s = std::sin(ang);
    return {c * v.x - s * v.y, s * v.x + c * v.y};
}
inline double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Corps rigide plan. pos = centre de masse ; forces/couples accumulés à chaque pas puis
// intégrés (Euler semi-implicite : vitesse d'abord, puis position avec la nouvelle vitesse).
struct RigidBody2D {
    Vec2 pos, vel;
    double angle = 0, angVel = 0;
    double mass = 1, inertia = 1; // kg, kg.m² autour du centre de masse
    Vec2 force;
    double torque = 0;

    Vec2 toWorld(Vec2 local) const { return pos + rotate(local, angle); }
    Vec2 velocityAt(Vec2 world) const { return vel + angVel * perp(world - pos); }
    void applyForce(Vec2 f, Vec2 worldPoint) {
        force += f;
        torque += cross(worldPoint - pos, f);
    }
};

} // namespace mx
