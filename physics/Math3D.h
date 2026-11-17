#pragma once
// Maths 3D minimales : x vers l'avant, y vers le haut, z vers la droite (repère direct).
// Rotation positive autour de +z = cabrage, autour de +x = inclinaison à droite,
// autour de +y = virage à gauche. Pour Unreal, on remplacera Vec3/Quat/RigidBody3D ;
// le reste du code n'utilise que cette API.
#include <cmath>

namespace mx {

struct Vec3 {
    double x = 0, y = 0, z = 0;
    Vec3 operator+(Vec3 o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(Vec3 o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(Vec3 o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(Vec3 o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
};
inline Vec3 operator*(double s, Vec3 v) { return v * s; }
inline double dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline double length(Vec3 a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalize(Vec3 a) { const double l = length(a); return l > 1e-12 ? a / l : Vec3{}; }
inline Vec3 mul(Vec3 a, Vec3 b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; } // composante par composante
inline Vec3 div(Vec3 a, Vec3 b) { return {a.x / b.x, a.y / b.y, a.z / b.z}; }
inline double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

constexpr Vec3 kX{1, 0, 0}, kY{0, 1, 0}, kZ{0, 0, 1};

// Quaternion unitaire (w, x, y, z) : orientation d'un corps (repère local -> monde).
struct Quat {
    double w = 1, x = 0, y = 0, z = 0;
};
inline Quat operator*(Quat a, Quat b) {
    return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
inline Quat conj(Quat q) { return {q.w, -q.x, -q.y, -q.z}; }
inline Quat normalize(Quat q) {
    const double l = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    return {q.w / l, q.x / l, q.y / l, q.z / l};
}
inline Quat axisAngle(Vec3 axis, double ang) {
    const Vec3 a = normalize(axis) * std::sin(ang / 2);
    return {std::cos(ang / 2), a.x, a.y, a.z};
}
inline Vec3 rotate(Quat q, Vec3 v) { // q v q*
    const Vec3 u{q.x, q.y, q.z};
    const Vec3 t = cross(u, v) * 2;
    return v + t * q.w + cross(u, t);
}
inline Vec3 rotateInv(Quat q, Vec3 v) { return rotate(conj(q), v); }
// Petit vecteur rotation (axe * angle) d'un quaternion, chemin le plus court.
inline Vec3 rotationVector(Quat q) {
    if (q.w < 0) q = {-q.w, -q.x, -q.y, -q.z};
    const Vec3 v{q.x, q.y, q.z};
    const double s = length(v);
    return s < 1e-12 ? v * 2 : v * (2 * std::atan2(s, q.w) / s);
}

// Corps rigide. pos = centre de masse ; angVel en repère monde ; inertie diagonale dans le
// repère du corps (axes principaux supposés alignés sur x/y/z du corps).
// Intégration Euler semi-implicite : vitesses d'abord (avec le terme gyroscopique ω x Iω),
// puis positions avec les nouvelles vitesses.
struct RigidBody3D {
    Vec3 pos, vel, angVel;
    Quat q;
    double mass = 1;
    Vec3 inertia{1, 1, 1}; // kg.m² autour de x (roulis), y (lacet), z (tangage) du corps
    Vec3 force, torque;

    Vec3 toWorld(Vec3 local) const { return pos + rotate(q, local); }
    Vec3 dirToWorld(Vec3 d) const { return rotate(q, d); }
    Vec3 velocityAt(Vec3 world) const { return vel + cross(angVel, world - pos); }
    void applyForce(Vec3 f, Vec3 worldPoint) {
        force += f;
        torque += cross(worldPoint - pos, f);
    }
    Vec3 inertiaMul(Vec3 w) const { return rotate(q, mul(inertia, rotateInv(q, w))); }    // I_monde * w
    Vec3 invInertiaMul(Vec3 w) const { return rotate(q, div(rotateInv(q, w), inertia)); } // I_monde^-1 * w

    void integrateVelocity(double dt) {
        vel += force * (dt / mass);
        angVel += invInertiaMul(torque - cross(angVel, inertiaMul(angVel))) * dt;
    }
    void integratePosition(double dt) {
        pos += vel * dt;
        const Quat w{0, angVel.x, angVel.y, angVel.z};
        const Quat dq = w * q;
        q = normalize({q.w + 0.5 * dt * dq.w, q.x + 0.5 * dt * dq.x, q.y + 0.5 * dt * dq.y, q.z + 0.5 * dt * dq.z});
    }
};

} // namespace mx
