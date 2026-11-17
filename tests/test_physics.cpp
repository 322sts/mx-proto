#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "BikeSim.h"
#include <algorithm>
#include <cmath>

using namespace mx;

namespace {
const Terrain& track() {
    static const Terrain t = Terrain::makeTestTrack();
    return t;
}
const Terrain& flat() {
    static const Terrain t = Terrain::makeFlat();
    return t;
}

bool finite(const BikeState& s) {
    for (const RigidBody3D* b : {&s.chassis, &s.fork, &s.swingarm, &s.rider})
        for (double v : {b->pos.x, b->pos.y, b->pos.z, b->vel.x, b->vel.y, b->vel.z, b->angVel.x, b->angVel.y, b->angVel.z, b->q.w})
            if (!std::isfinite(v)) return false;
    return std::isfinite(s.front.spin) && std::isfinite(s.rear.spin);
}

// Roule `seconds` avec les commandes données ; vérifie à chaque pas que rien n'explose
// et qu'aucune roue ne passe sous le sol.
void ride(BikeSim& sim, const Terrain& t, Inputs in, double seconds) {
    for (int i = 0; i < int(seconds * 1000); ++i) {
        sim.step(in, t, 0.001);
        const BikeState& s = sim.state();
        REQUIRE(finite(s));
        REQUIRE(s.speed < 60.0);
        for (const WheelState* w : {&s.front, &s.rear})
            REQUIRE(w->center.y > t.height(w->center.x, w->center.z));
    }
}

// Lance la moto en ligne droite sur le plat jusqu'à la vitesse v (pilote sur l'avant, gaz à mi-course).
void launch(BikeSim& sim, double v) {
    sim.reset(BikeParams{}, flat(), 0);
    Inputs in;
    in.throttle = 0.5;
    in.riderLean = 1;
    while (sim.state().speed < v && sim.state().time < 15) sim.step(in, flat(), 0.001);
}
} // namespace

TEST_CASE("circuit : la boucle se referme et le terrain est lisse (pas de marche en hauteur ni en pente)") {
    const Terrain& t = track();
    const TrackPoint &first = t.trackPath().front(), &last = t.trackPath().back();
    CHECK(std::hypot(last.x - first.x, last.z - first.z) < 0.2); // dernier point à un pas du départ
    double maxDh = 0, maxDs = 0;
    auto scan = [&](double x0, double z0, double dx, double dz, int n) {
        double hx0, hz0;
        t.sample(x0, z0, hx0, hz0);
        double hPrev = t.height(x0, z0);
        for (int i = 1; i < n; ++i) {
            double hx, hz;
            const double h = t.sample(x0 + i * dx, z0 + i * dz, hx, hz);
            maxDh = std::max(maxDh, std::abs(h - hPrev));
            maxDs = std::max(maxDs, std::max(std::abs(hx - hx0), std::abs(hz - hz0)));
            hPrev = h;
            hx0 = hx;
            hz0 = hz;
        }
    };
    // Lignes en long et en travers du circuit : départ/double/table (z ~ 0), whoops (z ~ 77), berms et virages.
    for (double z : {0.0, 3.0, 6.5, 74.0, 77.0, 12.0}) scan(t.xMin(), z, 0.001, 0, int((t.xMax() - t.xMin()) / 0.001));
    for (double x : {40.0, 125.0, 133.0, -25.0, -30.0, 0.0}) scan(x, t.zMin(), 0, 0.001, int((t.zMax() - t.zMin()) / 0.001));
    CHECK(maxDh < 0.002); // pente < 63°
    CHECK(maxDs < 0.05);  // pente continue
}

TEST_CASE("au repos : la moto se pose sur son sag, tient debout et ne vibre pas") {
    BikeSim sim;
    sim.reset(BikeParams{}, track(), track().spawn("Départ"));
    ride(sim, track(), Inputs{}, 2.0);
    double maxV = 0, maxW = 0;
    for (int i = 0; i < 1000; ++i) {
        sim.step(Inputs{}, track(), 0.001);
        maxV = std::max(maxV, length(sim.state().chassis.vel));
        maxW = std::max(maxW, length(sim.state().chassis.angVel));
    }
    const BikeState& s = sim.state();
    MESSAGE("sag fourche " << s.forkTravel * 1000 << " mm, sag AR " << s.rearTravel * 1000 << " mm, charge AV "
                           << s.front.normalForce << " N, AR " << s.rear.normalForce << " N");
    CHECK(maxV < 0.005);
    CHECK(maxW < 0.005);
    CHECK(std::abs(s.leanAngle) < 0.01);
    CHECK(s.forkTravel > 0.03);
    CHECK(s.forkTravel < 0.12);
    CHECK(s.rearTravel > 0.06);
    CHECK(s.rearTravel < 0.14);
    CHECK(std::abs(s.front.normalForce + s.rear.normalForce - 9.81 * 181) < 30);
}

TEST_CASE("gaz à fond, pilote sur l'avant : la moto accélère droit et traverse la piste sans exploser") {
    // (Gaz à fond pilote au neutre = looping arrière, comme une vraie 450 : c'est voulu.)
    BikeSim sim;
    sim.reset(BikeParams{}, track(), track().spawn("Départ"));
    Inputs in;
    in.throttle = 1;
    in.riderLean = 1;
    ride(sim, track(), in, 2.0);
    MESSAGE("vitesse après 2 s : " << sim.state().speed * 3.6 << " km/h");
    CHECK(sim.state().speed > 8.0);
    in.throttle = 0.6;
    ride(sim, track(), in, 20.0);
}

TEST_CASE("ligne droite sur le plat : la moto reste droite et dans l'axe") {
    BikeSim sim;
    sim.reset(BikeParams{}, flat(), 0);
    Inputs in;
    in.throttle = 0.6;
    in.riderLean = 0.5;
    ride(sim, flat(), in, 10.0);
    MESSAGE("après 10 s : " << sim.state().speed * 3.6 << " km/h, écart latéral " << sim.state().chassis.pos.z << " m");
    CHECK(std::abs(sim.state().chassis.pos.z) < 0.1);
    CHECK(std::abs(sim.state().leanAngle) < 0.02);
}

TEST_CASE("chaîne : l'arrière ne s'écrase pas et ne se détend pas en butée sous les gaz") {
    BikeSim sim;
    sim.reset(BikeParams{}, flat(), 0);
    for (int i = 0; i < 1000; ++i) sim.step(Inputs{}, flat(), 0.001);
    const double sag = sim.state().rearTravel;
    Inputs in;
    in.throttle = 0.6;
    double lo = sag, hi = sag;
    for (int i = 0; i < 3000; ++i) {
        sim.step(in, flat(), 0.001);
        lo = std::min(lo, sim.state().rearTravel);
        hi = std::max(hi, sim.state().rearTravel);
    }
    MESSAGE("sag AR " << sag * 1000 << " mm, sous les gaz " << lo * 1000 << ".." << hi * 1000 << " mm");
    CHECK(hi - sag < 0.03);
    CHECK(sag - lo < 0.03);
    CHECK(sim.state().chainTension > 0);
}

TEST_CASE("gaz en l'air : la roue AR accélère et la moto cabre") {
    BikeSim sim;
    sim.reset(BikeParams{}, track(), track().spawn("Départ"));
    // Moto lancée en l'air : on la soulève de 3 m.
    BikeState& s = const_cast<BikeState&>(sim.state());
    for (RigidBody3D* b : {&s.chassis, &s.fork, &s.swingarm, &s.rider}) b->pos.y += 3;
    Inputs in;
    in.throttle = 1;
    for (int i = 0; i < 300; ++i) sim.step(in, track(), 0.001);
    CHECK(sim.state().chassis.angVel.z > 0.1); // nez qui monte
}

TEST_CASE("virage établi : inclinaison et rayon obéissent à tan(incl) = v²/(g R)") {
    BikeSim sim;
    launch(sim, 8.0);
    Inputs in;
    in.steer = 0.5;
    in.riderLean = 0.3;
    for (int i = 0; i < 5000; ++i) {
        in.throttle = clampd(0.3 + 0.3 * (8.0 - sim.state().speed), 0, 1); // vitesse tenue
        sim.step(in, flat(), 0.001);
        REQUIRE(finite(sim.state()));
    }
    const BikeState& s = sim.state();
    const double yawRate = s.chassis.angVel.y, radius = s.speed / std::abs(yawRate);
    const double leanPhysics = std::atan(s.speed * s.speed / (9.81 * radius));
    MESSAGE("inclinaison " << s.leanAngle * 57.3 << "° (visée " << 0.5 * BikeParams{}.riderMaxLean * 57.3 << "°), rayon "
                           << radius << " m, inclinaison théorique " << leanPhysics * 57.3 << "°");
    CHECK(s.leanAngle > 0.1);  // penchée à droite
    CHECK(yawRate < -0.1);     // tourne à droite
    CHECK(s.steerAngle < 0);   // roue braquée vers l'intérieur
    CHECK(std::abs(s.leanAngle - leanPhysics) < 0.06); // à la largeur du pneu près (~3°)
}

TEST_CASE("contre-braquage : pour pencher à droite, le guidon part d'abord à gauche (rien de programmé)") {
    BikeSim sim;
    launch(sim, 12.0);
    Inputs in;
    in.steer = 1;
    in.throttle = 0.4;
    double firstSteer = 0;
    for (int i = 0; i < 1500; ++i) {
        sim.step(in, flat(), 0.001);
        if (firstSteer == 0 && std::abs(sim.state().steerAngle) > 0.01) firstSteer = sim.state().steerAngle;
    }
    MESSAGE("premier braquage " << firstSteer * 57.3 << "°, inclinaison après 1,5 s " << sim.state().leanAngle * 57.3 << "°");
    CHECK(firstSteer > 0);                // à gauche d'abord
    CHECK(sim.state().leanAngle > 0.3);   // puis la moto penche à droite
}

TEST_CASE("réglages extrêmes : chaque curseur à ses bornes reste stable") {
    for (const ParamInfo& info : paramTable()) {
        for (double v : {info.min, info.max}) {
            CAPTURE(info.key);
            CAPTURE(v);
            BikeParams p;
            p.*info.field = v;
            BikeSim sim;
            sim.reset(p, track(), track().spawn("Double"));
            Inputs in;
            in.throttle = 1;
            in.riderLean = 0.5;
            ride(sim, track(), in, 3.0); // passe le petit saut
            in.throttle = 0;
            in.frontBrake = 1;
            in.rearBrake = 1;
            in.steer = 0.5; // et tourne en freinant
            ride(sim, track(), in, 1.0);
        }
    }
}
