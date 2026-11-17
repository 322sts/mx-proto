#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include "BikeSim.h"
#include <cmath>

using namespace mx;

namespace {
const Terrain& track() {
    static const Terrain t = Terrain::makeTestTrack();
    return t;
}

bool finite(const BikeState& s) {
    for (const RigidBody2D* b : {&s.chassis, &s.fork, &s.swingarm, &s.rider})
        if (!std::isfinite(b->pos.x) || !std::isfinite(b->pos.y) || !std::isfinite(b->angle) ||
            !std::isfinite(b->vel.x) || !std::isfinite(b->vel.y) || !std::isfinite(b->angVel))
            return false;
    return std::isfinite(s.front.spin) && std::isfinite(s.rear.spin);
}

// Roule `seconds` avec les commandes données ; vérifie à chaque pas que rien n'explose
// et qu'aucune roue ne passe sous le sol.
void ride(BikeSim& sim, Inputs in, double seconds) {
    for (int i = 0; i < int(seconds * 1000); ++i) {
        sim.step(in, track(), 0.001);
        const BikeState& s = sim.state();
        REQUIRE(finite(s));
        REQUIRE(s.speed < 60.0);
        for (const WheelState* w : {&s.front, &s.rear})
            REQUIRE(w->center.y > track().height(w->center.x));
    }
}
} // namespace

TEST_CASE("le terrain est lisse (pas de marche en hauteur ni en pente)") {
    const Terrain& t = track();
    double maxDh = 0, maxDs = 0;
    for (double x = t.xMin(); x < t.xMax() - 0.001; x += 0.001) {
        maxDh = std::max(maxDh, std::abs(t.height(x + 0.001) - t.height(x)));
        maxDs = std::max(maxDs, std::abs(t.slope(x + 0.001) - t.slope(x)));
    }
    CHECK(maxDh < 0.002); // pente < 63°
    CHECK(maxDs < 0.05);  // pente continue
}

TEST_CASE("au repos : la moto se pose sur son sag et ne vibre pas") {
    BikeSim sim;
    sim.reset(BikeParams{}, track(), Terrain::kStartX);
    ride(sim, Inputs{}, 2.0);
    double maxV = 0, maxW = 0;
    for (int i = 0; i < 1000; ++i) {
        sim.step(Inputs{}, track(), 0.001);
        maxV = std::max(maxV, length(sim.state().chassis.vel));
        maxW = std::max(maxW, std::abs(sim.state().chassis.angVel));
    }
    const BikeState& s = sim.state();
    MESSAGE("sag fourche " << s.forkTravel * 1000 << " mm, sag AR " << s.rearTravel * 1000 << " mm, charge AV "
                           << s.front.normalForce << " N, AR " << s.rear.normalForce << " N");
    CHECK(maxV < 0.005);
    CHECK(maxW < 0.005);
    CHECK(s.forkTravel > 0.03);
    CHECK(s.forkTravel < 0.12);
    CHECK(s.rearTravel > 0.06);
    CHECK(s.rearTravel < 0.14);
    CHECK(std::abs(s.front.normalForce + s.rear.normalForce - 9.81 * 181) < 30);
}

TEST_CASE("gaz à fond, pilote sur l'avant : la moto accélère et traverse la piste sans exploser") {
    // (Gaz à fond pilote au neutre = looping arrière, comme une vraie 450 : c'est voulu.)
    BikeSim sim;
    sim.reset(BikeParams{}, track(), Terrain::kStartX);
    Inputs in;
    in.throttle = 1;
    in.riderLean = 1;
    ride(sim, in, 2.0);
    MESSAGE("vitesse après 2 s : " << sim.state().speed * 3.6 << " km/h");
    CHECK(sim.state().speed > 8.0);
    in.throttle = 0.6;
    ride(sim, in, 20.0);
}

TEST_CASE("gaz en l'air : la roue AR accélère et la moto cabre") {
    BikeSim sim;
    sim.reset(BikeParams{}, track(), Terrain::kStartX);
    // Moto lancée en l'air : on la soulève de 3 m.
    BikeState& s = const_cast<BikeState&>(sim.state());
    for (RigidBody2D* b : {&s.chassis, &s.fork, &s.swingarm, &s.rider}) b->pos.y += 3;
    Inputs in;
    in.throttle = 1;
    for (int i = 0; i < 300; ++i) sim.step(in, track(), 0.001);
    CHECK(sim.state().chassis.angVel > 0.1); // nez qui monte
}

TEST_CASE("réglages extrêmes : chaque curseur à ses bornes reste stable") {
    for (const ParamInfo& info : paramTable()) {
        for (double v : {info.min, info.max}) {
            CAPTURE(info.key);
            CAPTURE(v);
            BikeParams p;
            p.*info.field = v;
            BikeSim sim;
            sim.reset(p, track(), Terrain::kJump1X);
            Inputs in;
            in.throttle = 1;
            ride(sim, in, 3.0); // passe le petit saut
            in.throttle = 0;
            in.frontBrake = 1;
            in.rearBrake = 1;
            ride(sim, in, 1.0);
        }
    }
}
