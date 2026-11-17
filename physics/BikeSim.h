#pragma once
// Simulation de la moto + pilote dans le plan vertical.
//
// Corps rigides (Euler semi-implicite, pas fixe) :
//   châssis, fourche (fourreaux + masse de la roue AV), bras oscillant (+ masse de la
//   roue AR), pilote. La rotation propre de chaque roue est un degré de liberté à part
//   (roue articulée sur son axe), ce qui fait que gaz/frein en l'air font tourner la moto.
// Liaisons rigides (solveur d'impulsions en vitesse) : glissière de fourche, pivot du bras,
// butées de fin de course de sécurité. Tout le reste est fait de forces : ressorts,
// amortisseurs, pneus, pilote, moteur, freins.
#include "BikeParams.h"
#include "Terrain.h"

namespace mx {

struct WheelState {
    Vec2 center;             // m
    double radius = 0;       // m
    double angle = 0;        // rad    rotation absolue (pour le dessin)
    double spin = 0;         // rad/s  vitesse angulaire absolue (sens trigo : rouler en avant < 0)
    bool contact = false;
    Vec2 contactPoint, normal;
    double deflection = 0;   // m      écrasement du pneu
    double normalForce = 0;  // N
    double longForce = 0;    // N      positif = pousse la moto vers l'avant
    double slipRatio = 0;    // -      positif = patine (motricité), négatif = freine/bloque
};

struct BikeState {
    double time = 0;                      // s
    RigidBody2D chassis, fork, swingarm, rider;
    WheelState front, rear;
    double forkTravel = 0, forkSpeed = 0; // m, m/s le long de la fourche (positif = compression)
    double rearTravel = 0, rearSpeed = 0; // m, m/s à la roue AR
    double shockStroke = 0;               // m      course de l'amortisseur
    double engineSpeed = 0;               // rad/s  vilebrequin
    double speed = 0;                     // m/s    vitesse du châssis
    double riderLean = 0;                 // m      déplacement actuel du corps du pilote
    Inputs inputs;                        // commandes du dernier pas
};

// Points de conception, chacun dans le repère local du corps concerné (origine = son CdG).
struct BikeLayout {
    // Châssis
    Vec2 rearAxle, pivot, frontAxle, steeringHead, pegs, bars, shockTop, forkAxis;
    Vec2 skidPlate, rearFender, seat;
    // Fourche et bras
    Vec2 forkAxle, swingPivot, swingAxle;
    // Pilote
    Vec2 riderFeet, riderHands, riderHip, riderShoulder, riderHead;
};

class BikeSim {
public:
    // Construit la moto à l'abscisse x de la piste et la laisse se poser (freins serrés).
    void reset(const BikeParams& params, const Terrain& terrain, double x);
    void step(const Inputs& inputs, const Terrain& terrain, double dt);

    BikeParams& params() { return p_; } // modifiable en roulant (sauf champs needsReset)
    const BikeState& state() const { return s_; }
    const BikeLayout& layout() const { return L_; }

private:
    void tire(WheelState& w, RigidBody2D& carrier, Vec2 axle, double stiffness, double inertia,
              const Terrain& terrain, double dt, double& wheelTorque);
    void solveJoints(double dt);

    BikeParams p_;
    BikeState s_;
    BikeLayout L_;
};

} // namespace mx
