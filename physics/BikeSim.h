#pragma once
// Simulation 3D de la moto + pilote.
//
// Corps rigides (Euler semi-implicite, pas fixe) :
//   châssis, bloc avant (fourreaux + masse de la roue AV), bras oscillant (+ masse de la
//   roue AR), pilote. La rotation propre de chaque roue est un degré de liberté à part
//   (roue articulée sur son axe) : gaz/frein en l'air font tourner la moto, et son moment
//   cinétique donne l'effet gyroscopique au porteur.
// Liaisons rigides (solveur d'impulsions en vitesse) :
//   - direction + fourche = liaison cylindrique châssis/bloc avant le long de l'axe de direction
//     (rotation = braquage, translation = enfoncement : exact pour des tubes parallèles à l'axe) ;
//   - pivot du bras = charnière ;
//   - butées de fin de course et de braquage.
// Tout le reste est fait de forces : ressorts, amortisseurs, pneus, pilote, moteur, chaîne, freins.
#include "BikeParams.h"
#include "Terrain.h"

namespace mx {

struct WheelState {
    Vec3 center;             // m
    double radius = 0;       // m
    double angle = 0;        // rad    rotation par rapport au porteur (pour le dessin)
    double spin = 0;         // rad/s  vitesse angulaire absolue autour de l'axe de roue (+z porteur ; rouler en avant < 0)
    bool contact = false;
    Vec3 contactPoint, normal;
    double deflection = 0;   // m      écrasement du pneu
    double normalForce = 0;  // N
    double longForce = 0;    // N      positif = pousse vers l'avant de la roue
    double latForce = 0;     // N      positif = pousse vers la droite de la roue
    double slipRatio = 0;    // -      positif = patine (motricité), négatif = freine/bloque
    double slipAngle = 0;    // rad    dérive (positif = la roue glisse vers la gauche)
    double camber = 0;       // rad    inclinaison de la roue par rapport au sol (positif = vers la droite)
};

struct BikeState {
    double time = 0;                      // s
    RigidBody3D chassis, fork, swingarm, rider;
    WheelState front, rear;
    double forkTravel = 0, forkSpeed = 0; // m, m/s le long de la fourche (positif = compression)
    double rearTravel = 0, rearSpeed = 0; // m, m/s à la roue AR
    double shockStroke = 0;               // m      course de l'amortisseur
    double engineSpeed = 0;               // rad/s  vilebrequin
    double chainTension = 0;              // N      brin supérieur
    double speed = 0;                     // m/s    vitesse du châssis
    double riderLean = 0;                 // m      déplacement avant/arrière actuel du corps du pilote
    double steerAngle = 0;                // rad    braquage (positif = à gauche)
    double leanAngle = 0;                 // rad    inclinaison de la moto (positif = à droite)
    double steerTorque = 0;               // N.m    couple du pilote dans le guidon
    double feetDown = 0;                  // 0..1   pieds posés (basse vitesse)
    Inputs inputs;                        // commandes du dernier pas
};

// Points de conception, chacun dans le repère local du corps concerné (origine = son CdG).
// Plan de symétrie z = 0 ; les points pairs (repose-pieds, guidon) sont décalés de ±largeur en z.
struct BikeLayout {
    // Châssis
    Vec3 rearAxle, pivot, frontAxle, steerAxisPoint, steeringHead, pegs, bars, shockTop, forkAxis;
    Vec3 skidPlate, rearFender, seat, countershaft, foot;
    // Bloc avant et bras
    Vec3 forkAxle, forkAxisPoint, swingPivot, swingAxle;
    // Pilote
    Vec3 riderFeet, riderHands, riderHip, riderShoulder, riderHead;
};

class BikeSim {
public:
    // Construit la moto au point (x, z), cap yaw (lacet autour de +y, 0 = vers +x, + = vers la gauche),
    // et la laisse se poser (freins serrés).
    void reset(const BikeParams& params, const Terrain& terrain, double x, double z = 0, double yaw = 0);
    void reset(const BikeParams& params, const Terrain& terrain, const Spawn& at) { reset(params, terrain, at.x, at.z, at.yaw); }
    void step(const Inputs& inputs, const Terrain& terrain, double dt);

    BikeParams& params() { return p_; } // modifiable en roulant (sauf champs needsReset)
    const BikeState& state() const { return s_; }
    const BikeLayout& layout() const { return L_; }

private:
    void tire(WheelState& w, RigidBody3D& carrier, Vec3 axle, Vec3 axis, double stiffness, double spinInertia,
              const Terrain& terrain, double dt, double& spinTorque);
    void bodyContact(RigidBody3D& b, Vec3 local, const Terrain& terrain, double stiffness, double damping, double dt);
    double steerAngle() const;  // rad, rotation du bloc avant autour de l'axe de direction (+ = gauche)
    double swingAngle() const;  // rad, rotation du bras autour de son pivot (+ = détente au-delà du dessin)
    void solveJoints(double dt);

    BikeParams p_;
    BikeState s_;
    BikeLayout L_;
};

} // namespace mx
