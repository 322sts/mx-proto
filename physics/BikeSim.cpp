#include "BikeSim.h"
#include <algorithm>
#include <numbers>

namespace mx {

namespace {
constexpr double kGravity = 9.81;        // m/s²
constexpr double kAirDensity = 1.2;      // kg/m³
constexpr int kSolverIterations = 20;
constexpr double kBaumgarte = 0.2;       // fraction de l'erreur de position corrigée par pas
constexpr double kHardStopMargin = 0.03; // m au-delà du débattement : arrêt rigide de sécurité
constexpr double kBodyContactStiffness = 80000.0; // N/m   châssis/pilote qui touche le sol (chute)
constexpr double kBodyContactDamping = 3000.0;    // N.s/m
constexpr double kBodyContactMu = 0.6;

double sq(double v) { return v * v; }

// Courbe de couple moteur normalisée (fraction du couple max) selon le régime.
double torqueFraction(double omega) {
    static constexpr double rpm[] = {0, 1000, 4000, 8000, 10500, 12500, 20000};
    static constexpr double frac[] = {0.45, 0.50, 0.84, 1.00, 0.88, 0.70, 0.70};
    const double r = omega * 60.0 / (2 * std::numbers::pi);
    if (r <= 0) return frac[0];
    for (int i = 0; i + 1 < 7; ++i)
        if (r < rpm[i + 1]) return frac[i] + (frac[i + 1] - frac[i]) * (r - rpm[i]) / (rpm[i + 1] - rpm[i]);
    return frac[6];
}

// Adhérence longitudinale sur terre selon |taux de glissement| :
// montée parabolique jusqu'au pic (pente nulle au pic), puis décroissance exponentielle
// douce vers l'adhérence de glisse -> pas de décrochage brutal.
double gripCurve(double k, const BikeParams& p) {
    if (k < p.tirePeakSlip) {
        const double r = k / p.tirePeakSlip;
        return p.tirePeakMu * (2 * r - r * r);
    }
    return p.tireSlideMu + (p.tirePeakMu - p.tireSlideMu) * std::exp(-(k - p.tirePeakSlip) / p.tireSlipFalloff);
}

// Ressort-amortisseur de longueur à vide nulle entre le point pa de a et le point pb de b.
void spring(RigidBody2D& a, Vec2 pa, RigidBody2D& b, Vec2 pb, double k, double c) {
    const Vec2 f = (pa - pb) * k - (b.velocityAt(pb) - a.velocityAt(pa)) * c; // force sur b
    b.applyForce(f, pb);
    a.applyForce(-f, pa);
}
} // namespace

void BikeSim::reset(const BikeParams& p, const Terrain& terrain, double x) {
    p_ = p;
    s_ = BikeState{};
    const double Rf = p.frontWheelRadius, Rr = p.rearWheelRadius;

    // Points de conception : moto détendue, horizontale, origine à l'axe de roue AR.
    const Vec2 rearAxle{0, 0};
    const Vec2 pivot = Vec2{std::cos(p.swingarmAngle), std::sin(p.swingarmAngle)} * p.swingarmLength;
    const Vec2 frontAxle{p.wheelbase, Rf - Rr};
    const Vec2 axis{-std::sin(p.rakeAngle), std::cos(p.rakeAngle)}; // de l'axe de roue vers la tête de fourche
    const Vec2 com{p.chassisComX, p.chassisComY};
    const Vec2 riderCom{p.riderComX, p.riderComY};
    const Vec2 pegs{p.pegX, p.pegY}, bars{p.barX, p.barY};

    // Fourche = fourreaux (CdG ~25 cm au-dessus de l'axe, ~50 cm de long) + roue AV.
    const double mF = p.forkLowerMass + p.frontWheelMass;
    const Vec2 lowersCom = frontAxle + axis * 0.25;
    const Vec2 forkCom = (lowersCom * p.forkLowerMass + frontAxle * p.frontWheelMass) / mF;
    const double IF = p.forkLowerMass * (0.25 / 12 + sq(length(lowersCom - forkCom))) +
                      p.frontWheelMass * sq(length(frontAxle - forkCom));
    // Bras = barre uniforme + roue AR à son extrémité (inertie de rotation propre de la roue à part).
    const double mS = p.swingarmMass + p.rearWheelMass;
    const Vec2 mid = (pivot + rearAxle) * 0.5;
    const Vec2 swingCom = (mid * p.swingarmMass + rearAxle * p.rearWheelMass) / mS;
    const double IS = p.swingarmMass * (sq(p.swingarmLength) / 12 + sq(length(mid - swingCom))) +
                      p.rearWheelMass * sq(length(rearAxle - swingCom));

    L_.rearAxle = rearAxle - com;
    L_.pivot = pivot - com;
    L_.frontAxle = frontAxle - com;
    L_.steeringHead = frontAxle + axis * p.forkLength - com;
    L_.pegs = pegs - com;
    L_.bars = bars - com;
    L_.forkAxis = axis;
    L_.shockTop = pivot + Vec2{0.05, 0.30} - com;          // dessin seulement
    L_.skidPlate = Vec2{pivot.x + 0.20, -0.05} - com;      // sabot moteur
    L_.rearFender = Vec2{-0.35, 0.62} - com;               // bout de garde-boue AR
    L_.seat = Vec2{0.40, 0.62} - com;
    L_.forkAxle = frontAxle - forkCom;
    L_.swingPivot = pivot - swingCom;
    L_.swingAxle = rearAxle - swingCom;
    L_.riderFeet = pegs - riderCom;
    L_.riderHands = bars - riderCom;
    L_.riderHip = {-0.10, -0.08};
    L_.riderShoulder = {0.12, 0.40};
    L_.riderHead = {0.18, 0.58};

    // Placement : axe AR 1 cm au-dessus du sol, moto horizontale, tout à l'arrêt.
    const Vec2 origin{x, terrain.height(x) + Rr + 0.01};
    auto place = [&](RigidBody2D& b, Vec2 designCom, double m, double I) {
        b = RigidBody2D{};
        b.pos = origin + designCom;
        b.mass = m;
        b.inertia = I;
    };
    place(s_.chassis, com, p.chassisMass, p.chassisInertia);
    place(s_.fork, forkCom, mF, IF);
    place(s_.swingarm, swingCom, mS, IS);
    place(s_.rider, riderCom, p.riderMass, p.riderInertia);
    s_.front.radius = Rf;
    s_.rear.radius = Rr;

    // 1,5 s freins serrés : la moto se pose, les suspensions trouvent leur sag.
    Inputs hold;
    hold.frontBrake = hold.rearBrake = 1;
    for (int i = 0; i < 1500; ++i) step(hold, terrain, 0.001);
    s_.time = 0;
}

void BikeSim::tire(WheelState& w, RigidBody2D& carrier, Vec2 axle, double stiffness, double inertia,
                   const Terrain& terrain, double dt, double& wheelTorque) {
    w.center = axle;
    w.contact = false;
    w.deflection = w.normalForce = w.longForce = w.slipRatio = 0;
    const double r = w.radius;

    // Géométrie du contact : point du sol le plus proche du centre de roue (marche aussi
    // sur les lèvres et dans les creux). Si le centre est passé sous le sol (chute), on
    // repousse selon la normale locale.
    Vec2 n;
    double defl;
    const double ground = terrain.height(axle.x);
    if (axle.y <= ground) {
        n = terrain.normal(axle.x);
        defl = r + (ground - axle.y) * n.y;
    } else {
        const Vec2 q = terrain.closestPoint(axle, r + 0.05);
        const Vec2 d = axle - q;
        const double dist = length(d);
        if (dist >= r) return;
        n = dist > 1e-9 ? d / dist : terrain.normal(axle.x);
        defl = r - dist;
    }
    const Vec2 lever = n * -(std::max(r - defl, 0.0)); // centre -> point de contact
    const Vec2 vAxle = carrier.velocityAt(axle);

    // Force normale : ressort radial du pneu + amortissement, + la jante si le pneu est
    // écrasé jusqu'au bout. Bornée à >= 0 : le sol pousse, il ne colle pas.
    double N = stiffness * defl - p_.tireDamping * dot(vAxle, n);
    if (defl > p_.tireRimDeflection) N += p_.tireRimStiffness * (defl - p_.tireRimDeflection);
    N = std::max(N, 0.0);

    // Force longitudinale : glissement = vitesse de la bande de roulement au contact
    // rapportée à la vitesse sol. Fx = mu(glissement) * N.
    const Vec2 t{n.y, -n.x}; // tangente au sol, vers l'avant
    const double vPatch = dot(vAxle + w.spin * perp(lever), t);
    const double vGround = dot(vAxle, t);
    const double slip = -vPatch / std::max(std::abs(vGround), p_.tireLowSpeed);
    double Fx = (slip >= 0 ? 1.0 : -1.0) * gripCurve(std::abs(slip), p_) * N;
    // Limite "adhérence statique" : jamais plus que la force qui annule vPatch en un pas
    // (sinon le glissement s'inverse à chaque pas -> vibration). Rend la moto stable à l'arrêt.
    const double invMass = dot(lever, lever) / inertia + 1.0 / carrier.mass;
    const double fStop = std::abs(vPatch) / (dt * invMass);
    Fx = clampd(Fx, -fStop, fStop);

    const Vec2 f = n * N + t * Fx;
    carrier.applyForce(f, axle);    // roue articulée sur l'axe : la force passe au porteur
    wheelTorque += cross(lever, f); // et le couple au contact fait tourner la roue

    w.contact = N > 0;
    w.contactPoint = axle + lever;
    w.normal = n;
    w.deflection = defl;
    w.normalForce = N;
    w.longForce = Fx;
    w.slipRatio = slip;
}

void BikeSim::step(const Inputs& raw, const Terrain& terrain, double dt) {
    const BikeParams& p = p_;
    Inputs in;
    in.throttle = clampd(raw.throttle, 0, 1);
    in.frontBrake = clampd(raw.frontBrake, 0, 1);
    in.rearBrake = clampd(raw.rearBrake, 0, 1);
    in.riderLean = clampd(raw.riderLean, -1, 1);
    s_.inputs = in;

    RigidBody2D& C = s_.chassis;
    RigidBody2D& F = s_.fork;
    RigidBody2D& S = s_.swingarm;
    RigidBody2D& R = s_.rider;
    // Masses modifiables en roulant (corps dont l'origine reste le CdG).
    C.mass = p.chassisMass;
    C.inertia = p.chassisInertia;
    R.mass = p.riderMass;
    R.inertia = p.riderInertia;
    const double Iwf = p.frontWheelInertia;
    // Rapport unique sans embrayage : le vilebrequin tourne avec la roue AR, son inertie
    // y est ressentie multipliée par le carré du rapport.
    const double Iwr = p.rearWheelInertia + p.engineInertia * p.gearRatio * p.gearRatio;

    RigidBody2D* bodies[] = {&C, &F, &S, &R};
    for (RigidBody2D* b : bodies) {
        b->force = {0, -kGravity * b->mass};
        b->torque = 0;
    }
    double tqF = 0, tqR = 0; // couples sur la rotation propre des roues

    // Traînée aérodynamique (moto + pilote) appliquée au châssis : F = -1/2 rho CxS |v| v.
    C.force -= C.vel * (0.5 * kAirDensity * p.dragArea * length(C.vel));

    // --- Pilote ---
    // La cible se déplace à vitesse limitée (un corps humain ne se téléporte pas).
    const double target = in.riderLean * p.riderLeanRange;
    s_.riderLean += clampd(target - s_.riderLean, -p.riderLeanSpeed * dt, p.riderLeanSpeed * dt);
    // Pieds et mains sont rappelés vers les repose-pieds / le guidon décalés de "shift" :
    // le corps se déplace de shift. Les efforts étant appliqués à ces points décalés, la moto
    // ressent le poids déplacé (couple en plus = shift x poids) : cabrage/plongée, transfert de charge.
    const Vec2 shift{s_.riderLean, 0};
    spring(C, C.toWorld(L_.pegs + shift), R, R.toWorld(L_.riderFeet), p.riderLegStiffness, p.riderLegDamping);
    spring(C, C.toWorld(L_.bars + shift), R, R.toWorld(L_.riderHands), p.riderArmStiffness, p.riderArmDamping);

    // --- Fourche --- (s = enfoncement le long de l'axe de direction)
    {
        const Vec2 u = rotate(L_.forkAxis, C.angle);
        const Vec2 axle = F.toWorld(L_.forkAxle);
        const double s = dot(axle - C.toWorld(L_.frontAxle), u);
        const double sd = dot(F.velocityAt(axle) - C.velocityAt(axle), u);
        // Ressort préchargé + amortisseur (coefficients séparés compression / détente).
        double f = p.forkSpringRate * (s + p.forkPreload) + (sd > 0 ? p.forkCompDamping : p.forkRebDamping) * sd;
        // Cône anti-talonnage : effort en k * enfoncement² sur la fin de course.
        const double pen = s - (p.forkTravel - p.forkBottomZone);
        if (pen > 0) f += p.forkBottomStiffness * pen * pen;
        F.applyForce(-u * f, axle);
        C.applyForce(u * f, axle);
        s_.forkTravel = s;
        s_.forkSpeed = sd;
    }

    // --- Amortisseur arrière via biellette progressive ---
    {
        const double phi = -(S.angle - C.angle); // rotation de compression du bras (0 = détendu)
        const double xw = p.swingarmLength * phi, vw = -p.swingarmLength * (S.angVel - C.angVel);
        // Rapport de levier LR = d(course roue)/d(course amortisseur), qui baisse avec
        // l'enfoncement : l'amortisseur travaille de plus en plus -> suspension progressive.
        const double b = (p.linkRatioEnd - p.linkRatioStart) / p.rearTravel;
        const double xc = std::max(xw, 0.0);
        const double lr = std::max(0.5, p.linkRatioStart + b * xc);
        // Course amortisseur = intégrale de dx_roue / LR(x_roue) = ln(LR/LR0)/b.
        const double xs = std::abs(b) < 1e-9 ? xc / p.linkRatioStart : std::log(lr / p.linkRatioStart) / b;
        const double vs = vw / lr;
        const double fShock = p.shockSpringRate * (xs + p.shockPreload) +
                              (vs > 0 ? p.shockCompDamping : p.shockRebDamping) * vs;
        // Travaux virtuels : force équivalente à la roue = force amortisseur / LR.
        double fWheel = fShock / lr;
        const double pen = xw - (p.rearTravel - p.rearBottomZone);
        if (pen > 0) fWheel += p.rearBottomStiffness * pen * pen; // butée progressive
        const double tau = fWheel * p.swingarmLength; // couple qui détend le bras
        S.torque += tau;
        C.torque -= tau;
        s_.rearTravel = xw;
        s_.rearSpeed = vw;
        s_.shockStroke = xs;
    }

    // --- Pneus ---
    tire(s_.front, F, F.toWorld(L_.forkAxle), p.tireStiffnessFront, Iwf, terrain, dt, tqF);
    tire(s_.rear, S, S.toWorld(L_.swingAxle), p.tireStiffnessRear, Iwr, terrain, dt, tqR);

    // --- Moteur --- couple à la roue AR, réaction sur le châssis (pignon de sortie de boîte).
    // Emplacement futur : boîte, embrayage, tension de chaîne / anti-squat.
    {
        const double wheelRel = -(s_.rear.spin - C.angVel); // rotation "en avant" de la roue / châssis
        const double we = wheelRel * p.gearRatio;
        double crank = 0;
        // Sous engineLaunchSpeed, un embrayage virtuel patine et fournit le couple de ce régime.
        if (we < p.engineRevLimit) crank += in.throttle * p.engineMaxTorque * torqueFraction(std::max(we, p.engineLaunchSpeed));
        if (we > 0) crank -= (1 - in.throttle) * p.engineBrakeTorque * std::min(we / p.engineRevLimit, 1.0);
        const double wheelTorque = crank * p.gearRatio;
        tqR -= wheelTorque; // rouler en avant = rotation horaire = couple négatif
        C.torque += wheelTorque;
        s_.engineSpeed = we;
    }

    // --- Résistance au roulement et freins --- frottements traités de façon implicite :
    // on calcule la vitesse de roue prévue, et le couple de frottement est borné à celui qui
    // l'annule ce pas-ci -> une roue bloquée reste bloquée sans vibrer.
    auto friction = [&](WheelState& w, double& tq, double I, RigidBody2D& carrier, double brakeTorque) {
        double wPred = w.spin + dt * tq / I;
        const double trr = p.rollingResistance * w.normalForce * w.radius;
        const double t1 = clampd(-wPred * I / dt, -trr, trr);
        tq += t1;
        wPred += dt * t1 / I;
        // Frein : frottement disque/étrier entre la roue et son porteur (fourche ou bras).
        const double tb = clampd(-(wPred - carrier.angVel) * I / dt, -brakeTorque, brakeTorque);
        tq += tb;
        carrier.torque -= tb;
    };
    friction(s_.front, tqF, Iwf, F, in.frontBrake * p.frontBrakeMaxTorque);
    friction(s_.rear, tqR, Iwr, S, in.rearBrake * p.rearBrakeMaxTorque);

    // --- Chute : quelques points du châssis et du pilote ne traversent pas le sol ---
    auto bodyContact = [&](RigidBody2D& b, Vec2 local) {
        const Vec2 pt = b.toWorld(local);
        const double pen = terrain.height(pt.x) - pt.y;
        if (pen <= 0) return;
        const Vec2 n = terrain.normal(pt.x), t{n.y, -n.x};
        const Vec2 v = b.velocityAt(pt);
        const double N = std::max(0.0, kBodyContactStiffness * pen - kBodyContactDamping * dot(v, n));
        const double vt = dot(v, t);
        const double Ft = -clampd(vt * b.mass / dt * 0.5, -kBodyContactMu * N, kBodyContactMu * N);
        b.applyForce(n * N + t * Ft, pt);
    };
    for (Vec2 pt : {L_.skidPlate, L_.rearFender, L_.seat, L_.steeringHead}) bodyContact(C, pt);
    for (Vec2 pt : {L_.riderHead, L_.riderHip}) bodyContact(R, pt);

    // --- Intégration (Euler semi-implicite) ---
    for (RigidBody2D* b : bodies) {
        b->vel += b->force * (dt / b->mass);
        b->angVel += b->torque * dt / b->inertia;
    }
    s_.front.spin += tqF * dt / Iwf;
    s_.rear.spin += tqR * dt / Iwr;
    solveJoints(dt);
    for (RigidBody2D* b : bodies) {
        b->pos += b->vel * dt;
        b->angle += b->angVel * dt;
    }
    s_.front.angle += s_.front.spin * dt;
    s_.rear.angle += s_.rear.spin * dt;
    s_.time += dt;
    s_.speed = length(C.vel);
}

// Liaisons rigides par impulsions séquentielles (type Box2D) : on corrige les vitesses
// pour respecter les liaisons, avec un terme de Baumgarte pour la dérive de position.
void BikeSim::solveJoints(double dt) {
    const BikeParams& p = p_;
    RigidBody2D& C = s_.chassis;
    RigidBody2D& F = s_.fork;
    RigidBody2D& S = s_.swingarm;
    const double mC = 1 / C.mass, iC = 1 / C.inertia;
    const double mF = 1 / F.mass, iF = 1 / F.inertia;
    const double mS = 1 / S.mass, iS = 1 / S.inertia;
    const double beta = kBaumgarte / dt;

    // Pivot du bras oscillant (2 lignes : x et y).
    const Vec2 rC = rotate(L_.pivot, C.angle), rS = rotate(L_.swingPivot, S.angle);
    const Vec2 pivotErr = (S.pos + rS) - (C.pos + rC);
    const double k11 = mC + mS + iC * rC.y * rC.y + iS * rS.y * rS.y;
    const double k12 = -iC * rC.x * rC.y - iS * rS.x * rS.y;
    const double k22 = mC + mS + iC * rC.x * rC.x + iS * rS.x * rS.x;
    const double detP = k11 * k22 - k12 * k12;

    // Glissière de fourche : axe perpendiculaire bloqué + rotation bloquée.
    const Vec2 u = rotate(L_.forkAxis, C.angle), n = perp(u);
    const Vec2 rA = rotate(L_.frontAxle, C.angle), rB = rotate(L_.forkAxle, F.angle);
    const Vec2 d = (F.pos + rB) - (C.pos + rA);
    const double perpErr = dot(d, n), angErr = F.angle - C.angle, sF = dot(d, u);
    const double a1 = cross(d + rA, n), a2 = cross(rB, n);
    const double g11 = mC + mF + iC * a1 * a1 + iF * a2 * a2;
    const double g12 = iC * a1 + iF * a2;
    const double g22 = iC + iF;
    const double detF = g11 * g22 - g12 * g12;
    // Butées de sécurité de la fourche (le long de l'axe).
    const double a1u = cross(d + rA, u), a2u = cross(rB, u);
    const double kAx = mC + mF + iC * a1u * a1u + iF * a2u * a2u;
    const double sMax = p.forkTravel + kHardStopMargin;
    // Butées du bras (en angle).
    const double phi = -(S.angle - C.angle), phiMax = (p.rearTravel + kHardStopMargin) / p.swingarmLength;
    const double kAng = iC + iS;

    // Ligne inégalité (butée) : impulsion cumulée >= 0 ; terme spéculatif err/dt quand
    // on est encore loin de la butée -> elle n'agit que si on allait la dépasser.
    auto limitImpulse = [&](double cdot, double err, double k, double& acc) {
        const double rhs = cdot + (err < 0 ? beta * err : err / dt);
        const double old = acc;
        acc = std::max(old - rhs / k, 0.0);
        return acc - old;
    };
    double accF0 = 0, accF1 = 0, accS0 = 0, accS1 = 0;

    for (int it = 0; it < kSolverIterations; ++it) {
        { // pivot
            const Vec2 cd = S.vel + S.angVel * perp(rS) - C.vel - C.angVel * perp(rC) + pivotErr * beta;
            const Vec2 P{-(k22 * cd.x - k12 * cd.y) / detP, -(-k12 * cd.x + k11 * cd.y) / detP};
            C.vel -= P * mC;
            C.angVel -= iC * cross(rC, P);
            S.vel += P * mS;
            S.angVel += iS * cross(rS, P);
        }
        { // glissière
            const double c1 = dot(n, F.vel - C.vel) + a2 * F.angVel - a1 * C.angVel + beta * perpErr;
            const double c2 = F.angVel - C.angVel + beta * angErr;
            const double l1 = -(g22 * c1 - g12 * c2) / detF;
            const double l2 = -(-g12 * c1 + g11 * c2) / detF;
            C.vel -= n * (mC * l1);
            C.angVel -= iC * (a1 * l1 + l2);
            F.vel += n * (mF * l1);
            F.angVel += iF * (a2 * l1 + l2);
        }
        { // butées fourche : s >= 0 (détente totale) et s <= sMax
            const double cd = dot(u, F.vel - C.vel) + a2u * F.angVel - a1u * C.angVel;
            double l = limitImpulse(cd, sF, kAx, accF0);
            l -= limitImpulse(-cd, sMax - sF, kAx, accF1);
            C.vel -= u * (mC * l);
            C.angVel -= iC * a1u * l;
            F.vel += u * (mF * l);
            F.angVel += iF * a2u * l;
        }
        { // butées bras : phi >= 0 et phi <= phiMax
            const double cd = C.angVel - S.angVel; // d(phi)/dt
            double l = limitImpulse(cd, phi, kAng, accS0);
            l -= limitImpulse(-cd, phiMax - phi, kAng, accS1);
            C.angVel += iC * l;
            S.angVel -= iS * l;
        }
    }
}

} // namespace mx
