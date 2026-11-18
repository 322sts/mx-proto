#include "BikeSim.h"
#include <algorithm>
#include <array>
#include <limits>
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
constexpr double kFootStiffness = 30000.0;        // N/m   pied posé au sol (jambe tendue)
constexpr double kFootDamping = 1500.0;           // N.s/m
constexpr double kFootOut = 0.40;                 // m     écart latéral du pied posé

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

// Adhérence sur terre selon le glissement combiné normalisé s (1 = au pic) :
// montée parabolique jusqu'au pic (pente nulle au pic), puis décroissance exponentielle
// douce vers l'adhérence de glisse -> pas de décrochage brutal. En ligne droite (dérive nulle)
// s = taux de glissement / glissement au pic : même courbe qu'en 2D.
double gripCurve(double s, const BikeParams& p) {
    if (s < 1) return p.tirePeakMu * (2 * s - s * s);
    return p.tireSlideMu + (p.tirePeakMu - p.tireSlideMu) * std::exp(-(s - 1) * p.tirePeakSlip / p.tireSlipFalloff);
}

// Ressort-amortisseur de longueur à vide nulle entre le point pa de a et le point pb de b.
void spring(RigidBody3D& a, Vec3 pa, RigidBody3D& b, Vec3 pb, double k, double c) {
    const Vec3 f = (pa - pb) * k - (b.velocityAt(pb) - a.velocityAt(pa)) * c; // force sur b
    b.applyForce(f, pb);
    a.applyForce(-f, pa);
}

// Inertie (diagonale, axes x/y/z) d'une masse ponctuelle décalée de d du centre de masse.
Vec3 pointInertia(double m, Vec3 d) { return {m * (sq(d.y) + sq(d.z)), m * (sq(d.x) + sq(d.z)), m * (sq(d.x) + sq(d.y))}; }
// Barre fine de longueur len selon la direction unitaire u, autour de son milieu.
Vec3 rodInertia(double m, double len, Vec3 u) {
    const double k = m * len * len / 12;
    return {k * (1 - sq(u.x)), k * (1 - sq(u.y)), k * (1 - sq(u.z))};
}
} // namespace

void BikeSim::reset(const BikeParams& p, const Terrain& terrain, double x, double z, double yaw) {
    p_ = p;
    s_ = BikeState{};
    const double Rf = p.frontWheelRadius, Rr = p.rearWheelRadius;

    // Points de conception : moto détendue, droite, horizontale, origine à l'axe de roue AR.
    const Vec3 rearAxle{0, 0, 0};
    const Vec3 pivot = Vec3{std::cos(p.swingarmAngle), std::sin(p.swingarmAngle), 0} * p.swingarmLength;
    const Vec3 frontAxle{p.wheelbase, Rf - Rr, 0};
    const Vec3 axis{-std::sin(p.rakeAngle), std::cos(p.rakeAngle), 0};   // de l'axe de roue vers la tête de fourche
    const Vec3 axisFwd{std::cos(p.rakeAngle), std::sin(p.rakeAngle), 0}; // perpendiculaire, vers l'avant
    const Vec3 axisPoint = frontAxle - axisFwd * p.forkOffset;           // axe de direction (derrière l'axe de roue)
    const Vec3 com{p.chassisComX, p.chassisComY, 0};
    const Vec3 riderCom{p.riderComX, p.riderComY, 0};
    const Vec3 pegs{p.pegX, p.pegY, 0}, bars{p.barX, p.barY, 0};

    // Bloc avant = fourreaux (CdG ~25 cm au-dessus de l'axe, ~50 cm de long) + roue AV.
    // La rotation propre de la roue est à part ; ici seulement son inertie transversale (I/2, disque).
    const double mF = p.forkLowerMass + p.frontWheelMass;
    const Vec3 lowersCom = frontAxle + axis * 0.25;
    const Vec3 forkCom = (lowersCom * p.forkLowerMass + frontAxle * p.frontWheelMass) / mF;
    Vec3 IF = rodInertia(p.forkLowerMass, 0.5, axis) + pointInertia(p.forkLowerMass, lowersCom - forkCom) +
              pointInertia(p.frontWheelMass, frontAxle - forkCom) + Vec3{p.frontWheelInertia / 2, p.frontWheelInertia / 2, 0};
    // Guidon + tés : leur masse est comptée dans le châssis (suspendue), seule leur inertie autour
    // de l'axe de direction tourne avec le bloc avant.
    IF += Vec3{sq(axis.x), sq(axis.y), 0} * p.barsInertia;
    // Bras = barre + roue AR à son extrémité.
    const double mS = p.swingarmMass + p.rearWheelMass;
    const Vec3 mid = (pivot + rearAxle) * 0.5;
    const Vec3 swingCom = (mid * p.swingarmMass + rearAxle * p.rearWheelMass) / mS;
    const Vec3 IS = rodInertia(p.swingarmMass, p.swingarmLength, normalize(pivot - rearAxle)) +
                    pointInertia(p.swingarmMass, mid - swingCom) + pointInertia(p.rearWheelMass, rearAxle - swingCom) +
                    Vec3{p.rearWheelInertia / 2, p.rearWheelInertia / 2, 0};

    L_.rearAxle = rearAxle - com;
    L_.pivot = pivot - com;
    L_.frontAxle = frontAxle - com;
    L_.steerAxisPoint = axisPoint - com;
    L_.steeringHead = axisPoint + axis * p.forkLength - com;
    L_.pegs = pegs - com;
    L_.bars = bars - com;
    L_.forkAxis = axis;
    L_.shockTop = pivot + Vec3{0.05, 0.30, 0} - com;          // dessin seulement
    L_.skidPlate = Vec3{pivot.x + 0.20, -0.05, 0} - com;      // sabot moteur
    L_.rearFender = Vec3{-0.35, 0.62, 0} - com;               // bout de garde-boue AR
    L_.seat = Vec3{0.40, 0.62, 0} - com;
    L_.countershaft = Vec3{p.countershaftX, p.countershaftY, 0} - com;
    L_.foot = Vec3{p.pegX, -Rr + 0.08, 0} - com;              // semelle du pied posé, ~2 cm du sol au sag
    L_.forkAxle = frontAxle - forkCom;
    L_.forkAxisPoint = axisPoint - forkCom;
    L_.swingPivot = pivot - swingCom;
    L_.swingAxle = rearAxle - swingCom;
    L_.riderFeet = pegs - riderCom;
    L_.riderHands = bars - riderCom;
    L_.riderHip = {-0.10, -0.08, 0};
    L_.riderShoulder = {0.12, 0.40, 0};
    L_.riderHead = {0.18, 0.58, 0};

    // Placement : axe AR 1 cm au-dessus du sol (le plus haut des deux sols sous les roues), moto
    // droite au cap demandé, tout à l'arrêt.
    const Quat heading = axisAngle(kY, yaw);
    const Vec3 fwd = rotate(heading, kX);
    const double ground = std::max(terrain.height(x, z), terrain.height(x + fwd.x * p.wheelbase, z + fwd.z * p.wheelbase));
    const Vec3 origin{x, ground + Rr + 0.01, z};
    auto place = [&](RigidBody3D& b, Vec3 designCom, double m, Vec3 I) {
        b = RigidBody3D{};
        b.pos = origin + rotate(heading, designCom);
        b.q = heading;
        b.mass = m;
        b.inertia = I;
    };
    place(s_.chassis, com, p.chassisMass, {p.chassisRollInertia, p.chassisYawInertia, p.chassisInertia});
    place(s_.fork, forkCom, mF, IF);
    place(s_.swingarm, swingCom, mS, IS);
    place(s_.rider, riderCom, p.riderMass, {p.riderRollInertia, p.riderYawInertia, p.riderInertia});
    s_.front.radius = Rf;
    s_.rear.radius = Rr;

    // 1,5 s freins serrés : la moto se pose, les suspensions trouvent leur sag.
    Inputs hold;
    hold.frontBrake = hold.rearBrake = 1;
    for (int i = 0; i < 1500; ++i) step(hold, terrain, 0.001);
    s_.time = 0;
}

double BikeSim::steerAngle() const {
    // Avant du bloc avant vu dans le repère châssis, projeté sur le plan perpendiculaire à l'axe.
    const Vec3 u = L_.forkAxis;
    const Vec3 ref1 = normalize(kX - u * dot(kX, u)), ref2 = cross(u, ref1); // ref2 = vers la gauche
    const Vec3 xF = rotateInv(s_.chassis.q, s_.fork.dirToWorld(kX));
    return std::atan2(dot(xF, ref2), dot(xF, ref1));
}

double BikeSim::swingAngle() const {
    const Vec3 xS = rotateInv(s_.chassis.q, s_.swingarm.dirToWorld(kX));
    return std::atan2(xS.y, xS.x);
}

void BikeSim::tire(WheelState& w, RigidBody3D& carrier, Vec3 axle, Vec3 a, double stiffness, double spinInertia,
                   const Terrain& terrain, double dt, double& spinTorque) {
    w.center = axle;
    w.contact = false;
    w.deflection = w.normalForce = w.longForce = w.latForce = w.slipRatio = w.slipAngle = w.camber = 0;
    const double R = w.radius, rc = std::min(p_.tireCrownRadius, 0.5 * R);

    // Géométrie : pneu = tore (rayon de profil rc). Le point de la "ligne médiane" du profil le
    // plus bas vers le sol est dans le plan de roue, du côté opposé à la normale du sol ; en
    // angle, le contact se décale donc sur le flanc. On réévalue le sol sous le contact estimé
    // (3 passes) pour suivre les lèvres et les creux.
    Vec3 n = terrain.normal(axle.x, axle.z), core, ground;
    for (int it = 0; it < 3; ++it) {
        Vec3 d = n - a * dot(n, a);
        const double dl = length(d);
        if (dl < 1e-6) return; // roue couchée à plat : pas de contact pneu (la chute est gérée par les points du châssis)
        d = d / -dl;
        core = axle + d * (R - rc);
        const Vec3 cp = core - n * rc;
        double hx, hz;
        ground = {cp.x, terrain.sample(cp.x, cp.z, hx, hz), cp.z};
        n = normalize(Vec3{-hx, 1, -hz});
    }
    const double coreHeight = dot(core - ground, n);
    const double defl = rc - coreHeight;
    if (defl <= 0) return;
    const Vec3 P = core - n * coreHeight; // point de contact, sur le sol
    const Vec3 lever = P - axle;
    const Vec3 vAxle = carrier.velocityAt(axle);

    // Force normale : ressort radial du pneu + amortissement, + la jante si le pneu est
    // écrasé jusqu'au bout. Bornée à >= 0 : le sol pousse, il ne colle pas.
    double N = stiffness * defl - p_.tireDamping * dot(vAxle, n);
    if (defl > p_.tireRimDeflection) N += p_.tireRimStiffness * (defl - p_.tireRimDeflection);
    N = std::max(N, 0.0);

    // Glissements. Vitesse de la bande de roulement au contact = vitesse de l'axe + rotation de
    // la roue (celle du porteur hors axe + la rotation propre autour de l'axe).
    const Vec3 wWheel = carrier.angVel - a * dot(carrier.angVel, a) + a * w.spin;
    const Vec3 vPatch = vAxle + cross(wWheel, lever);
    const Vec3 tx = normalize(cross(n, a)); // vers l'avant de la roue, dans le plan du sol
    const Vec3 ty = cross(tx, n);           // vers la droite de la roue
    const double vGx = dot(vAxle, tx), vPx = dot(vPatch, tx), vPy = dot(vPatch, ty);
    const double denom = std::max(std::abs(vGx), p_.tireLowSpeed);
    const double kappa = -vPx / denom;  // taux de glissement longitudinal
    const double tanAlpha = -vPy / denom; // dérive : > 0 quand la bande glisse vers la gauche
    const double gamma = std::asin(clampd(-dot(a, n), -1, 1)); // carrossage, > 0 = roue penchée à droite

    // Glissement combiné (ellipse d'adhérence) : chaque direction normalisée par son glissement
    // au pic, adhérence mu(s) partagée entre les deux. Le carrossage s'ajoute à la dérive comme une
    // dérive équivalente : pente initiale d'adhérence 2*mu_pic/dérive_pic, d'où la conversion.
    const double sx = kappa / p_.tirePeakSlip;
    const double sy = tanAlpha / p_.tirePeakSlipAngle + p_.tireCamberThrust * gamma / (2 * p_.tirePeakMu);
    const double s = std::hypot(sx, sy);
    double Fx = 0, Fy = 0;
    if (s > 1e-9) {
        const double F = gripCurve(s, p_) * N;
        // Partage entre long et latéral : sur la terre, une roue qui patine fort garde de l'accroche
        // latérale (les crampons s'enfoncent et labourent). Au-delà de tireSpinLatCap (en glissements
        // au pic), le patinage n'"avale" plus davantage la force latérale.
        const double sxd = clampd(sx, -p_.tireSpinLatCap, p_.tireSpinLatCap), sd = std::hypot(sxd, sy);
        Fx = F * sxd / sd;
        Fy = F * sy / sd;
    }
    // Limite "adhérence statique" : jamais plus que la force qui annule le glissement en un pas
    // (sinon il s'inverse à chaque pas -> vibration). Rend la moto stable à l'arrêt.
    const double armX = dot(cross(lever, tx), a);
    const double invMx = armX * armX / spinInertia + 1.0 / carrier.mass;
    Fx = clampd(Fx, -std::abs(vPx) / (dt * invMx), std::abs(vPx) / (dt * invMx));
    const Vec3 cy = cross(P - carrier.pos, ty);
    const double invMy = 1.0 / carrier.mass + dot(cy, carrier.invInertiaMul(cy));
    Fy = clampd(Fy, -std::abs(vPy) / (dt * invMy), std::abs(vPy) / (dt * invMy));

    // Roue articulée sur l'axe : la force passe au porteur ; du couple au contact, la composante
    // selon l'axe fait tourner la roue, le reste (roulis/lacet du contact décalé) va au porteur.
    const Vec3 f = n * N + tx * Fx + ty * Fy;
    carrier.applyForce(f, axle);
    const Vec3 tq = cross(lever, f);
    const double ta = dot(tq, a);
    spinTorque += ta;
    carrier.torque += tq - a * ta;

    w.contact = N > 0;
    w.contactPoint = P;
    w.normal = n;
    w.deflection = defl;
    w.normalForce = N;
    w.longForce = Fx;
    w.latForce = Fy;
    w.slipRatio = kappa;
    w.slipAngle = std::atan(tanAlpha);
    w.camber = gamma;
}

// Point d'un corps qui touche le sol (chute, pied posé) : ressort-amortisseur selon la normale,
// frottement de Coulomb borné (pas plus que ce qui arrête le glissement en deux pas).
void BikeSim::bodyContact(RigidBody3D& b, Vec3 local, const Terrain& terrain, double stiffness, double damping, double dt) {
    const Vec3 pt = b.toWorld(local);
    double hx, hz;
    const double pen = terrain.sample(pt.x, pt.z, hx, hz) - pt.y;
    if (pen <= 0 || stiffness <= 0) return;
    const Vec3 n = normalize(Vec3{-hx, 1, -hz});
    const Vec3 v = b.velocityAt(pt);
    const double N = std::max(0.0, stiffness * pen * n.y - damping * dot(v, n));
    const Vec3 vt = v - n * dot(v, n);
    const double vtl = length(vt);
    const Vec3 Ft = vtl > 1e-9 ? vt * (-std::min(kBodyContactMu * N, vtl * b.mass / dt * 0.5) / vtl) : Vec3{};
    b.applyForce(n * N + Ft, pt);
}

void BikeSim::step(const Inputs& raw, const Terrain& terrain, double dt) {
    const BikeParams& p = p_;
    Inputs in;
    in.throttle = clampd(raw.throttle, 0, 1);
    in.frontBrake = clampd(raw.frontBrake, 0, 1);
    in.rearBrake = clampd(raw.rearBrake, 0, 1);
    in.riderLean = clampd(raw.riderLean, -1, 1);
    in.steer = clampd(raw.steer, -1, 1);
    s_.inputs = in;

    RigidBody3D& C = s_.chassis;
    RigidBody3D& F = s_.fork;
    RigidBody3D& S = s_.swingarm;
    RigidBody3D& R = s_.rider;
    // Masses modifiables en roulant (corps dont l'origine reste le CdG).
    C.mass = p.chassisMass;
    C.inertia = {p.chassisRollInertia, p.chassisYawInertia, p.chassisInertia};
    R.mass = p.riderMass;
    R.inertia = {p.riderRollInertia, p.riderYawInertia, p.riderInertia};
    const double Iwf = p.frontWheelInertia;
    // Rapport unique sans embrayage : le vilebrequin tourne avec la roue AR, son inertie
    // y est ressentie multipliée par le carré du rapport.
    const double Iwr = p.rearWheelInertia + p.engineInertia * p.gearRatio * p.gearRatio;

    RigidBody3D* bodies[] = {&C, &F, &S, &R};
    for (RigidBody3D* b : bodies) {
        b->force = {0, -kGravity * b->mass, 0};
        b->torque = {};
    }
    double tqF = 0, tqR = 0; // couples sur la rotation propre des roues
    const Vec3 aF = F.dirToWorld(kZ), aR = S.dirToWorld(kZ); // axes de roue (vers la droite)

    // Effet gyroscopique : le moment cinétique propre de la roue H = I*spin*axe tourne avec son
    // porteur ; le porteur doit fournir dH/dt = ω x H, il subit donc -ω x H. (Vilebrequin : son
    // moment cinétique réel est ~20 fois plus petit que celui de la roue AR, négligé.)
    F.torque -= cross(F.angVel, aF * (p.frontWheelInertia * s_.front.spin));
    S.torque -= cross(S.angVel, aR * (p.rearWheelInertia * s_.rear.spin));

    // Traînée aérodynamique (moto + pilote) appliquée au châssis : F = -1/2 rho CxS |v| v.
    C.force -= C.vel * (0.5 * kAirDensity * p.dragArea * length(C.vel));

    // --- Pilote ---
    // La cible se déplace à vitesse limitée (un corps humain ne se téléporte pas).
    const double target = in.riderLean * p.riderLeanRange;
    s_.riderLean += clampd(target - s_.riderLean, -p.riderLeanSpeed * dt, p.riderLeanSpeed * dt);
    // Pieds et mains (gauche et droite) rappelés vers les repose-pieds / le guidon décalés de
    // "shift" : le corps se déplace de shift. Les efforts étant appliqués à ces points décalés, la
    // moto ressent le poids déplacé (couple en plus = shift x poids) : cabrage/plongée, transfert de charge.
    const Vec3 shift{s_.riderLean, 0, 0};
    for (double side : {-1.0, 1.0}) {
        const Vec3 peg{0, 0, side * p.pegWidth}, bar{0, 0, side * p.barWidth};
        spring(C, C.toWorld(L_.pegs + peg + shift), R, R.toWorld(L_.riderFeet + peg), p.riderLegStiffness / 2, p.riderLegDamping / 2);
        spring(C, C.toWorld(L_.bars + bar + shift), R, R.toWorld(L_.riderHands + bar), p.riderArmStiffness / 2, p.riderArmDamping / 2);
    }
    // Gainage : roulis et lacet du pilote rappelés vers ceux de la moto (genoux serrés). Le
    // tangage reste libre (jambes/bras ci-dessus), comme en 2D.
    {
        const Vec3 rel = rotationVector(conj(C.q) * R.q);       // rotation pilote / moto, repère moto
        const Vec3 wrel = rotateInv(C.q, R.angVel - C.angVel);
        const Vec3 tq = C.dirToWorld({-(p.riderGripStiffness * rel.x + p.riderGripDamping * wrel.x),
                                      -(p.riderGripStiffness * rel.y + p.riderGripDamping * wrel.y), 0});
        R.torque += tq;
        C.torque -= tq;
    }

    // --- Fourche --- (s = enfoncement le long de l'axe de direction)
    const Vec3 u = C.dirToWorld(L_.forkAxis);
    {
        const Vec3 axle = F.toWorld(L_.forkAxle);
        const double s = dot(F.toWorld(L_.forkAxisPoint) - C.toWorld(L_.steerAxisPoint), u);
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

    // --- Direction ---
    {
        const double delta = steerAngle(), deltaDot = dot(F.angVel - C.angVel, u);
        // Inclinaison de la moto (roulis par rapport à la verticale, + = à droite) et sa vitesse.
        const Vec3 fwd = C.dirToWorld(kX), up = C.dirToWorld(kY);
        const Vec3 rightH = normalize(cross(fwd, kY));
        const double lean = std::atan2(dot(up, rightH), dot(up, kY));
        const double leanRate = dot(C.angVel, normalize(Vec3{fwd.x, 0, fwd.z}));
        // Pilote : le stick demande une inclinaison (à fond = le maximum que l'adhérence permet).
        // Vitesse perçue = vitesse horizontale (les bosses ne doivent pas faire varier la cible).
        const double v = std::max(std::hypot(C.vel.x, C.vel.z), 1.0), L = p.wheelbase;
        const double rMin = 1.5 * L / std::tan(p.steerLock * std::cos(p.rakeAngle)); // virage le plus serré tenable
        // Dévers du sol (berm) : l'adhérence limite l'inclinaison par rapport au SOL, pas à la verticale.
        // Dans un berm penché de β vers l'intérieur, il peut pencher de riderMaxLean + β et tourner bien
        // plus serré. Le pilote "voit" le dévers sous la moto avec un temps de réaction (0,2 s) : une
        // mesure instantanée sautait à chaque bosse (roue en l'air) et faisait dandiner la moto.
        {
            const Vec3 mid = (s_.front.center + s_.rear.center) * 0.5;
            const Vec3 n = terrain.normal(mid.x, mid.z);
            const double seen = std::atan2(dot(n, rightH), n.y); // + = sol penché vers la droite
            s_.groundBank += (seen - s_.groundBank) * std::min(1.0, dt / 0.2);
        }
        const double bank = s_.groundBank;
        // Cercle d'adhérence du pneu AR : toute la poussée passe par lui, et il porte aussi sa part de
        // l'effort latéral. Il reste donc sqrt(mu² - (Fx/N)²) pour tenir l'angle ; le pilote relève la
        // moto d'autant (sortie de virage façon MX Bikes, par la physique). Rapporter la poussée au
        // poids total sous-estimait l'usure du pneu AR : glisse puis highside au banc.
        // Poussée lue au pneu AR (Fx/N), filtrée sur 0,15 s, gelée quand la roue AR est en l'air.
        // Une roue qui patine au-delà du pic n'a plus de marge : il relève la moto pour retrouver de la
        // motricité (sinon glisse puis highside au banc quand l'arrière raccroche).
        if (s_.rear.normalForce > 200) {
            const double spin = clampd(s_.rear.slipRatio / p.tirePeakSlip, 0, 1) * p.tirePeakMu;
            const double use = std::max(std::max(0.0, s_.rear.longForce) / s_.rear.normalForce, spin);
            s_.driveUse += (use - s_.driveUse) * std::min(1.0, dt / 0.15);
        }
        const double latBudget = std::sqrt(std::max(0.0, sq(p.tirePeakMu) - sq(s_.driveUse)));
        // Il ne compte que sur une part du dévers (riderBermTrust) : en plongeant vers la corde, moins
        // relevée, il garde de la marge (sinon l'avant décroche, constaté au banc stick à fond).
        const double maxLean = clampd(std::atan(std::min(std::tan(p.riderMaxLean), latBudget)) + p.riderBermTrust * (in.steer >= 0 ? bank : -bank), 0.0, 1.2);
        // Le stick commande l'inclinaison (fraction du maximum) : accélérer ne la fait plus augmenter.
        // À basse vitesse, limitée au virage le plus serré permis par la butée de braquage.
        const double lowCap = std::atan(v * v / (kGravity * rMin));
        const double want = clampd(in.steer * maxLean, -lowCap, lowCap);
        // Il choisit une bascule voulue (vitesse de roulis plafonnée), d'où une accélération de roulis
        // voulue, et en déduit l'angle de guidon par le modèle du pendule inversé sur ses roues :
        //   h φ'' = g tan φ + (v²/L) δe   (δe = braquage effectif, + = à gauche)
        //   => δe = (L / v²) (h φ''_voulu - g tan φ).
        // Le terme -g tan φ L / v² est le braquage du virage établi (empattement x courbure) ; le reste
        // est l'équilibre : braquer du côté où elle tombe. Pour pencher à droite il braque d'abord un
        // peu à gauche (contre-braquage), d'autant moins qu'il va vite. Rien n'est imposé à la moto :
        // il tient le guidon avec ses bras (ressort-amortisseur), la chasse, le gyroscope et les pneus agissent.
        constexpr double h = 0.8; // m, hauteur du CdG moto + pilote dans le modèle mental du pilote
        const double rate = clampd(p.riderLeanResponse * (want - lean), -p.riderMaxRollRate, p.riderMaxRollRate);
        const double rollAcc = p.riderSteerGain * (rate - leanRate);
        // Le terme d'équilibre est calculé avec une vitesse plafonnée : à haute vitesse le vrai 1/v² le
        // rendrait trop faible face aux efforts de la roue AV sur le guidon (chasse, gyroscope, pneu),
        // et l'inclinaison dériverait au-delà de la cible. Un pilote corrige plus fermement que le modèle.
        const double vFb = std::min(v, 7.0);
        // Il ne met jamais plus de guidon que ce qui sert à cette vitesse : 1,5 x le braquage du virage le
        // plus serré tenable, + 3° pour l'équilibre. Sans cette limite, avant délesté (entrée de berm sur
        // les gaz), il braquait de 25° à 40 km/h et la moto partait en guidonnage.
        const double deltaMax = std::min(p.steerLock, 1.5 * L * kGravity * std::tan(maxLean) / (v * v * std::cos(p.rakeAngle)) + 0.05);
        const double deltaLean = clampd(((L / (vFb * vFb)) * h * rollAcc - (L / (v * v)) * kGravity * std::tan(clampd(lean, -1.2, 1.2))) / std::cos(p.rakeAngle),
                                        -deltaMax, deltaMax);
        // Presque à l'arrêt (pieds posés), braquer ne fait plus pencher la moto : il tourne simplement
        // le guidon comme demandé par le stick (droite = braque à droite).
        const double w = clampd(s_.speed - 0.5, 0, 1);
        // Sans appui sur la roue AV (whoops, sauts), corriger au guidon ne sert à rien : il garde le guidon
        // droit, sinon l'avant retombe braqué en travers (moto qui "croise").
        s_.frontGrip += (std::min(1.0, s_.front.normalForce / 300.0) - s_.frontGrip) * std::min(1.0, dt / 0.05);
        const double deltaCmd = w * (0.3 + 0.7 * s_.frontGrip) * deltaLean + (1 - w) * (-in.steer * p.steerLock);
        const double tauRider = clampd(p.riderBarHold * (deltaCmd - delta) - 0.05 * p.riderBarHold * deltaDot,
                                       -p.riderMaxSteerTorque, p.riderMaxSteerTorque);
        const double tau = tauRider - p.steerDamping * deltaDot;
        F.torque += u * tau;
        C.torque -= u * tau;
        s_.steerAngle = delta;
        s_.leanAngle = lean;
        s_.steerTorque = tauRider;
    }

    // --- Amortisseur arrière via biellette progressive ---
    const Vec3 zC = C.dirToWorld(kZ);
    {
        const double phi = -swingAngle(); // rotation de compression du bras (0 = détendu)
        const double xw = p.swingarmLength * phi, vw = -p.swingarmLength * dot(S.angVel - C.angVel, zC);
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
        S.torque += zC * tau;
        C.torque -= zC * tau;
        s_.rearTravel = xw;
        s_.rearSpeed = vw;
        s_.shockStroke = xs;
    }

    // --- Pneus ---
    tire(s_.front, F, F.toWorld(L_.forkAxle), aF, p.tireStiffnessFront, Iwf, terrain, dt, tqF);
    tire(s_.rear, S, S.toWorld(L_.swingAxle), aR, p.tireStiffnessRear, Iwr, terrain, dt, tqR);

    // --- Moteur --- couple à la roue AR transmis par la chaîne.
    // Emplacement futur : boîte, embrayage.
    {
        const double wheelRel = -(s_.rear.spin - dot(C.angVel, aR)); // rotation "en avant" de la roue / châssis
        const double we = wheelRel * p.gearRatio;
        double crank = 0;
        // Sous engineLaunchSpeed, un embrayage virtuel patine et fournit le couple de ce régime.
        if (we < p.engineRevLimit) crank += in.throttle * p.engineMaxTorque * torqueFraction(std::max(we, p.engineLaunchSpeed));
        if (we > 0) crank -= (1 - in.throttle) * p.engineBrakeTorque * std::min(we / p.engineRevLimit, 1.0);
        const double wheelTorque = crank * p.gearRatio;
        tqR -= wheelTorque; // rouler en avant = rotation horaire = couple négatif

        // Chaîne : brin supérieur tendu entre le haut de la couronne et le haut du pignon.
        // Tension T = couple roue / bras de levier de la ligne de chaîne autour de l'axe AR.
        // Elle tire l'axe AR (donc le bras) vers le pignon -> couple qui détend le bras autour
        // du pivot = anti-squat ; la réaction (-T) s'applique au cadre au pignon de sortie.
        // La somme des couples (roue + bras + cadre) est nulle : la chaîne est interne à la moto.
        // shortcut: brin supérieur seul, même en frein moteur (T < 0) ; brin inférieur à l'étape boîte.
        // shortcut: chaîne dans le plan de symétrie (en vrai à gauche, ~10 cm) ; négligeable sur le lacet.
        const Vec3 axle = S.toWorld(L_.swingAxle);
        const Vec3 rearTop = axle + C.dirToWorld(kY) * p.rearSprocketRadius;
        const Vec3 frontTop = C.toWorld(L_.countershaft + kY * p.frontSprocketRadius);
        const Vec3 e = normalize(frontTop - rearTop);
        const double arm = dot(cross(e, rearTop - axle), aR); // distance axe AR -> ligne de chaîne (> 0)
        const double T = wheelTorque / arm;
        S.applyForce(e * T, axle);
        C.applyForce(-e * T, frontTop);
        s_.chainTension = T;
        s_.engineSpeed = we;
    }

    // --- Résistance au roulement et freins --- frottements traités de façon implicite :
    // on calcule la vitesse de roue prévue, et le couple de frottement est borné à celui qui
    // l'annule ce pas-ci -> une roue bloquée reste bloquée sans vibrer.
    auto friction = [&](WheelState& w, double& tq, double I, RigidBody3D& carrier, Vec3 a, double brakeTorque) {
        double wPred = w.spin + dt * tq / I;
        const double trr = p.rollingResistance * w.normalForce * w.radius;
        const double t1 = clampd(-wPred * I / dt, -trr, trr);
        tq += t1;
        wPred += dt * t1 / I;
        // Frein : frottement disque/étrier entre la roue et son porteur (fourche ou bras).
        const double tb = clampd(-(wPred - dot(carrier.angVel, a)) * I / dt, -brakeTorque, brakeTorque);
        tq += tb;
        carrier.torque -= a * tb;
    };
    friction(s_.front, tqF, Iwf, F, aF, in.frontBrake * p.frontBrakeMaxTorque);
    friction(s_.rear, tqR, Iwr, S, aR, in.rearBrake * p.rearBrakeMaxTorque);

    // --- Chute : points du châssis et du pilote qui ne traversent pas le sol ---
    for (Vec3 pt : {L_.skidPlate, L_.rearFender, L_.seat, L_.steeringHead})
        bodyContact(C, pt, terrain, kBodyContactStiffness, kBodyContactDamping, dt);
    for (double side : {-1.0, 1.0}) {
        bodyContact(C, L_.bars + Vec3{0, 0, side * p.barWidth}, terrain, kBodyContactStiffness, kBodyContactDamping, dt);
        bodyContact(C, L_.pegs + Vec3{0, 0, side * (p.pegWidth + 0.06)}, terrain, kBodyContactStiffness, kBodyContactDamping, dt);
        bodyContact(R, L_.riderShoulder + Vec3{0, 0, side * 0.22}, terrain, kBodyContactStiffness, kBodyContactDamping, dt);
    }
    for (Vec3 pt : {L_.riderHead, L_.riderHip}) bodyContact(R, pt, terrain, kBodyContactStiffness, kBodyContactDamping, dt);
    // Pied à terre : sous footDownSpeed le pilote sort les pieds (relevés progressivement sur 1 m/s
    // au-dessus) ; ils ne portent que si la moto penche de leur côté.
    s_.feetDown = p.footDownSpeed > 0 ? clampd(p.footDownSpeed + 1.0 - s_.speed, 0, 1) : 0.0;
    for (double side : {-1.0, 1.0})
        bodyContact(C, L_.foot + Vec3{0, 0, side * kFootOut}, terrain, kFootStiffness * s_.feetDown, kFootDamping * s_.feetDown, dt);

    // --- Intégration (Euler semi-implicite) ---
    for (RigidBody3D* b : bodies) b->integrateVelocity(dt);
    s_.front.spin += tqF * dt / Iwf;
    s_.rear.spin += tqR * dt / Iwr;
    solveJoints(dt);
    for (RigidBody3D* b : bodies) b->integratePosition(dt);
    s_.front.angle += (s_.front.spin - dot(F.angVel, aF)) * dt;
    s_.rear.angle += (s_.rear.spin - dot(S.angVel, aR)) * dt;
    s_.time += dt;
    s_.speed = length(C.vel);
}

// Liaisons rigides par impulsions séquentielles (type Box2D/Bullet) : chaque ligne de contrainte
// a son jacobien J (linéaire + angulaire pour les deux corps) ; on corrige les vitesses pour
// annuler J.v, avec un terme de Baumgarte pour la dérive de position.
void BikeSim::solveJoints(double dt) {
    const BikeParams& p = p_;
    RigidBody3D& C = s_.chassis;
    RigidBody3D& F = s_.fork;
    RigidBody3D& S = s_.swingarm;
    const double beta = kBaumgarte / dt;
    constexpr double inf = std::numeric_limits<double>::infinity();

    struct Row {
        RigidBody3D *a, *b;
        Vec3 la, aa, lb, ab; // jacobien
        Vec3 ia, ib;         // I^-1 * partie angulaire
        double eff, bias, lo, hi, acc;
    };
    std::array<Row, 16> rows;
    int n = 0;
    auto add = [&](RigidBody3D& A, RigidBody3D& B, Vec3 la, Vec3 aa, Vec3 lb, Vec3 ab, double bias, double lo, double hi) {
        Row& r = rows[n++];
        r = {&A, &B, la, aa, lb, ab, A.invInertiaMul(aa), B.invInertiaMul(ab), 0, bias, lo, hi, 0};
        r.eff = 1.0 / (dot(la, la) / A.mass + dot(aa, r.ia) + dot(lb, lb) / B.mass + dot(ab, r.ib));
    };
    auto equal = [&](RigidBody3D& A, RigidBody3D& B, Vec3 la, Vec3 aa, Vec3 lb, Vec3 ab, double err) {
        add(A, B, la, aa, lb, ab, beta * err, -inf, inf);
    };
    // Butée (err >= 0 doit rester vrai) : terme spéculatif err/dt quand on est encore loin ->
    // elle n'agit que si on allait la dépasser ce pas-ci.
    auto limit = [&](RigidBody3D& A, RigidBody3D& B, Vec3 la, Vec3 aa, Vec3 lb, Vec3 ab, double err) {
        add(A, B, la, aa, lb, ab, err < 0 ? beta * err : err / dt, 0, inf);
    };
    const Vec3 zero{};

    // Pivot du bras : point commun (3 lignes) + axes de rotation alignés (2 lignes).
    {
        const Vec3 rA = C.dirToWorld(L_.pivot), rB = S.dirToWorld(L_.swingPivot);
        const Vec3 err = (S.pos + rB) - (C.pos + rA);
        for (Vec3 e : {kX, kY, kZ}) equal(C, S, -e, -cross(rA, e), e, cross(rB, e), dot(err, e));
        const Vec3 zA = C.dirToWorld(kZ), angErr = cross(zA, S.dirToWorld(kZ));
        for (Vec3 e : {C.dirToWorld(kX), C.dirToWorld(kY)}) equal(C, S, zero, -e, zero, e, dot(angErr, e));
        // Butées du bras (en angle) : 0 <= phi <= phiMax.
        const double phi = -swingAngle(), phiMax = (p.rearTravel + kHardStopMargin) / p.swingarmLength;
        limit(C, S, zero, zA, zero, -zA, phi);
        limit(C, S, zero, -zA, zero, zA, phiMax - phi);
    }
    // Direction + fourche : liaison cylindrique autour de l'axe de direction u.
    {
        const Vec3 u = C.dirToWorld(L_.forkAxis);
        const Vec3 pA = C.toWorld(L_.steerAxisPoint), pB = F.toWorld(L_.forkAxisPoint);
        const Vec3 rA = pA - C.pos, rB = pB - F.pos, d = pB - pA;
        const Vec3 n1 = C.dirToWorld(normalize(cross(kZ, L_.forkAxis))), n2 = C.dirToWorld(kZ);
        const Vec3 angErr = cross(u, F.dirToWorld(L_.forkAxis));
        for (Vec3 e : {n1, n2}) {
            equal(C, F, -e, -cross(rA + d, e), e, cross(rB, e), dot(d, e)); // l'axe de roue reste sur l'axe de direction
            equal(C, F, zero, -e, zero, e, dot(angErr, e));                   // seule rotation permise : autour de u
        }
        // Butées de sécurité de la fourche (le long de l'axe) : 0 <= s <= sMax.
        const double s = dot(d, u), sMax = p.forkTravel + kHardStopMargin;
        limit(C, F, -u, -cross(rA + d, u), u, cross(rB, u), s);
        limit(C, F, u, cross(rA + d, u), -u, -cross(rB, u), sMax - s);
        // Butées de braquage : -lock <= delta <= lock.
        const double delta = steerAngle();
        limit(C, F, zero, -u, zero, u, delta + p.steerLock);
        limit(C, F, zero, u, zero, -u, p.steerLock - delta);
    }

    for (int it = 0; it < kSolverIterations; ++it) {
        for (int i = 0; i < n; ++i) {
            Row& r = rows[i];
            const double cdot = dot(r.la, r.a->vel) + dot(r.aa, r.a->angVel) + dot(r.lb, r.b->vel) + dot(r.ab, r.b->angVel);
            const double old = r.acc;
            r.acc = clampd(old - (cdot + r.bias) * r.eff, r.lo, r.hi);
            const double l = r.acc - old;
            r.a->vel += r.la * (l / r.a->mass);
            r.a->angVel += r.ia * l;
            r.b->vel += r.lb * (l / r.b->mass);
            r.b->angVel += r.ib * l;
        }
    }
}

} // namespace mx
