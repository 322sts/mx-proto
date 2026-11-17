#pragma once
// Tous les réglages de la moto et du pilote, en unités SI (m, kg, s, N, rad).
// Valeurs par défaut : 450 de cross moderne (~106 kg en ordre de marche), pilote de
// 75 kg debout. Les points de géométrie sont donnés dans le repère moto, suspensions
// TOTALEMENT DÉTENDUES, par rapport à l'axe de roue arrière (x vers l'avant, y vers le haut,
// z vers la droite ; la moto est symétrique, les largeurs sont des demi-écarts).
#include <span>

namespace mx {

struct BikeParams {
    // --- Masses et géométrie (appliquées au reset) ---
    double chassisMass = 77.0;        // kg    cadre + moteur + réservoir + fluides (suspendu)
    double chassisInertia = 11.0;     // kg.m² inertie en tangage autour du CdG châssis
    double chassisRollInertia = 4.0;  // kg.m² en roulis (inclinaison)
    double chassisYawInertia = 10.0;  // kg.m² en lacet
    double forkLowerMass = 6.0;       // kg    fourreaux + axe + étrier (non suspendu)
    double swingarmMass = 4.5;        // kg    bras oscillant + biellette (non suspendu)
    double frontWheelMass = 8.5;      // kg    jante 21" + pneu + disque
    double rearWheelMass = 10.0;      // kg    jante 18" + pneu + disque + couronne
    double frontWheelInertia = 0.45;  // kg.m² inertie de rotation autour de l'axe de roue
    double rearWheelInertia = 0.90;   // kg.m² pneu lourd + mousse : plus d'effet gaz / frein AR en l'air
    double frontWheelRadius = 0.355;  // m     rayon extérieur du pneu
    double rearWheelRadius = 0.340;   // m
    double wheelbase = 1.50;          // m     distance horizontale entre axes, détendu
    double rakeAngle = 0.4712;        // rad   angle de chasse / verticale (27°)
    double forkLength = 0.95;         // m     axe de roue -> tête de fourche (dessin)
    double forkOffset = 0.045;        // m     déport : axe de roue en avant de l'axe de direction (chasse ~135 mm)
    double swingarmLength = 0.58;     // m     pivot -> axe de roue AR
    double swingarmAngle = 0.28;      // rad   pivot au-dessus de l'axe, détendu (16°)
    double chassisComX = 0.78;        // m     CdG châssis depuis l'axe AR
    double chassisComY = 0.36;        // m
    double pegX = 0.60, pegY = 0.10;  // m     repose-pieds depuis l'axe AR
    double pegWidth = 0.20;           // m     demi-écart des repose-pieds
    double barX = 1.00, barY = 0.91;  // m     poignées du guidon depuis l'axe AR
    double barWidth = 0.40;           // m     demi-largeur du guidon (800 mm)
    double riderComX = 0.62;          // m     CdG pilote (debout, neutre) depuis l'axe AR
    double riderComY = 0.86;          // m

    // --- Fourche (le long de l'axe de direction, deux tubes ensemble) ---
    double forkTravel = 0.300;          // m
    double forkSpringRate = 12000.0;    // N/m   2 ressorts de 0,61 kg/mm
    double forkPreload = 0.005;         // m     précharge ressort
    double forkCompDamping = 2100.0;    // N.s/m compression
    double forkRebDamping = 2650.0;     // N.s/m détente
    double forkBottomZone = 0.040;      // m     fin de course où agit le cône anti-talonnage
    double forkBottomStiffness = 1.5e7; // N/m²  F = k * enfoncement² (progressif, pas un mur)

    // --- Amortisseur arrière (via biellette progressive) ---
    double rearTravel = 0.310;          // m     débattement à la roue (arc à l'axe)
    double shockSpringRate = 62000.0;   // N/m   à l'amortisseur (6,3 kg/mm)
    double shockPreload = 0.008;        // m
    double linkRatioStart = 2.8;        // -     course roue / course amortisseur, détendu
    double linkRatioEnd = 1.85;         // -     en fin de course (plus bas = plus dur = progressif)
    double shockCompDamping = 7400.0;   // N.s/m à la tige d'amortisseur, compression
    double shockRebDamping = 11600.0;   // N.s/m détente
    double rearBottomZone = 0.035;      // m     zone de la butée (en course roue)
    double rearBottomStiffness = 1.5e7; // N/m²  à la roue

    // --- Direction ---
    double barsInertia = 0.08;  // kg.m²   guidon + tés + tubes, autour de l'axe de direction
    double steerLock = 0.70;    // rad     butée de braquage (40°)
    double steerDamping = 6.0;  // N.m.s/rad amortisseur de direction + frottements

    // --- Pneus ---
    double tireStiffnessFront = 120000.0; // N/m   raideur radiale
    double tireStiffnessRear = 140000.0;  // N/m
    double tireDamping = 400.0;           // N.s/m amortissement radial
    double tireRimDeflection = 0.060;     // m     écrasement où la jante commence à taper
    double tireRimStiffness = 1.0e6;      // N/m   raideur ajoutée une fois sur la jante
    double tirePeakMu = 0.89;             // -     adhérence max (terre)
    double tirePeakSlip = 0.20;           // -     taux de glissement au pic
    double tireSlideMu = 0.68;            // -     adhérence en patinage / blocage complet
    double tireSlipFalloff = 0.40;        // -     vitesse de baisse après le pic (en taux de glissement)
    double tirePeakSlipAngle = 0.15;      // -     tan(dérive) au pic d'adhérence latérale (~8,5°)
    double tireCamberThrust = 0.7;        // N/N/rad poussée de carrossage (pneu incliné qui pousse vers l'intérieur)
    double tireCrownRadius = 0.05;        // m     rayon du profil arrondi du pneu (contact qui se décale en angle)
    double tireLowSpeed = 2.0;            // m/s   plancher du dénominateur du glissement (basse vitesse)
    double rollingResistance = 0.04;      // -     coefficient de résistance au roulement (terre)

    // --- Moteur et transmission (rapport unique ; boîte/embrayage à une étape suivante) ---
    double engineMaxTorque = 50.0;    // N.m    couple vilebrequin au sommet de la courbe
    double engineRevLimit = 1257.0;   // rad/s  (12000 tr/min)
    double engineLaunchSpeed = 419.0; // rad/s  (4000 tr/min) embrayage virtuel qui patine en dessous
    double gearRatio = 13.0;          // -      rapport total vilebrequin -> roue AR
    // Transmission finale par chaîne : sa tension tire le bras oscillant vers le pignon de sortie
    // de boîte et retient l'arrière sous les gaz (anti-squat). La ligne de chaîne vs le pivot fixe l'effet.
    double countershaftX = 0.61;        // m      pignon de sortie de boîte depuis l'axe AR (détendu)
    double countershaftY = 0.18;        // m      (2 cm au-dessus du pivot : anti-squat ~100 % au sag)
    double frontSprocketRadius = 0.033; // m      13 dents, chaîne 520
    double rearSprocketRadius = 0.1265; // m      50 dents
    double engineInertia = 0.0025;    // kg.m²  vilebrequin + embrayage, ressenti x gearRatio² à la roue
    double engineBrakeTorque = 8.0;   // N.m    couple vilebrequin au rupteur, gaz fermés
    double dragArea = 0.55;           // m²     Cx * surface frontale moto + pilote

    // --- Freins ---
    double frontBrakeMaxTorque = 650.0; // N.m levier à fond
    double rearBrakeMaxTorque = 400.0;  // N.m pédale à fond

    // --- Pilote (un corps tenu par des ressorts-amortisseurs aux pieds et aux mains) ---
    double riderMass = 75.0;            // kg
    double riderInertia = 6.0;          // kg.m² en tangage
    double riderRollInertia = 6.0;      // kg.m² en roulis
    double riderYawInertia = 2.0;       // kg.m² en lacet
    double riderLegStiffness = 30000.0; // N/m    pieds <-> repose-pieds
    double riderLegDamping = 1200.0;    // N.s/m
    double riderArmStiffness = 6000.0;  // N/m    mains <-> guidon
    double riderArmDamping = 300.0;     // N.s/m
    double riderLeanRange = 0.25;       // m      déplacement du corps stick à fond
    double riderLeanSpeed = 1.2;        // m/s    vitesse max de déplacement du corps
    // Gainage : genoux serrés sur la moto, le pilote suit le roulis et le lacet de la moto.
    double riderGripStiffness = 3000.0; // N.m/rad
    double riderGripDamping = 200.0;    // N.m.s/rad
    // Le pilote vise l'inclinaison demandée en poussant sur le guidon (couple autour de l'axe
    // de direction) : le contre-braquage n'est pas programmé, il sort de la physique.
    double riderMaxLean = 0.60;         // rad    inclinaison visée stick à fond (34° ; tan = 0,67 < mu : marge avant la glisse)
    double riderLeanResponse = 4.0;     // 1/s    vitesse de bascule voulue par radian d'écart d'inclinaison
    double riderMaxRollRate = 1.5;      // rad/s  bascule la plus rapide qu'il s'autorise (~85°/s)
    double riderSteerGain = 40.0;       // N.m.s/rad couple au guidon par rad/s d'écart de vitesse de bascule
    double riderMaxSteerTorque = 40.0;  // N.m    force max dans le guidon
    double footDownSpeed = 3.0;         // m/s    en dessous, le pilote pose les pieds
};

struct Inputs {
    double throttle = 0;   // 0..1
    double frontBrake = 0; // 0..1
    double rearBrake = 0;  // 0..1
    double riderLean = 0;  // -1 (arrière) .. +1 (avant)
    double steer = 0;      // -1 (gauche) .. +1 (droite) : inclinaison visée
};

// Métadonnées pour les curseurs, les presets et les tests : les bornes gardent la sim stable.
struct ParamInfo {
    const char* section;
    const char* key;   // clé dans les fichiers preset
    const char* label;
    double BikeParams::*field;
    double min, max;     // bornes SI
    double displayScale; // l'UI affiche valeur * displayScale
    const char* format;  // format printf de la valeur affichée (avec unité)
    bool needsReset;     // géométrie / masses non suspendues : la moto est reconstruite
};
std::span<const ParamInfo> paramTable();

} // namespace mx
