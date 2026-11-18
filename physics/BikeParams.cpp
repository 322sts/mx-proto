#include "BikeParams.h"

namespace mx {

namespace {
using P = BikeParams;
constexpr double DEG = 57.29578;  // rad -> deg for display
constexpr double RPM = 9.549297;  // rad/s -> tr/min
const char* G = "Masses et géométrie (reset auto)";
const char* FK = "Fourche";
const char* SH = "Amortisseur arrière";
const char* DI = "Direction";
const char* TY = "Pneus";
const char* RI = "Pilote";
const char* EN = "Moteur";
const char* BR = "Freins";

const ParamInfo kTable[] = {
    {G, "chassisMass", "Masse châssis", &P::chassisMass, 50, 120, 1, "%.1f kg", false},
    {G, "chassisInertia", "Inertie châssis (tangage)", &P::chassisInertia, 5, 25, 1, "%.1f kg.m²", false},
    {G, "chassisRollInertia", "Inertie châssis (roulis)", &P::chassisRollInertia, 2, 10, 1, "%.1f kg.m²", false},
    {G, "chassisYawInertia", "Inertie châssis (lacet)", &P::chassisYawInertia, 5, 20, 1, "%.1f kg.m²", false},
    {G, "forkLowerMass", "Masse fourreaux", &P::forkLowerMass, 3, 10, 1, "%.1f kg", true},
    {G, "swingarmMass", "Masse bras oscillant", &P::swingarmMass, 2.5, 8, 1, "%.1f kg", true},
    {G, "frontWheelMass", "Masse roue AV", &P::frontWheelMass, 5, 13, 1, "%.1f kg", true},
    {G, "rearWheelMass", "Masse roue AR", &P::rearWheelMass, 6, 15, 1, "%.1f kg", true},
    {G, "frontWheelInertia", "Inertie roue AV", &P::frontWheelInertia, 0.2, 0.9, 1, "%.2f kg.m²", false},
    {G, "rearWheelInertia", "Inertie roue AR", &P::rearWheelInertia, 0.3, 1.2, 1, "%.2f kg.m²", false},
    {G, "frontWheelRadius", "Rayon roue AV", &P::frontWheelRadius, 0.30, 0.40, 1000, "%.0f mm", true},
    {G, "rearWheelRadius", "Rayon roue AR", &P::rearWheelRadius, 0.30, 0.40, 1000, "%.0f mm", true},
    {G, "wheelbase", "Empattement (détendu)", &P::wheelbase, 1.38, 1.62, 1000, "%.0f mm", true},
    {G, "rakeAngle", "Angle de chasse", &P::rakeAngle, 0.40, 0.56, DEG, "%.1f°", true},
    {G, "forkLength", "Longueur fourche", &P::forkLength, 0.85, 1.05, 1000, "%.0f mm", true},
    {G, "forkOffset", "Déport de fourche", &P::forkOffset, 0.0, 0.07, 1000, "%.0f mm", true},
    {G, "swingarmLength", "Longueur bras", &P::swingarmLength, 0.52, 0.64, 1000, "%.0f mm", true},
    {G, "swingarmAngle", "Angle bras (détendu)", &P::swingarmAngle, 0.10, 0.40, DEG, "%.1f°", true},
    {G, "chassisComX", "CdG châssis X", &P::chassisComX, 0.60, 0.95, 1000, "%.0f mm", true},
    {G, "chassisComY", "CdG châssis Y", &P::chassisComY, 0.25, 0.50, 1000, "%.0f mm", true},
    {G, "pegX", "Repose-pieds X", &P::pegX, 0.45, 0.75, 1000, "%.0f mm", true},
    {G, "pegY", "Repose-pieds Y", &P::pegY, 0.00, 0.20, 1000, "%.0f mm", true},
    {G, "pegWidth", "Repose-pieds demi-écart", &P::pegWidth, 0.14, 0.28, 1000, "%.0f mm", true},
    {G, "barX", "Guidon X", &P::barX, 0.85, 1.15, 1000, "%.0f mm", true},
    {G, "barY", "Guidon Y", &P::barY, 0.80, 1.05, 1000, "%.0f mm", true},
    {G, "barWidth", "Guidon demi-largeur", &P::barWidth, 0.33, 0.45, 1000, "%.0f mm", true},
    {G, "riderComX", "CdG pilote X", &P::riderComX, 0.45, 0.85, 1000, "%.0f mm", true},
    {G, "riderComY", "CdG pilote Y", &P::riderComY, 0.60, 1.00, 1000, "%.0f mm", true},

    {FK, "forkTravel", "Débattement", &P::forkTravel, 0.25, 0.32, 1000, "%.0f mm", false},
    {FK, "forkSpringRate", "Raideur ressort", &P::forkSpringRate, 6000, 14000, 0.001, "%.2f N/mm", false},
    {FK, "forkPreload", "Précharge", &P::forkPreload, 0, 0.02, 1000, "%.1f mm", false},
    {FK, "forkCompDamping", "Amorti compression", &P::forkCompDamping, 100, 4000, 1, "%.0f N.s/m", false},
    {FK, "forkRebDamping", "Amorti détente", &P::forkRebDamping, 100, 5000, 1, "%.0f N.s/m", false},
    {FK, "forkBottomZone", "Zone butée", &P::forkBottomZone, 0.01, 0.08, 1000, "%.0f mm", false},
    {FK, "forkBottomStiffness", "Raideur butée", &P::forkBottomStiffness, 2e6, 5e7, 1e-6, "%.1f N/mm²", false},

    {SH, "rearTravel", "Débattement roue", &P::rearTravel, 0.26, 0.33, 1000, "%.0f mm", false},
    {SH, "shockSpringRate", "Raideur ressort", &P::shockSpringRate, 30000, 70000, 0.001, "%.1f N/mm", false},
    {SH, "shockPreload", "Précharge", &P::shockPreload, 0, 0.025, 1000, "%.1f mm", false},
    {SH, "linkRatioStart", "Rapport biellette début", &P::linkRatioStart, 2.0, 3.6, 1, "%.2f", false},
    {SH, "linkRatioEnd", "Rapport biellette fin", &P::linkRatioEnd, 1.5, 3.0, 1, "%.2f", false},
    {SH, "shockCompDamping", "Amorti compression", &P::shockCompDamping, 500, 12000, 1, "%.0f N.s/m", false},
    {SH, "shockRebDamping", "Amorti détente", &P::shockRebDamping, 1000, 20000, 1, "%.0f N.s/m", false},
    {SH, "rearBottomZone", "Zone butée", &P::rearBottomZone, 0.01, 0.08, 1000, "%.0f mm", false},
    {SH, "rearBottomStiffness", "Raideur butée", &P::rearBottomStiffness, 2e6, 5e7, 1e-6, "%.1f N/mm²", false},

    {DI, "barsInertia", "Inertie guidon", &P::barsInertia, 0.02, 0.3, 1, "%.2f kg.m²", true},
    {DI, "steerLock", "Butée de braquage", &P::steerLock, 0.4, 0.8, DEG, "%.0f°", false},
    {DI, "steerDamping", "Amortisseur de direction", &P::steerDamping, 0, 30, 1, "%.1f N.m.s/rad", false},

    {TY, "tireStiffnessFront", "Raideur radiale AV", &P::tireStiffnessFront, 60000, 250000, 0.001, "%.0f N/mm", false},
    {TY, "tireStiffnessRear", "Raideur radiale AR", &P::tireStiffnessRear, 60000, 250000, 0.001, "%.0f N/mm", false},
    {TY, "tireDamping", "Amorti radial", &P::tireDamping, 50, 1500, 1, "%.0f N.s/m", false},
    {TY, "tireRimDeflection", "Écrasement avant jante", &P::tireRimDeflection, 0.03, 0.09, 1000, "%.0f mm", false},
    {TY, "tireRimStiffness", "Raideur jante", &P::tireRimStiffness, 2e5, 3e6, 0.001, "%.0f N/mm", false},
    {TY, "tirePeakMu", "Adhérence max (μ)", &P::tirePeakMu, 0.4, 1.6, 1, "%.2f", false},
    {TY, "tirePeakSlip", "Glissement au pic", &P::tirePeakSlip, 0.05, 0.40, 100, "%.0f %%", false},
    {TY, "tireSlideMu", "Adhérence en glisse (μ)", &P::tireSlideMu, 0.2, 1.4, 1, "%.2f", false},
    {TY, "tireSlipFalloff", "Chute après le pic", &P::tireSlipFalloff, 0.05, 2.0, 100, "%.0f %%", false},
    {TY, "tirePeakSlipAngle", "Dérive au pic (tan)", &P::tirePeakSlipAngle, 0.05, 0.35, 1, "%.2f", false},
    {TY, "tireCamberThrust", "Poussée de carrossage", &P::tireCamberThrust, 0, 1.5, 1, "%.2f N/N/rad", false},
    {TY, "tireSpinLatCap", "Accroche latérale en patinage", &P::tireSpinLatCap, 1, 20, 1, "%.1f", false},
    {TY, "tireCrownRadius", "Rayon de profil", &P::tireCrownRadius, 0.02, 0.09, 1000, "%.0f mm", false},
    {TY, "tireLowSpeed", "Vitesse réf. basse", &P::tireLowSpeed, 0.5, 5, 1, "%.1f m/s", false},
    {TY, "rollingResistance", "Résistance roulement", &P::rollingResistance, 0, 0.15, 1, "%.3f", false},

    {RI, "riderMass", "Masse pilote", &P::riderMass, 45, 110, 1, "%.0f kg", false},
    {RI, "riderInertia", "Inertie pilote (tangage)", &P::riderInertia, 2, 15, 1, "%.1f kg.m²", false},
    {RI, "riderRollInertia", "Inertie pilote (roulis)", &P::riderRollInertia, 2, 15, 1, "%.1f kg.m²", false},
    {RI, "riderYawInertia", "Inertie pilote (lacet)", &P::riderYawInertia, 0.5, 6, 1, "%.1f kg.m²", false},
    {RI, "riderLegStiffness", "Raideur jambes", &P::riderLegStiffness, 5000, 80000, 0.001, "%.1f N/mm", false},
    {RI, "riderLegDamping", "Amorti jambes", &P::riderLegDamping, 100, 5000, 1, "%.0f N.s/m", false},
    {RI, "riderArmStiffness", "Raideur bras", &P::riderArmStiffness, 1000, 30000, 0.001, "%.1f N/mm", false},
    {RI, "riderArmDamping", "Amorti bras", &P::riderArmDamping, 20, 2000, 1, "%.0f N.s/m", false},
    {RI, "riderLeanRange", "Amplitude déplacement", &P::riderLeanRange, 0.05, 0.40, 1000, "%.0f mm", false},
    {RI, "riderLeanSpeed", "Vitesse déplacement", &P::riderLeanSpeed, 0.3, 4, 1, "%.1f m/s", false},
    {RI, "riderGripStiffness", "Gainage (genoux)", &P::riderGripStiffness, 500, 10000, 1, "%.0f N.m/rad", false},
    {RI, "riderGripDamping", "Gainage amorti", &P::riderGripDamping, 20, 1000, 1, "%.0f N.m.s/rad", false},
    {RI, "riderMaxLean", "Inclinaison max (vite)", &P::riderMaxLean, 0.3, 0.9, DEG, "%.0f°", false},
    {RI, "riderBermTrust", "Confiance dans le dévers", &P::riderBermTrust, 0, 1, 100, "%.0f %%", false},
    {RI, "riderLeanResponse", "Guidon : réactivité", &P::riderLeanResponse, 1, 10, 1, "%.1f /s", false},
    {RI, "riderMaxRollRate", "Guidon : bascule max", &P::riderMaxRollRate, 0.5, 3, DEG, "%.0f°/s", false},
    {RI, "riderSteerGain", "Guidon : suivi de bascule", &P::riderSteerGain, 2, 20, 1, "%.1f /s", false},
    {RI, "riderMaxSteerTorque", "Guidon : couple max", &P::riderMaxSteerTorque, 10, 100, 1, "%.0f N.m", false},
    {RI, "riderBarHold", "Guidon : fermeté des bras", &P::riderBarHold, 30, 400, 1, "%.0f N.m/rad", false},
    {RI, "footDownSpeed", "Pied à terre sous", &P::footDownSpeed, 0, 6, 3.6, "%.0f km/h", false},

    {EN, "engineMaxTorque", "Couple max", &P::engineMaxTorque, 10, 80, 1, "%.0f N.m", false},
    {EN, "engineRevLimit", "Rupteur", &P::engineRevLimit, 838, 1466, RPM, "%.0f tr/min", false},
    {EN, "engineLaunchSpeed", "Régime embrayage virtuel", &P::engineLaunchSpeed, 0, 838, RPM, "%.0f tr/min", false},
    {EN, "gearRatio", "Rapport total", &P::gearRatio, 6, 25, 1, "%.1f", false},
    {EN, "countershaftX", "Pignon sortie boîte X", &P::countershaftX, 0.50, 0.72, 1000, "%.0f mm", true},
    {EN, "countershaftY", "Pignon sortie boîte Y", &P::countershaftY, 0.05, 0.25, 1000, "%.0f mm", true},
    {EN, "frontSprocketRadius", "Rayon pignon", &P::frontSprocketRadius, 0.025, 0.045, 1000, "%.1f mm", false},
    {EN, "rearSprocketRadius", "Rayon couronne", &P::rearSprocketRadius, 0.10, 0.16, 1000, "%.1f mm", false},
    {EN, "engineInertia", "Inertie vilebrequin", &P::engineInertia, 0.0005, 0.01, 1, "%.4f kg.m²", false},
    {EN, "engineBrakeTorque", "Frein moteur", &P::engineBrakeTorque, 0, 25, 1, "%.1f N.m", false},
    {EN, "dragArea", "Traînée (Cx.S)", &P::dragArea, 0, 1.2, 1, "%.2f m²", false},

    {BR, "frontBrakeMaxTorque", "Couple max AV", &P::frontBrakeMaxTorque, 0, 1500, 1, "%.0f N.m", false},
    {BR, "rearBrakeMaxTorque", "Couple max AR", &P::rearBrakeMaxTorque, 0, 1200, 1, "%.0f N.m", false},
};
} // namespace

std::span<const ParamInfo> paramTable() { return kTable; }

} // namespace mx
