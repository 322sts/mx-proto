// Jeu de test jouable : rendu 3D raylib, manette Xbox / clavier, réglages ImGui, courbes ImPlot.
// La physique est entièrement dans mx_physics ; ici on ne fait que lire les commandes,
// avancer la sim à pas fixe et dessiner.
#include "BikeSim.h"

#include "imgui.h"
#include "implot.h"
#include "raylib.h"
#include "raymath.h"
#include "rlImGui.h"
#include "rlgl.h"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

using namespace mx;

namespace {

constexpr double kDt = 0.001;   // s : physique à 1000 Hz
constexpr int kMaxSubsteps = 50; // par image : au-delà on abandonne le retard (pas de spirale de la mort)
constexpr double kDeg = 57.2958;

Vector3 V(Vec3 v) { return {float(v.x), float(v.y), float(v.z)}; }

// ---------------------------------------------------------------- Commandes
struct Controls {
    bool triggerArmed[2] = {false, false}; // certaines manettes renvoient 0 tant que la gâchette n'a pas bougé
    double throttleKb = 0, frontKb = 0, rearBrake = 0, steerKb = 0;
    float rearBrakeRampTime = 0.4f; // s pour serrer à fond le frein AR (bouton tout-ou-rien)

    double trigger(int i, GamepadAxis axis) {
        const float v = GetGamepadAxisMovement(0, axis); // -1 relâchée .. +1 enfoncée
        if (v < -0.5f) triggerArmed[i] = true;
        return triggerArmed[i] ? clampd((v + 1.0) * 0.5, 0, 1) : 0.0;
    }
    static double ramp(double v, bool held, double dt, double t) { return clampd(v + (held ? dt / t : -4 * dt / t), 0, 1); }
    static double deadzone(double v) { return std::abs(v) < 0.12 ? 0.0 : (v - std::copysign(0.12, v)) / 0.88; }

    Inputs read(double dt, bool keyboard) {
        Inputs in;
        const bool pad = IsGamepadAvailable(0);
        const bool kUp = keyboard && (IsKeyDown(KEY_UP) || IsKeyDown(KEY_W));
        const bool kDown = keyboard && (IsKeyDown(KEY_DOWN) || IsKeyDown(KEY_S));
        const bool kRear = keyboard && IsKeyDown(KEY_SPACE);
        throttleKb = ramp(throttleKb, kUp, dt, 0.15);
        frontKb = ramp(frontKb, kDown, dt, 0.15);
        rearBrake = ramp(rearBrake, kRear || (pad && IsGamepadButtonDown(0, GAMEPAD_BUTTON_LEFT_TRIGGER_1)), dt, rearBrakeRampTime);
        in.throttle = std::max(throttleKb, pad ? trigger(0, GAMEPAD_AXIS_RIGHT_TRIGGER) : 0.0);
        in.frontBrake = std::max(frontKb, pad ? trigger(1, GAMEPAD_AXIS_LEFT_TRIGGER) : 0.0);
        in.rearBrake = rearBrake;
        // Pilote avant/arrière : stick droit (ou gauche) vertical, haut = vers l'avant.
        double lean = 0, steer = 0;
        if (pad) {
            const double r = -GetGamepadAxisMovement(0, GAMEPAD_AXIS_RIGHT_Y);
            const double l = -GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_Y);
            lean = deadzone(std::abs(r) > std::abs(l) ? r : l);
            steer = deadzone(GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_X)); // inclinaison visée
        }
        if (keyboard && IsKeyDown(KEY_E)) lean = 1;
        if (keyboard && IsKeyDown(KEY_Q)) lean = -1;
        // Clavier : direction progressive (un stick ne saute pas de 0 à 1).
        const bool kLeft = keyboard && (IsKeyDown(KEY_LEFT) || IsKeyDown(KEY_A));
        const bool kRight = keyboard && (IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_D));
        const double steerTarget = (kRight ? 1.0 : 0.0) - (kLeft ? 1.0 : 0.0);
        steerKb += clampd(steerTarget - steerKb, -dt / 0.25, dt / 0.25);
        in.riderLean = lean;
        in.steer = std::abs(steer) > std::abs(steerKb) ? steer : steerKb;
        return in;
    }
};

// ---------------------------------------------------------------- Courbes
struct History {
    static constexpr size_t kMax = 3000; // ~5 s à 144 FPS
    std::vector<float> t, fork, rear, loadF, loadR, slipF, slipR, kmh, lean, steer, latF, latR;
    void push(const BikeState& s) {
        auto add = [](std::vector<float>& v, double x) {
            v.push_back(float(x));
            if (v.size() > kMax) v.erase(v.begin());
        };
        add(t, s.time);
        add(fork, s.forkTravel * 1000);
        add(rear, s.rearTravel * 1000);
        add(loadF, s.front.normalForce);
        add(loadR, s.rear.normalForce);
        add(slipF, s.front.slipRatio * 100);
        add(slipR, s.rear.slipRatio * 100);
        add(kmh, s.speed * 3.6);
        add(lean, s.leanAngle * kDeg);
        add(steer, s.steerAngle * kDeg);
        add(latF, s.front.slipAngle * kDeg);
        add(latR, s.rear.slipAngle * kDeg);
    }
    void clear() { *this = History{}; }
};

// ---------------------------------------------------------------- Presets (clé=valeur)
bool savePreset(const BikeParams& p, const std::string& path) {
    std::error_code ec;
    const std::filesystem::path fp(std::filesystem::u8path(path));
    if (fp.has_parent_path()) std::filesystem::create_directories(fp.parent_path(), ec);
    std::ofstream f(fp);
    if (!f) return false;
    f << "# preset MX proto (unites SI)\n" << std::setprecision(10);
    for (const ParamInfo& i : paramTable()) f << i.key << '=' << p.*i.field << '\n';
    return bool(f);
}

bool loadPreset(BikeParams& p, const std::string& path) {
    std::ifstream f(std::filesystem::u8path(path));
    if (!f) return false;
    std::string line;
    while (std::getline(f, line)) {
        const auto eq = line.find('=');
        if (line.empty() || line[0] == '#' || eq == std::string::npos) continue;
        const std::string key = line.substr(0, eq);
        for (const ParamInfo& i : paramTable()) {
            if (key != i.key) continue;
            try {
                p.*i.field = clampd(std::stod(line.substr(eq + 1)), i.min, i.max); // fichier = entrée non fiable
            } catch (...) {
            }
        }
    }
    return true;
}

// ---------------------------------------------------------------- Dessin
// Maillage de la piste, construit une fois : triangles non indexés (raylib indexe en 16 bits),
// éclairage d'un soleil fixe et damier de 2 m cuits dans les couleurs des sommets.
Model buildTerrainModel(const Terrain& t) {
    const double dx = 0.25, dz = 0.5;
    const int nx = int((t.xMax() - t.xMin()) / dx), nz = int((t.zMax() - t.zMin()) / dz);
    Mesh mesh{};
    mesh.triangleCount = nx * nz * 2;
    mesh.vertexCount = mesh.triangleCount * 3;
    mesh.vertices = static_cast<float*>(MemAlloc(sizeof(float) * 3 * mesh.vertexCount));
    mesh.colors = static_cast<unsigned char*>(MemAlloc(4 * mesh.vertexCount));
    const Vec3 sun = normalize(Vec3{0.4, 1.0, 0.3});
    int k = 0;
    auto vertex = [&](double x, double z) {
        const double h = t.height(x, z);
        const Vec3 n = t.normal(x, z);
        const bool lane = std::abs(z) < 9;
        const bool check = (int(std::floor(x / 2)) + int(std::floor(z / 2))) & 1;
        const double shade = (0.55 + 0.45 * std::max(0.0, dot(n, sun))) * (check ? 0.94 : 1.0);
        const double r = lane ? 139 : 150, g = lane ? 100 : 125, b = lane ? 62 : 80;
        mesh.vertices[3 * k] = float(x);
        mesh.vertices[3 * k + 1] = float(h);
        mesh.vertices[3 * k + 2] = float(z);
        mesh.colors[4 * k] = (unsigned char)(r * shade);
        mesh.colors[4 * k + 1] = (unsigned char)(g * shade);
        mesh.colors[4 * k + 2] = (unsigned char)(b * shade);
        mesh.colors[4 * k + 3] = 255;
        ++k;
    };
    for (int i = 0; i < nx; ++i) {
        for (int j = 0; j < nz; ++j) {
            const double x0 = t.xMin() + i * dx, x1 = x0 + dx, z0 = t.zMin() + j * dz, z1 = z0 + dz;
            // Sens antihoraire vu du dessus (y vers le haut).
            vertex(x0, z0); vertex(x0, z1); vertex(x1, z0);
            vertex(x1, z0); vertex(x0, z1); vertex(x1, z1);
        }
    }
    UploadMesh(&mesh, false);
    return LoadModelFromMesh(mesh);
}

void tube(Vec3 a, Vec3 b, double r, Color c) { DrawCylinderEx(V(a), V(b), float(r), float(r), 8, c); }

// Articulation (genou/coude) entre a et b, segments l1 et l2, pliée du côté de "bend".
Vec3 ik(Vec3 a, Vec3 b, double l1, double l2, Vec3 bend) {
    const Vec3 d = b - a;
    const double len = std::max(length(d), 1e-6), L = std::min(len, l1 + l2 - 1e-3);
    const Vec3 u = d / len;
    const double x = (l1 * l1 - l2 * l2 + L * L) / (2 * L);
    const Vec3 n = normalize(bend - u * dot(bend, u));
    return a + u * x + n * std::sqrt(std::max(0.0, l1 * l1 - x * x));
}

// Roue : pneu en tore (24 tronçons), jante, 6 rayons qui tournent avec la roue.
void drawWheel(const WheelState& w, const RigidBody3D& carrier) {
    const Vec3 a = carrier.dirToWorld(kZ);
    const Quat spin = carrier.q * axisAngle(kZ, w.angle);
    const double R = w.radius;
    for (int i = 0; i < 24; ++i) {
        const double t0 = i * 2 * PI / 24, t1 = (i + 1) * 2 * PI / 24;
        auto ring = [&](double t, double r) { return w.center + rotate(carrier.q, Vec3{std::cos(t), std::sin(t), 0}) * r; };
        tube(ring(t0, R - 0.045), ring(t1, R - 0.045), 0.045, {30, 30, 30, 255});
        tube(ring(t0, R - 0.1), ring(t1, R - 0.1), 0.012, {170, 170, 175, 255});
    }
    for (int i = 0; i < 6; ++i) {
        const Vec3 dir = rotate(spin, Vec3{std::cos(i * PI / 3), std::sin(i * PI / 3), 0});
        tube(w.center + a * (i % 2 ? 0.03 : -0.03), w.center + dir * (R - 0.1), 0.004, {150, 150, 150, 255});
    }
    tube(w.center - a * 0.06, w.center + a * 0.06, 0.03, {90, 90, 90, 255});
}

void drawBike(const BikeSim& sim, bool forces) {
    const BikeState& s = sim.state();
    const BikeLayout& L = sim.layout();
    const BikeParams& p = const_cast<BikeSim&>(sim).params();
    const RigidBody3D &C = s.chassis, &F = s.fork, &S = s.swingarm, &R = s.rider;
    const Color frame{220, 90, 20, 255}, metal{120, 120, 130, 255}, gold{210, 170, 60, 255}, plastic{40, 40, 160, 255};
    const Vec3 Z = C.dirToWorld(kZ);

    drawWheel(s.rear, S);
    drawWheel(s.front, F);

    // Bras oscillant (deux branches) + amortisseur.
    const Vec3 pivot = C.toWorld(L.pivot), rearAxle = S.toWorld(L.swingAxle);
    for (double side : {-1.0, 1.0}) tube(pivot + Z * (side * 0.09), rearAxle + Z * (side * 0.09), 0.025, metal);
    const Vec3 shockTop = C.toWorld(L.shockTop), shockLow = pivot + (rearAxle - pivot) * 0.35;
    tube(shockTop, shockLow, 0.03, {230, 200, 30, 255});

    // Cadre, moteur, selle/réservoir, repose-pieds.
    const Vec3 head = C.toWorld(L.steeringHead), seat = C.toWorld(L.seat), skid = C.toWorld(L.skidPlate);
    const Vec3 fender = C.toWorld(L.rearFender), pegs = C.toWorld(L.pegs);
    for (auto [a, b] : {std::pair{head, skid}, {head, seat}, {seat, fender}, {seat, pivot}, {skid, pivot}}) tube(a, b, 0.022, frame);
    rlPushMatrix();
    const Matrix m = MatrixMultiply(QuaternionToMatrix({float(C.q.x), float(C.q.y), float(C.q.z), float(C.q.w)}),
                                    MatrixTranslate(float(C.pos.x), float(C.pos.y), float(C.pos.z)));
    rlMultMatrixf(MatrixToFloatV(m).v);
    const Vec3 eng = L.skidPlate + Vec3{0.0, 0.2, 0};
    DrawCube(V(eng), 0.38f, 0.32f, 0.26f, {70, 70, 75, 255});
    rlPopMatrix();
    tube(head + C.dirToWorld(kX) * -0.05, seat + C.dirToWorld(kY) * 0.03, 0.09, plastic);
    tube(seat, fender, 0.05, plastic);
    for (double side : {-1.0, 1.0}) tube(pegs, pegs + Z * (side * (p.pegWidth + 0.05)), 0.018, metal);

    // Direction : le guidon et les tés tournent autour de l'axe de direction avec le bloc avant.
    const Vec3 u = C.dirToWorld(L.forkAxis), axisPt = C.toWorld(L.steerAxisPoint);
    const Quat steerQ = axisAngle(u, s.steerAngle);
    auto steered = [&](Vec3 world) { return axisPt + rotate(steerQ, world - axisPt); };
    const Vec3 barsMid = steered(C.toWorld(L.bars)), barZ = rotate(steerQ, Z);
    tube(barsMid - barZ * p.barWidth, barsMid + barZ * p.barWidth, 0.014, metal);
    tube(head, barsMid, 0.02, metal);
    // Fourche : fourreaux (bloc avant) + tubes plongeurs jusqu'au té du haut.
    const Vec3 axle = F.toWorld(L.forkAxle), Zf = F.dirToWorld(kZ), uf = F.dirToWorld(L.forkAxis);
    const Vec3 offset = L.frontAxle - L.steerAxisPoint; // déport, repère châssis
    for (double side : {-1.0, 1.0}) {
        const Vec3 low = axle + Zf * (side * 0.09);
        tube(low, low + uf * 0.5, 0.035, {50, 50, 55, 255});
        tube(low + uf * 0.45, steered(C.toWorld(L.steeringHead + offset + kZ * (side * 0.09))), 0.026, gold);
    }

    // Pilote : bassin, buste, tête ; genoux et coudes posés par cinématique inverse.
    const Vec3 hip = R.toWorld(L.riderHip), shoulder = R.toWorld(L.riderShoulder), headR = R.toWorld(L.riderHead);
    const Vec3 Zr = R.dirToWorld(kZ), fwd = C.dirToWorld(kX), up = C.dirToWorld(kY);
    const Color suit{30, 110, 200, 255}, pants{230, 230, 230, 255};
    tube(hip, shoulder, 0.13, suit);
    DrawSphere(V(headR), 0.13f, {240, 240, 240, 255});
    for (double side : {-1.0, 1.0}) {
        const Vec3 h = hip + Zr * (side * 0.11), sh = shoulder + Zr * (side * 0.19);
        const Vec3 foot = pegs + Z * (side * p.pegWidth), grip = barsMid + barZ * (side * (p.barWidth - 0.05));
        const Vec3 knee = ik(h, foot, 0.46, 0.46, fwd), elbow = ik(sh, grip, 0.30, 0.32, Z * side * 0.5 - up - fwd * 0.3);
        tube(h, knee, 0.07, pants);
        tube(knee, foot, 0.055, pants);
        tube(sh, elbow, 0.05, suit);
        tube(elbow, grip, 0.045, suit);
    }
    DrawSphere(V(R.pos), 0.03f, RED); // CdG pilote

    if (forces) {
        for (const WheelState* w : {&s.front, &s.rear}) {
            if (!w->contact) continue;
            const RigidBody3D& carrier = w == &s.front ? F : S;
            const Vec3 tx = normalize(cross(w->normal, carrier.dirToWorld(kZ))), ty = cross(tx, w->normal);
            const double k = 1.0 / 2500; // m par N
            tube(w->contactPoint, w->contactPoint + w->normal * (w->normalForce * k), 0.012, RED);
            tube(w->contactPoint, w->contactPoint + tx * (w->longForce * k), 0.012, {0, 160, 255, 255});
            tube(w->contactPoint, w->contactPoint + ty * (w->latForce * k), 0.012, {40, 220, 60, 255});
        }
    }
}

} // namespace

int main() {
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(1600, 900, "MX Proto - Etape 2 (3D)");
    SetExitKey(KEY_NULL);

    rlImGuiBeginInitImGui();
    ImGui::StyleColorsDark();
    if (FileExists("C:/Windows/Fonts/segoeui.ttf")) // police avec accents
        ImGui::GetIO().Fonts->AddFontFromFileTTF("C:/Windows/Fonts/segoeui.ttf", 17.0f);
    rlImGuiEndInitImGui();
    ImPlot::CreateContext();

    const Terrain track = Terrain::makeTestTrack();
    const Model terrainModel = buildTerrainModel(track);
    BikeSim sim;
    double resetX = Terrain::kStartX;
    sim.reset(BikeParams{}, track, resetX);

    Controls controls;
    History hist;
    Vec3 camTarget = sim.state().chassis.pos;
    double camYaw = 0;  // cap lissé de la caméra de poursuite
    float camDist = 4.5f;
    bool sideView = false;
    double accum = 0;
    bool paused = false, slowmo = false, showPanel = true, showForces = false;
    char presetPath[256] = "presets/mon_reglage.ini";
    std::string status;

    auto doReset = [&](double x) {
        resetX = x;
        sim.reset(sim.params(), track, x);
        hist.clear();
        accum = 0;
        camYaw = 0;
    };

    while (!WindowShouldClose()) {
        const double frameDt = GetFrameTime();
        const bool kb = !ImGui::GetIO().WantCaptureKeyboard;

        if (kb) {
            if (IsKeyPressed(KEY_R)) doReset(Terrain::kStartX);
            if (IsKeyPressed(KEY_ONE)) doReset(Terrain::kJump1X);
            if (IsKeyPressed(KEY_TWO)) doReset(Terrain::kJump2X);
            if (IsKeyPressed(KEY_THREE)) doReset(Terrain::kWhoopsX);
            if (IsKeyPressed(KEY_P)) paused = !paused;
            if (IsKeyPressed(KEY_T)) slowmo = !slowmo;
            if (IsKeyPressed(KEY_C)) sideView = !sideView;
            if (IsKeyPressed(KEY_F1)) showPanel = !showPanel;
        }
        if (!ImGui::GetIO().WantCaptureMouse) camDist = std::clamp(camDist * (1 - 0.1f * GetMouseWheelMove()), 1.5f, 30.0f);

        // Physique à pas fixe, découplée du rendu.
        const Inputs in = controls.read(frameDt, kb);
        if (!paused) {
            accum += std::min(frameDt, 0.1) * (slowmo ? 0.25 : 1.0);
            int n = 0;
            while (accum >= kDt && n < kMaxSubsteps) {
                sim.step(in, track, kDt);
                accum -= kDt;
                ++n;
            }
            if (n == kMaxSubsteps) accum = 0;
            hist.push(sim.state());
        }

        // Caméra : poursuite (derrière, cap lissé, sans le roulis) ou vue de côté (côté droit).
        const BikeState& s = sim.state();
        // Suit la moto exactement à l'horizontale ; la hauteur est lissée pour adoucir sauts et bosses.
        camTarget = {s.chassis.pos.x, camTarget.y + (s.chassis.pos.y - camTarget.y) * (1 - std::exp(-8.0 * frameDt)), s.chassis.pos.z};
        const Vec3 fwd = s.chassis.dirToWorld(kX);
        if (fwd.x * fwd.x + fwd.z * fwd.z > 0.01) {
            const double yaw = std::atan2(-fwd.z, fwd.x);
            camYaw += std::remainder(yaw - camYaw, 2 * PI) * (1 - std::exp(-2.5 * frameDt));
        }
        const Vec3 back{-std::cos(camYaw), 0, std::sin(camYaw)}, right{std::sin(camYaw), 0, std::cos(camYaw)};
        Camera3D camera{};
        camera.target = V(camTarget + Vec3{0, 0.4, 0});
        camera.position = V(sideView ? camTarget + right * camDist + Vec3{0, 0.5, 0} : camTarget + back * camDist + Vec3{0, 1.6, 0});
        camera.up = {0, 1, 0};
        camera.fovy = 55;
        camera.projection = CAMERA_PERSPECTIVE;

        BeginDrawing();
        ClearBackground({170, 205, 235, 255});
        BeginMode3D(camera);
        DrawModel(terrainModel, {0, 0, 0}, 1, WHITE);
        drawBike(sim, showForces);
        EndMode3D();

        rlImGuiBegin();
        // Infos toujours visibles.
        ImGui::SetNextWindowPos({10, 10}, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowBgAlpha(0.6f);
        ImGui::Begin("Infos", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse);
        ImGui::Text("%5.1f km/h   %5.0f tr/min", s.speed * 3.6, std::max(0.0, s.engineSpeed) * 9.5493);
        ImGui::Text("Inclinaison %+5.1f°   Braquage %+5.1f°   Couple guidon %+4.0f N.m", s.leanAngle * kDeg, s.steerAngle * kDeg, s.steerTorque);
        ImGui::Text("Fourche %5.0f mm   Roue AR %5.0f mm   Amortisseur %4.0f mm", s.forkTravel * 1000,
                    s.rearTravel * 1000, s.shockStroke * 1000);
        ImGui::Text("Charge AV %5.0f N   AR %5.0f N", s.front.normalForce, s.rear.normalForce);
        ImGui::Text("Glissement AV %+5.0f %%   AR %+5.0f %%", s.front.slipRatio * 100, s.rear.slipRatio * 100);
        ImGui::Text("Dérive AV %+5.1f°   AR %+5.1f°", s.front.slipAngle * kDeg, s.rear.slipAngle * kDeg);
        ImGui::ProgressBar(float(s.inputs.throttle), {120, 0}, "gaz");
        ImGui::SameLine();
        ImGui::ProgressBar(float(s.inputs.frontBrake), {120, 0}, "frein AV");
        ImGui::SameLine();
        ImGui::ProgressBar(float(s.inputs.rearBrake), {120, 0}, "frein AR");
        ImGui::ProgressBar(float(0.5 + 0.5 * s.inputs.riderLean), {372, 0}, "pilote  arrière <-> avant");
        ImGui::ProgressBar(float(0.5 + 0.5 * s.inputs.steer), {372, 0}, "inclinaison visée  gauche <-> droite");
        if (s.feetDown > 0) ImGui::TextDisabled("Pieds posés");
        if (paused) ImGui::TextColored({1, 0.4f, 0.4f, 1}, "PAUSE (P)");
        if (slowmo) ImGui::TextColored({1, 0.8f, 0.2f, 1}, "RALENTI x0.25 (T)");
        ImGui::TextDisabled(IsGamepadAvailable(0) ? "Manette OK" : "Pas de manette : clavier (W/S gaz/frein, A/D direction, Q/E pilote, Espace)");
        ImGui::TextDisabled("R départ  1/2/3 saut 1/saut 2/whoops  C vue côté  P pause  T ralenti  F1 panneau  molette zoom");
        ImGui::End();

        if (showPanel) {
            ImGui::SetNextWindowPos({float(GetScreenWidth() - 430), 10}, ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize({420, float(GetScreenHeight() - 20)}, ImGuiCond_FirstUseEver);
            ImGui::Begin("Réglages (F1)");
            if (ImGui::Button("Valeurs par défaut")) {
                sim.params() = BikeParams{};
                doReset(resetX);
                status = "Valeurs par défaut";
            }
            ImGui::InputText("Fichier", presetPath, sizeof presetPath);
            if (ImGui::Button("Sauver preset")) status = savePreset(sim.params(), presetPath) ? "Preset sauvé" : "Erreur d'écriture";
            ImGui::SameLine();
            if (ImGui::Button("Charger preset")) {
                BikeParams p = sim.params();
                if (loadPreset(p, presetPath)) {
                    sim.params() = p;
                    doReset(resetX);
                    status = "Preset chargé";
                } else {
                    status = "Fichier introuvable";
                }
            }
            if (!status.empty()) ImGui::TextDisabled("%s", status.c_str());
            ImGui::Checkbox("Vecteurs de force aux contacts", &showForces);
            ImGui::SliderFloat("Rampe frein AR (LB)", &controls.rearBrakeRampTime, 0.05f, 1.5f, "%.2f s");

            const char* section = nullptr;
            bool open = false;
            for (const ParamInfo& i : paramTable()) {
                if (!section || std::string(section) != i.section) {
                    section = i.section;
                    open = ImGui::CollapsingHeader(section);
                }
                if (!open) continue;
                ImGui::PushID(i.key);
                double& field = sim.params().*i.field;
                float v = float(field * i.displayScale);
                if (ImGui::SliderFloat(i.label, &v, float(i.min * i.displayScale), float(i.max * i.displayScale), i.format,
                                       ImGuiSliderFlags_AlwaysClamp))
                    field = clampd(v / i.displayScale, i.min, i.max);
                if (i.needsReset && ImGui::IsItemDeactivatedAfterEdit()) doReset(resetX);
                ImGui::PopID();
            }

            if (ImGui::CollapsingHeader("Courbes (5 s)", ImGuiTreeNodeFlags_DefaultOpen) && !hist.t.empty()) {
                const float tNow = hist.t.back();
                const int n = int(hist.t.size());
                auto plot = [&](const char* title, const char* unit, std::initializer_list<std::pair<const char*, const std::vector<float>*>> lines) {
                    if (ImPlot::BeginPlot(title, ImVec2(-1, 150))) {
                        ImPlot::SetupAxes(nullptr, unit, ImPlotAxisFlags_NoTickLabels, ImPlotAxisFlags_AutoFit);
                        ImPlot::SetupAxisLimits(ImAxis_X1, tNow - 5, tNow, ImGuiCond_Always);
                        for (auto& [name, data] : lines) ImPlot::PlotLine(name, hist.t.data(), data->data(), n);
                        ImPlot::EndPlot();
                    }
                };
                plot("Inclinaison / braquage", "°", {{"Inclinaison", &hist.lean}, {"Braquage", &hist.steer}});
                plot("Enfoncements", "mm", {{"Fourche", &hist.fork}, {"Roue AR", &hist.rear}});
                plot("Charge pneus", "N", {{"AV", &hist.loadF}, {"AR", &hist.loadR}});
                plot("Glissement pneus", "%", {{"AV", &hist.slipF}, {"AR", &hist.slipR}});
                plot("Dérive pneus", "°", {{"AV", &hist.latF}, {"AR", &hist.latR}});
                plot("Vitesse", "km/h", {{"Vitesse", &hist.kmh}});
            }
            ImGui::End();
        }
        rlImGuiEnd();
        EndDrawing();
    }

    UnloadModel(terrainModel);
    ImPlot::DestroyContext();
    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
