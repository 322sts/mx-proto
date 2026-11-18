// Jeu de test jouable : rendu 3D raylib, manette Xbox / clavier, réglages ImGui, courbes ImPlot.
// La physique est entièrement dans mx_physics ; ici on ne fait que lire les commandes,
// avancer la sim à pas fixe et dessiner.
#include "BikeSim.h"
#include "Render.h"

#include "imgui.h"
#include "implot.h"
#include "raylib.h"
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
constexpr double PI_ = 3.14159265358979;

// ---------------------------------------------------------------- Commandes
struct Controls {
    bool triggerArmed[2] = {false, false}; // certaines manettes renvoient 0 tant que la gâchette n'a pas bougé
    double throttleKb = 0, frontKb = 0, rearBrake = 0, steerKb = 0;
    float rearBrakeRampTime = 0.4f; // s pour serrer à fond le frein AR (bouton tout-ou-rien)
    float steerSensitivity = 3.0f;  // stick x sensibilité : à 3 le virage le plus serré est atteint au tiers du stick

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
        in.steer = clampd((std::abs(steer) > std::abs(steerKb) ? steer : steerKb) * steerSensitivity, -1, 1);
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

} // namespace

int main() {
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(1600, 900, "MX Proto - Etape 2 (3D)");
    SetTargetFPS(144); // sans limite, la carte graphique tourne à vide à 1000+ images/s
    SetExitKey(KEY_NULL);

    rlImGuiBeginInitImGui();
    ImGui::StyleColorsDark();
    if (FileExists("C:/Windows/Fonts/segoeui.ttf")) // police avec accents
        ImGui::GetIO().Fonts->AddFontFromFileTTF("C:/Windows/Fonts/segoeui.ttf", 17.0f);
    rlImGuiEndInitImGui();
    ImPlot::CreateContext();

    const Terrain track = Terrain::makeTestTrack();
    Renderer renderer;
    renderer.init(track);
    BikeSim sim;
    std::string spawnName = "Départ";
    sim.reset(BikeParams{}, track, track.spawn(spawnName));

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

    auto doReset = [&](const std::string& name) {
        spawnName = name;
        const Spawn& at = track.spawn(name);
        sim.reset(sim.params(), track, at);
        hist.clear();
        accum = 0;
        camYaw = at.yaw;
    };

    while (!WindowShouldClose()) {
        const double frameDt = GetFrameTime();
        const bool kb = !ImGui::GetIO().WantCaptureKeyboard;

        if (kb) {
            if (IsKeyPressed(KEY_R)) doReset(spawnName);
            const auto& spawns = track.spawns();
            for (int k = 0; k < int(spawns.size()) && k < 9; ++k)
                if (IsKeyPressed(KeyboardKey(KEY_ONE + k))) doReset(spawns[k].name);
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
            camYaw += std::remainder(yaw - camYaw, 2 * PI_) * (1 - std::exp(-2.5 * frameDt));
        }
        const Vec3 back{-std::cos(camYaw), 0, std::sin(camYaw)}, right{std::sin(camYaw), 0, std::cos(camYaw)};
        Camera3D camera{};
        camera.target = V(camTarget + Vec3{0, 0.4, 0});
        camera.position = V(sideView ? camTarget + right * camDist + Vec3{0, 0.5, 0} : camTarget + back * camDist + Vec3{0, 0.9 + 0.2 * camDist, 0});
        camera.up = {0, 1, 0};
        camera.fovy = 55;
        camera.projection = CAMERA_PERSPECTIVE;

        BeginDrawing();
        ClearBackground(Renderer::kSky);
        BeginMode3D(camera);
        renderer.drawWorld(camera);
        renderer.drawShadow(sim, track);
        renderer.drawBike(sim, showForces);
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
        {
            std::string keys = "R recommencer (" + spawnName + ")  ";
            for (size_t k = 0; k < track.spawns().size(); ++k) keys += std::to_string(k + 1) + " " + track.spawns()[k].name + "  ";
            ImGui::TextDisabled("%s", keys.c_str());
        }
        ImGui::TextDisabled("C vue côté  P pause  T ralenti  F1 panneau  molette zoom");
        ImGui::End();

        if (showPanel) {
            ImGui::SetNextWindowPos({float(GetScreenWidth() - 430), 10}, ImGuiCond_FirstUseEver);
            ImGui::SetNextWindowSize({420, float(GetScreenHeight() - 20)}, ImGuiCond_FirstUseEver);
            ImGui::Begin("Réglages (F1)");
            if (ImGui::Button("Valeurs par défaut")) {
                sim.params() = BikeParams{};
                doReset(spawnName);
                status = "Valeurs par défaut";
            }
            ImGui::InputText("Fichier", presetPath, sizeof presetPath);
            if (ImGui::Button("Sauver preset")) status = savePreset(sim.params(), presetPath) ? "Preset sauvé" : "Erreur d'écriture";
            ImGui::SameLine();
            if (ImGui::Button("Charger preset")) {
                BikeParams p = sim.params();
                if (loadPreset(p, presetPath)) {
                    sim.params() = p;
                    doReset(spawnName);
                    status = "Preset chargé";
                } else {
                    status = "Fichier introuvable";
                }
            }
            if (!status.empty()) ImGui::TextDisabled("%s", status.c_str());
            ImGui::Checkbox("Vecteurs de force aux contacts", &showForces);
            ImGui::SliderFloat("Rampe frein AR (LB)", &controls.rearBrakeRampTime, 0.05f, 1.5f, "%.2f s");
            ImGui::SliderFloat("Sensibilité direction", &controls.steerSensitivity, 1.0f, 5.0f, "x %.1f");

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
                if (i.needsReset && ImGui::IsItemDeactivatedAfterEdit()) doReset(spawnName);
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

    renderer.unload();
    ImPlot::DestroyContext();
    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
