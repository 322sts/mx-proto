// Jeu de test jouable : rendu raylib, manette Xbox / clavier, réglages ImGui, courbes ImPlot.
// La physique est entièrement dans mx_physics ; ici on ne fait que lire les commandes,
// avancer la sim à pas fixe et dessiner.
#include "BikeSim.h"

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

Vector2 V(Vec2 v) { return {float(v.x), float(-v.y)}; } // monde (y vers le haut) -> raylib (y vers le bas)
Vec2 fwdOf(double angle) { return rotate({1, 0}, angle); }

// ---------------------------------------------------------------- Commandes
struct Controls {
    bool triggerArmed[2] = {false, false}; // certaines manettes renvoient 0 tant que la gâchette n'a pas bougé
    double throttleKb = 0, frontKb = 0, rearBrake = 0;
    float rearBrakeRampTime = 0.4f; // s pour serrer à fond le frein AR (bouton tout-ou-rien)

    double trigger(int i, GamepadAxis axis) {
        const float v = GetGamepadAxisMovement(0, axis); // -1 relâchée .. +1 enfoncée
        if (v < -0.5f) triggerArmed[i] = true;
        return triggerArmed[i] ? clampd((v + 1.0) * 0.5, 0, 1) : 0.0;
    }
    static double ramp(double v, bool held, double dt, double t) { return clampd(v + (held ? dt / t : -4 * dt / t), 0, 1); }

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
        // Pilote : stick droit (ou gauche) vertical, haut = vers l'avant.
        double lean = 0;
        if (pad) {
            const double r = -GetGamepadAxisMovement(0, GAMEPAD_AXIS_RIGHT_Y);
            const double l = -GetGamepadAxisMovement(0, GAMEPAD_AXIS_LEFT_Y);
            lean = std::abs(r) > std::abs(l) ? r : l;
            if (std::abs(lean) < 0.12) lean = 0;
        }
        if (keyboard && (IsKeyDown(KEY_RIGHT) || IsKeyDown(KEY_D))) lean = 1;
        if (keyboard && (IsKeyDown(KEY_LEFT) || IsKeyDown(KEY_A))) lean = -1;
        in.riderLean = lean;
        return in;
    }
};

// ---------------------------------------------------------------- Courbes
struct History {
    static constexpr size_t kMax = 3000; // ~5 s à 144 FPS
    std::vector<float> t, fork, rear, loadF, loadR, slipF, slipR, kmh;
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
void drawSpring(Vec2 a, Vec2 b, double width, int coils, Color c) {
    const Vec2 d = b - a, n = perp(d / std::max(length(d), 1e-6));
    Vec2 prev = a;
    for (int i = 1; i <= 2 * coils; ++i) {
        Vec2 pt = a + d * (double(i) / (2 * coils)) + n * ((i % 2 ? 1 : -1) * width);
        if (i == 2 * coils) pt = b;
        DrawLineEx(V(prev), V(pt), 0.012f, c);
        prev = pt;
    }
}

// Articulation (genou/coude) pour relier deux points avec deux segments de longueurs l1, l2.
Vec2 ik(Vec2 a, Vec2 b, double l1, double l2, Vec2 bend) {
    const Vec2 d = b - a;
    const double len = std::max(length(d), 1e-6), L = std::min(len, l1 + l2 - 1e-3);
    const Vec2 u = d / len;
    const double x = (l1 * l1 - l2 * l2 + L * L) / (2 * L);
    Vec2 n = perp(u);
    if (dot(n, bend) < 0) n = -n;
    return a + u * x + n * std::sqrt(std::max(0.0, l1 * l1 - x * x));
}

void drawTerrain(const Terrain& t, double x0, double x1) {
    const Color dirt{139, 100, 62, 255}, top{96, 68, 40, 255};
    const double step = 0.1;
    for (double x = std::floor(x0 / step) * step; x < x1; x += step) {
        const Vec2 a{x, t.height(x)}, b{x + step, t.height(x + step)};
        DrawTriangle(V(a), V({a.x, -30}), V(b), dirt);
        DrawTriangle(V(b), V({a.x, -30}), V({b.x, -30}), dirt);
        DrawLineEx(V(a), V(b), 0.05f, top);
    }
    for (double x = std::ceil(x0 / 10) * 10; x < x1; x += 10) // repères tous les 10 m
        DrawLineEx(V({x, t.height(x)}), V({x, t.height(x) + 0.25}), 0.03f, {60, 60, 60, 160});
}

void drawWheel(const WheelState& w) {
    DrawRing(V(w.center), float(w.radius - 0.09), float(w.radius), 0, 360, 48, {30, 30, 30, 255});
    DrawRing(V(w.center), float(w.radius - 0.105), float(w.radius - 0.09), 0, 360, 48, {170, 170, 175, 255});
    for (int i = 0; i < 6; ++i) {
        const Vec2 dir = rotate({1, 0}, w.angle + i * 3.14159265 / 3);
        DrawLineEx(V(w.center), V(w.center + dir * (w.radius - 0.1)), 0.012f, {150, 150, 150, 255});
    }
    DrawCircleV(V(w.center), 0.03f, {90, 90, 90, 255});
}

void drawBike(const BikeSim& sim, bool forces) {
    const BikeState& s = sim.state();
    const BikeLayout& L = sim.layout();
    const RigidBody2D &C = s.chassis, &F = s.fork, &S = s.swingarm, &R = s.rider;
    const Vec2 fwd = fwdOf(C.angle), up = perp(fwd);
    const Color frame{220, 90, 20, 255}, metal{120, 120, 130, 255}, gold{210, 170, 60, 255};

    const Vec2 rearAxle = S.toWorld(L.swingAxle), frontAxle = F.toWorld(L.forkAxle);
    const Vec2 pivot = C.toWorld(L.pivot), head = C.toWorld(L.steeringHead), seat = C.toWorld(L.seat);
    const Vec2 skid = C.toWorld(L.skidPlate), fender = C.toWorld(L.rearFender), pegs = C.toWorld(L.pegs);
    const Vec2 bars = C.toWorld(L.bars);

    drawWheel(s.rear);
    drawWheel(s.front);

    // Bras oscillant + amortisseur (ressort dessiné : on voit sa compression).
    DrawLineEx(V(pivot), V(rearAxle), 0.05f, metal);
    const Vec2 shockLow = pivot + (rearAxle - pivot) * 0.35, shockTop = C.toWorld(L.shockTop);
    DrawLineEx(V(shockTop), V(shockLow), 0.035f, {60, 60, 60, 255});
    drawSpring(shockTop + (shockLow - shockTop) * 0.1, shockLow, 0.035, 7, {230, 200, 30, 255});

    // Moteur (bloc), cadre.
    const Vec2 e[4] = {C.toWorld(L.skidPlate + Vec2{-0.18, 0.03}), C.toWorld(L.skidPlate + Vec2{0.20, 0.03}),
                       C.toWorld(L.skidPlate + Vec2{0.14, 0.33}), C.toWorld(L.skidPlate + Vec2{-0.16, 0.33})};
    DrawTriangle(V(e[0]), V(e[1]), V(e[2]), {70, 70, 75, 255});
    DrawTriangle(V(e[0]), V(e[2]), V(e[3]), {70, 70, 75, 255});
    for (auto [a, b] : {std::pair{head, skid}, {head, seat}, {seat, fender}, {seat, pivot}, {skid, pivot}})
        DrawLineEx(V(a), V(b), 0.04f, frame);
    DrawLineEx(V(head - fwd * 0.05), V(seat + up * 0.03), 0.09f, {40, 40, 160, 255}); // réservoir/selle
    DrawLineEx(V(pegs - fwd * 0.06), V(pegs + fwd * 0.06), 0.03f, metal);

    // Fourche : tubes fixés au châssis, fourreaux qui coulissent, ressort entre axe et tête.
    const Vec2 u = rotate(L.forkAxis, C.angle), side = perp(u) * -0.06;
    DrawLineEx(V(head), V(head - u * 0.55), 0.045f, gold);
    DrawLineEx(V(frontAxle), V(frontAxle + u * 0.5), 0.06f, {50, 50, 55, 255});
    drawSpring(frontAxle + side, head + side, 0.025, 10, {230, 200, 30, 255});
    DrawLineEx(V(head), V(bars), 0.03f, metal);

    // Pilote : buste, tête, jambes jusqu'aux repose-pieds, bras jusqu'au guidon.
    const Vec2 hip = R.toWorld(L.riderHip), shoulder = R.toWorld(L.riderShoulder), headR = R.toWorld(L.riderHead);
    const Vec2 knee = ik(hip, pegs, 0.46, 0.46, fwd);
    const Vec2 elbow = ik(shoulder, bars, 0.30, 0.32, fwd * -0.3 - up);
    const Color suit{30, 110, 200, 255}, pants{230, 230, 230, 255};
    DrawLineEx(V(hip), V(knee), 0.11f, pants);
    DrawLineEx(V(knee), V(pegs), 0.09f, pants);
    DrawLineEx(V(hip), V(shoulder), 0.18f, suit);
    DrawLineEx(V(shoulder), V(elbow), 0.08f, suit);
    DrawLineEx(V(elbow), V(bars), 0.07f, suit);
    DrawCircleV(V(headR), 0.13f, {240, 240, 240, 255});
    DrawCircleV(V(R.pos), 0.03f, RED); // CdG pilote

    if (forces) {
        for (const WheelState* w : {&s.front, &s.rear}) {
            if (!w->contact) continue;
            const Vec2 t{w->normal.y, -w->normal.x};
            const double k = 1.0 / 2500; // m par N
            DrawLineEx(V(w->contactPoint), V(w->contactPoint + w->normal * (w->normalForce * k)), 0.025f, RED);
            DrawLineEx(V(w->contactPoint), V(w->contactPoint + t * (w->longForce * k)), 0.025f, {0, 160, 255, 255});
        }
    }
}

} // namespace

int main() {
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(1600, 900, "MX Proto - Etape 1 (2D)");
    SetExitKey(KEY_NULL);

    rlImGuiBeginInitImGui();
    ImGui::StyleColorsDark();
    if (FileExists("C:/Windows/Fonts/segoeui.ttf")) // police avec accents
        ImGui::GetIO().Fonts->AddFontFromFileTTF("C:/Windows/Fonts/segoeui.ttf", 17.0f);
    rlImGuiEndInitImGui();
    ImPlot::CreateContext();

    const Terrain track = Terrain::makeTestTrack();
    BikeSim sim;
    double resetX = Terrain::kStartX;
    sim.reset(BikeParams{}, track, resetX);

    Controls controls;
    History hist;
    Vec2 cam = sim.state().chassis.pos;
    float zoom = 110; // pixels par mètre
    double accum = 0;
    bool paused = false, slowmo = false, showPanel = true, showForces = false;
    char presetPath[256] = "presets/mon_reglage.ini";
    std::string status;

    auto doReset = [&](double x) {
        resetX = x;
        sim.reset(sim.params(), track, x);
        hist.clear();
        accum = 0;
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
            if (IsKeyPressed(KEY_F1)) showPanel = !showPanel;
        }
        if (!ImGui::GetIO().WantCaptureMouse) zoom = std::clamp(zoom * (1 + 0.1f * GetMouseWheelMove()), 30.0f, 400.0f);

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

        // Caméra qui suit le châssis avec un lissage exponentiel.
        const BikeState& s = sim.state();
        cam = cam + (s.chassis.pos - cam) * (1 - std::exp(-6.0 * frameDt));
        Camera2D camera{};
        camera.offset = {GetScreenWidth() * 0.45f, GetScreenHeight() * 0.55f};
        camera.target = V(cam);
        camera.zoom = zoom;

        BeginDrawing();
        ClearBackground({170, 205, 235, 255});
        BeginMode2D(camera);
        rlDisableBackfaceCulling();
        const double halfW = GetScreenWidth() / zoom;
        drawTerrain(track, cam.x - halfW, cam.x + halfW);
        drawBike(sim, showForces);
        EndMode2D();

        rlImGuiBegin();
        // Infos toujours visibles.
        ImGui::SetNextWindowPos({10, 10}, ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowBgAlpha(0.6f);
        ImGui::Begin("Infos", nullptr, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse);
        ImGui::Text("%5.1f km/h   %5.0f tr/min", s.speed * 3.6, std::max(0.0, s.engineSpeed) * 9.5493);
        ImGui::Text("Fourche %5.0f mm   Roue AR %5.0f mm   Amortisseur %4.0f mm", s.forkTravel * 1000,
                    s.rearTravel * 1000, s.shockStroke * 1000);
        ImGui::Text("Charge AV %5.0f N   AR %5.0f N", s.front.normalForce, s.rear.normalForce);
        ImGui::Text("Glissement AV %+5.0f %%   AR %+5.0f %%", s.front.slipRatio * 100, s.rear.slipRatio * 100);
        ImGui::ProgressBar(float(s.inputs.throttle), {120, 0}, "gaz");
        ImGui::SameLine();
        ImGui::ProgressBar(float(s.inputs.frontBrake), {120, 0}, "frein AV");
        ImGui::SameLine();
        ImGui::ProgressBar(float(s.inputs.rearBrake), {120, 0}, "frein AR");
        ImGui::ProgressBar(float(0.5 + 0.5 * s.inputs.riderLean), {372, 0}, "pilote  arrière <-> avant");
        if (paused) ImGui::TextColored({1, 0.4f, 0.4f, 1}, "PAUSE (P)");
        if (slowmo) ImGui::TextColored({1, 0.8f, 0.2f, 1}, "RALENTI x0.25 (T)");
        ImGui::TextDisabled(IsGamepadAvailable(0) ? "Manette OK" : "Pas de manette : clavier (flèches/WASD, Espace)");
        ImGui::TextDisabled("R départ  1/2/3 saut 1/saut 2/whoops  P pause  T ralenti  F1 panneau  molette zoom");
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
                plot("Enfoncements", "mm", {{"Fourche", &hist.fork}, {"Roue AR", &hist.rear}});
                plot("Charge pneus", "N", {{"AV", &hist.loadF}, {"AR", &hist.loadR}});
                plot("Glissement pneus", "%", {{"AV", &hist.slipF}, {"AR", &hist.slipR}});
                plot("Vitesse", "km/h", {{"Vitesse", &hist.kmh}});
            }
            ImGui::End();
        }
        rlImGuiEnd();
        EndDrawing();
    }

    ImPlot::DestroyContext();
    rlImGuiShutdown();
    CloseWindow();
    return 0;
}
