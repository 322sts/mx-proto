#include "Render.h"

#include "raymath.h"
#include "rlgl.h"

#include <algorithm>
#include <cmath>

namespace mx {

namespace {

// Éclairage : un soleil + lumière du ciel (hémisphère), brume qui tire vers la couleur du ciel
// avec la distance. Indispensable pour lire les volumes et juger les distances.
const char* kVertexShader = R"(#version 330
in vec3 vertexPosition;
in vec3 vertexNormal;
in vec4 vertexColor;
uniform mat4 mvp;
uniform mat4 matModel;
uniform mat4 matNormal;
out vec3 fragPos;
out vec3 fragNormal;
out vec4 fragColor;
void main() {
    fragPos = vec3(matModel * vec4(vertexPosition, 1.0));
    fragNormal = normalize(vec3(matNormal * vec4(vertexNormal, 0.0)));
    fragColor = vertexColor;
    gl_Position = mvp * vec4(vertexPosition, 1.0);
})";
const char* kFragmentShader = R"(#version 330
in vec3 fragPos;
in vec3 fragNormal;
in vec4 fragColor;
uniform vec4 colDiffuse;
uniform vec3 viewPos;
uniform vec3 skyColor;
out vec4 finalColor;
void main() {
    vec3 n = normalize(fragNormal);
    vec3 sun = normalize(vec3(0.45, 1.0, 0.25));
    float diffuse = max(dot(n, sun), 0.0);
    float sky = 0.5 + 0.5 * n.y;
    vec3 base = (fragColor * colDiffuse).rgb;
    vec3 c = base * (0.55 * sky + 0.6 * diffuse);
    float fog = 1.0 - exp(-length(fragPos - viewPos) * 0.0075);
    finalColor = vec4(mix(c, skyColor, clamp(fog, 0.0, 0.8)), 1.0);
})";

Vector3 V(Vec3 v) { return {float(v.x), float(v.y), float(v.z)}; }
Matrix rot(Quat q) { return QuaternionToMatrix({float(q.x), float(q.y), float(q.z), float(q.w)}); }
Matrix mul(Matrix a, Matrix b) { return MatrixMultiply(a, b); } // a d'abord, puis b

// Bruit déterministe (grain du sol, position des arbres).
double hash(int x, int y) {
    unsigned h = unsigned(x) * 374761393u + unsigned(y) * 668265263u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return double(h ^ (h >> 16)) / 4294967295.0;
}
double valueNoise(double x, double y) {
    const int ix = int(std::floor(x)), iy = int(std::floor(y));
    const double fx = x - ix, fy = y - iy, sx = fx * fx * (3 - 2 * fx), sy = fy * fy * (3 - 2 * fy);
    const double a = hash(ix, iy), b = hash(ix + 1, iy), c = hash(ix, iy + 1), d = hash(ix + 1, iy + 1);
    return a + (b - a) * sx + (c - a) * sy + (a - b - c + d) * sx * sy;
}

Color lerp(Color a, Color b, double t) {
    t = clampd(t, 0, 1);
    return {(unsigned char)(a.r + (b.r - a.r) * t), (unsigned char)(a.g + (b.g - a.g) * t), (unsigned char)(a.b + (b.b - a.b) * t), 255};
}
Color scale(Color c, double k) {
    return {(unsigned char)clampd(c.r * k, 0, 255), (unsigned char)clampd(c.g * k, 0, 255), (unsigned char)clampd(c.b * k, 0, 255), c.a};
}

// Distance à la ligne médiane du circuit, sur une grille de 1 m (pour les couleurs et le décor).
struct TrackDistance {
    double x0, z0;
    int nx, nz;
    std::vector<float> d;
    explicit TrackDistance(const Terrain& t) : x0(t.xMin()), z0(t.zMin()) {
        nx = int(t.xMax() - t.xMin()) + 1;
        nz = int(t.zMax() - t.zMin()) + 1;
        d.assign(size_t(nx) * nz, 1e6f);
        const auto& path = t.trackPath();
        for (int iz = 0; iz < nz; ++iz)
            for (int ix = 0; ix < nx; ++ix) {
                float best = 1e6f;
                for (size_t i = 0; i < path.size(); i += 5) best = std::min(best, float(std::hypot(x0 + ix - path[i].x, z0 + iz - path[i].z)));
                d[size_t(iz) * nx + ix] = best;
            }
    }
    double at(double x, double z) const {
        const double u = clampd(x - x0, 0, nx - 1.001), w = clampd(z - z0, 0, nz - 1.001);
        const int i = int(u), j = int(w);
        const double fu = u - i, fw = w - j;
        auto g = [&](int a, int b) { return double(d[size_t(b) * nx + a]); };
        return (g(i, j) * (1 - fu) + g(i + 1, j) * fu) * (1 - fw) + (g(i, j + 1) * (1 - fu) + g(i + 1, j + 1) * fu) * fw;
    }
};

} // namespace

void Renderer::init(const Terrain& t) {
    shader_ = LoadShaderFromMemory(kVertexShader, kFragmentShader);
    locViewPos_ = GetShaderLocation(shader_, "viewPos");
    locSky_ = GetShaderLocation(shader_, "skyColor");
    const float sky[3] = {kSky.r / 255.f, kSky.g / 255.f, kSky.b / 255.f};
    SetShaderValue(shader_, locSky_, sky, SHADER_UNIFORM_VEC3);
    material_ = LoadMaterialDefault();
    material_.shader = shader_;
    meshes_[Cube] = GenMeshCube(1, 1, 1);
    meshes_[Cylinder] = GenMeshCylinder(1, 1, 14);
    meshes_[Sphere] = GenMeshSphere(1, 12, 16);

    // ---- Terrain : blocs de 200 x 200 mailles de 25 cm (indices 16 bits de raylib), normales lisses.
    const TrackDistance dist(t);
    const double cell = 0.25;
    const int NX = int((t.xMax() - t.xMin()) / cell), NZ = int((t.zMax() - t.zMin()) / cell), C = 200;
    const Color dirt{138, 100, 62, 255}, darkDirt{112, 80, 50, 255}, grass{92, 128, 58, 255}, dryGrass{128, 140, 74, 255};
    for (int cz = 0; cz < NZ; cz += C) {
        for (int cx = 0; cx < NX; cx += C) {
            const int nx = std::min(C, NX - cx) + 1, nz = std::min(C, NZ - cz) + 1;
            Mesh m{};
            m.vertexCount = nx * nz;
            m.triangleCount = (nx - 1) * (nz - 1) * 2;
            m.vertices = static_cast<float*>(MemAlloc(sizeof(float) * 3 * m.vertexCount));
            m.normals = static_cast<float*>(MemAlloc(sizeof(float) * 3 * m.vertexCount));
            m.colors = static_cast<unsigned char*>(MemAlloc(4 * m.vertexCount));
            m.indices = static_cast<unsigned short*>(MemAlloc(sizeof(unsigned short) * 3 * m.triangleCount));
            for (int j = 0; j < nz; ++j) {
                for (int i = 0; i < nx; ++i) {
                    const int k = j * nx + i;
                    const double x = t.xMin() + (cx + i) * cell, z = t.zMin() + (cz + j) * cell;
                    const Vec3 n = t.normal(x, z);
                    m.vertices[3 * k] = float(x);
                    m.vertices[3 * k + 1] = float(t.height(x, z));
                    m.vertices[3 * k + 2] = float(z);
                    m.normals[3 * k] = float(n.x);
                    m.normals[3 * k + 1] = float(n.y);
                    m.normals[3 * k + 2] = float(n.z);
                    // Terre sur la piste et ses flancs, herbe autour ; taches + grain pour sentir la vitesse.
                    const double dt = dist.at(x, z);
                    const Color ground = lerp(lerp(dirt, darkDirt, valueNoise(x * 0.4, z * 0.4)), lerp(grass, dryGrass, valueNoise(x * 0.15, z * 0.15)),
                                              (dt - Terrain::kTrackHalfWidth - 1.5) / 3.0);
                    const Color c = scale(ground, 0.9 + 0.2 * hash(cx + i, cz + j));
                    m.colors[4 * k] = c.r;
                    m.colors[4 * k + 1] = c.g;
                    m.colors[4 * k + 2] = c.b;
                    m.colors[4 * k + 3] = 255;
                }
            }
            int q = 0;
            for (int j = 0; j + 1 < nz; ++j) {
                for (int i = 0; i + 1 < nx; ++i) {
                    const unsigned short a = (unsigned short)(j * nx + i), b = (unsigned short)(a + 1), c = (unsigned short)(a + nx), d = (unsigned short)(c + 1);
                    // Sens antihoraire vu du dessus (normale vers +y).
                    for (unsigned short v : {a, c, b, b, c, d}) m.indices[q++] = v;
                }
            }
            UploadMesh(&m, false);
            Model model = LoadModelFromMesh(m);
            model.materials[0].shader = shader_;
            chunks_.push_back(model);
        }
    }

    // ---- Décor statique : piquets de bord de piste, bottes de paille dans les virages, grille de départ, arbres.
    auto groundAt = [&](double x, double z) { return Vec3{x, t.height(x, z), z}; };
    const auto& path = t.trackPath();
    for (size_t i = 0; i < path.size(); i += 50) { // tous les 5 m
        const TrackPoint& p = path[i];
        const Vec3 right{-std::sin(p.heading), 0, std::cos(p.heading)};
        for (double side : {-1.0, 1.0}) {
            const Vec3 g = groundAt(p.x + right.x * side * 4.8, p.z + right.z * side * 4.8);
            const Color c = (i / 50) % 2 ? Color{240, 240, 240, 255} : Color{240, 110, 20, 255};
            decor_.push_back({Cylinder, mul(MatrixScale(0.035f, 0.9f, 0.035f), MatrixTranslate(float(g.x), float(g.y), float(g.z))), c});
        }
    }
    for (size_t i = 0; i < path.size(); i += 15) { // bottes de paille côté extérieur des virages
        const TrackPoint& p = path[i];
        if (std::abs(p.curvature) < 0.01) continue;
        const double side = p.curvature > 0 ? -1 : 1, d = 9.6;
        const Vec3 right{-std::sin(p.heading), 0, std::cos(p.heading)};
        const Vec3 g = groundAt(p.x + right.x * side * d, p.z + right.z * side * d);
        const Matrix m = mul(mul(MatrixScale(1.3f, 0.5f, 0.55f), MatrixRotateY(float(-p.heading))), MatrixTranslate(float(g.x), float(g.y + 0.25), float(g.z)));
        decor_.push_back({Cube, m, {214, 180, 90, 255}});
    }
    { // grille de départ : poutre basse en travers de la piste + poteaux
        const TrackPoint& p = path[20];
        const Vec3 right{-std::sin(p.heading), 0, std::cos(p.heading)};
        for (double side : {-1.0, 1.0}) {
            const Vec3 g = groundAt(p.x + right.x * side * 4.4, p.z + right.z * side * 4.4);
            decor_.push_back({Cube, mul(MatrixScale(0.15f, 1.2f, 0.15f), MatrixTranslate(float(g.x), float(g.y + 0.6), float(g.z))), {80, 80, 90, 255}});
        }
        const Vec3 g = groundAt(p.x, p.z);
        decor_.push_back({Cube, mul(mul(MatrixScale(0.12f, 0.12f, 8.8f), MatrixRotateY(float(-p.heading))), MatrixTranslate(float(g.x), float(g.y + 1.1), float(g.z))), {200, 30, 40, 255}});
    }
    for (int i = 0, placed = 0; i < 4000 && placed < 260; ++i) { // arbres dans le pré, loin de la piste
        const double x = t.xMin() + hash(i, 7) * (t.xMax() - t.xMin()), z = t.zMin() + hash(i, 13) * (t.zMax() - t.zMin());
        if (dist.at(x, z) < 15) continue;
        ++placed;
        const double hgt = 3 + 3 * hash(i, 29), y = t.height(x, z);
        decor_.push_back({Cylinder, mul(MatrixScale(0.18f, float(hgt * 0.45), 0.18f), MatrixTranslate(float(x), float(y), float(z))), {90, 65, 40, 255}});
        const float r = float(1.2 + 1.0 * hash(i, 31));
        decor_.push_back({Sphere, mul(MatrixScale(r, float(hgt * 0.42), r), MatrixTranslate(float(x), float(y + hgt * 0.62), float(z))),
                          lerp({50, 95, 40, 255}, {85, 120, 50, 255}, hash(i, 41))});
    }
}

void Renderer::unload() {
    for (Model& m : chunks_) UnloadModel(m);
    chunks_.clear();
    for (Mesh& m : meshes_) UnloadMesh(m);
    UnloadShader(shader_);
}

void Renderer::draw(Prim prim, const Matrix& m, Color c) {
    material_.maps[MATERIAL_MAP_DIFFUSE].color = c;
    DrawMesh(meshes_[prim], material_, m);
}

void Renderer::tube(Vec3 a, Vec3 b, double r, Color c) {
    const Vec3 d = b - a;
    const double len = length(d);
    if (len < 1e-6) return;
    const Matrix align = QuaternionToMatrix(QuaternionFromVector3ToVector3({0, 1, 0}, V(d / len)));
    draw(Cylinder, mul(mul(MatrixScale(float(r), float(len), float(r)), align), MatrixTranslate(float(a.x), float(a.y), float(a.z))), c);
}

void Renderer::ball(Vec3 c, double r, Color col) {
    draw(Sphere, mul(MatrixScale(float(r), float(r), float(r)), MatrixTranslate(float(c.x), float(c.y), float(c.z))), col);
}

// Boîte de taille `size` centrée en `center` dans le repère (q, origin), tournée de `local` sur elle-même.
void Renderer::box(Quat q, Vec3 origin, Vec3 center, Vec3 size, Color c, Quat local) {
    const Matrix m = mul(mul(mul(mul(MatrixScale(float(size.x), float(size.y), float(size.z)), rot(local)), MatrixTranslate(float(center.x), float(center.y), float(center.z))), rot(q)),
                         MatrixTranslate(float(origin.x), float(origin.y), float(origin.z)));
    draw(Cube, m, c);
}

void Renderer::drawWorld(const Camera3D& camera) {
    SetShaderValue(shader_, locViewPos_, &camera.position, SHADER_UNIFORM_VEC3);
    for (const Model& m : chunks_) DrawModel(m, {0, 0, 0}, 1, WHITE);
    for (const Instance& i : decor_) draw(i.prim, i.m, i.c);
}

// Ombre portée (soleil presque au zénith) : ellipse sombre plaquée sur le sol sous la moto,
// plus pâle quand la moto monte. Le meilleur repère pour juger la hauteur en l'air.
void Renderer::drawShadow(const BikeSim& sim, const Terrain& t) {
    const BikeState& s = sim.state();
    const Vec3 c = (s.front.center + s.rear.center) * 0.5;
    Vec3 f = s.front.center - s.rear.center;
    f.y = 0;
    f = length(f) > 0.1 ? normalize(f) : Vec3{1, 0, 0};
    const Vec3 r{-f.z, 0, f.x};
    const double height = c.y - t.height(c.x, c.z);
    const unsigned char alpha = (unsigned char)(110 * clampd(1.2 - height / 8.0, 0.2, 1.0));
    const double a = 1.05, b = 0.32;
    rlDisableBackfaceCulling();
    rlBegin(RL_TRIANGLES);
    rlColor4ub(20, 15, 10, alpha);
    // Ellipse découpée en anneaux pour épouser le relief (un éventail plat passerait sous une bosse).
    auto pt = [&](double ang, double k) {
        const double x = c.x + (f.x * a * std::cos(ang) + r.x * b * std::sin(ang)) * k, z = c.z + (f.z * a * std::cos(ang) + r.z * b * std::sin(ang)) * k;
        rlVertex3f(float(x), float(t.height(x, z) + 0.07), float(z));
    };
    const int n = 24, rings = 4;
    for (int j = 0; j < rings; ++j) {
        const double k0 = double(j) / rings, k1 = double(j + 1) / rings;
        for (int i = 0; i < n; ++i) {
            const double a0 = i * 2 * PI / n, a1 = (i + 1) * 2 * PI / n;
            pt(a0, k0); pt(a1, k1); pt(a0, k1);
            pt(a0, k0); pt(a1, k0); pt(a1, k1);
        }
    }
    rlEnd();
    rlDrawRenderBatchActive();
    rlEnableBackfaceCulling();
}

void Renderer::drawBike(const BikeSim& sim, bool forces) {
    const BikeState& s = sim.state();
    const BikeLayout& L = sim.layout();
    const BikeParams& p = const_cast<BikeSim&>(sim).params();
    const RigidBody3D &C = s.chassis, &F = s.fork, &S = s.swingarm, &R = s.rider;
    const Color team{205, 30, 40, 255}, white{235, 235, 238, 255}, black{25, 25, 28, 255}, alu{175, 178, 185, 255};
    const Color frameCol{150, 152, 160, 255}, engineCol{60, 62, 68, 255}, gold{215, 170, 50, 255};
    auto D = [&](double x, double y, double z) { return L.rearAxle + Vec3{x, y, z}; }; // point de dessin depuis l'axe AR (moto détendue)
    const Vec3 Z = C.dirToWorld(kZ);

    // ---- Roues : pneu à crampons (tore épais), jante, rayons, moyeu ; disque avant à gauche.
    auto wheel = [&](const WheelState& w, const RigidBody3D& carrier, bool front) {
        const Vec3 a = carrier.dirToWorld(kZ);
        const double Rw = w.radius;
        const int n = 28;
        auto ring = [&](double ang, double r) { return w.center + rotate(carrier.q, Vec3{std::cos(ang), std::sin(ang), 0}) * r; };
        for (int i = 0; i < n; ++i) {
            const double a0 = i * 2 * PI / n, a1 = (i + 1) * 2 * PI / n;
            tube(ring(a0, Rw - 0.055), ring(a1, Rw - 0.055), 0.055, black);
            tube(ring(a0, Rw - 0.115), ring(a1, Rw - 0.115), 0.013, alu);
        }
        const Quat spin = carrier.q * axisAngle(kZ, w.angle);
        for (int i = 0; i < 12; ++i) {
            const Vec3 dir = rotate(spin, Vec3{std::cos(i * PI / 6), std::sin(i * PI / 6), 0});
            tube(w.center + a * (i % 2 ? 0.04 : -0.04), w.center + dir * (Rw - 0.115), 0.003, alu);
        }
        tube(w.center - a * 0.07, w.center + a * 0.07, 0.035, alu);
        if (front) tube(w.center - a * 0.075, w.center - a * 0.068, 0.13, {120, 120, 125, 255});
    };
    wheel(s.rear, S, false);
    wheel(s.front, F, true);

    // ---- Bras oscillant, amortisseur, chaîne.
    const Vec3 pivot = C.toWorld(L.pivot), rearAxle = S.toWorld(L.swingAxle);
    for (double side : {-1.0, 1.0}) tube(pivot + Z * (side * 0.085), rearAxle + Z * (side * 0.085), 0.03, alu);
    const Vec3 shockTop = C.toWorld(L.shockTop), shockLow = pivot + (rearAxle - pivot) * 0.3;
    tube(shockLow, shockTop, 0.032, {230, 200, 30, 255});
    tube(shockTop, shockTop + (shockLow - shockTop) * 0.4, 0.04, black);
    const Vec3 cs = C.toWorld(L.countershaft) - Z * 0.11;
    tube(rearAxle - Z * 0.11 + C.dirToWorld(kY) * p.rearSprocketRadius, cs + C.dirToWorld(kY) * p.frontSprocketRadius, 0.008, black);
    tube(rearAxle - Z * 0.105, rearAxle - Z * 0.115, p.rearSprocketRadius, alu);

    // ---- Cadre, moteur, réservoir, selle, plastiques, échappement (repère châssis).
    const Quat q = C.q;
    const Vec3 o = C.pos;
    const Vec3 head = C.toWorld(L.steeringHead), seat = C.toWorld(L.seat), skid = C.toWorld(L.skidPlate);
    for (auto [a, b] : {std::pair{head, skid}, {head, C.toWorld(D(0.45, 0.70, 0))}, {C.toWorld(D(0.45, 0.70, 0)), pivot}, {skid, pivot}})
        for (double side : {-1.0, 1.0}) tube(a + Z * (side * 0.05), b + Z * (side * 0.07), 0.018, frameCol);
    box(q, o, D(0.80, 0.20, 0), {0.36, 0.30, 0.22}, engineCol);                                 // carter
    box(q, o, D(0.92, 0.42, 0), {0.18, 0.16, 0.18}, engineCol, axisAngle(kZ, -0.25));          // culasse
    box(q, o, D(0.82, 0.00, 0), {0.42, 0.03, 0.22}, alu);                                       // sabot
    box(q, o, D(0.88, 0.80, 0), {0.40, 0.17, 0.30}, team);                                      // réservoir
    for (double side : {-1.0, 1.0}) {
        box(q, o, D(0.98, 0.70, side * 0.17), {0.32, 0.27, 0.035}, team, axisAngle(kZ, 0.35)); // ouïes de radiateur
        box(q, o, D(1.03, 0.58, side * 0.12), {0.05, 0.28, 0.16}, {80, 80, 85, 255});         // radiateurs
        box(q, o, D(0.20, 0.55, side * 0.14), {0.36, 0.22, 0.025}, white);                     // plaques latérales
        box(q, o, D(0.60, 0.10, side * 0.20), {0.09, 0.025, 0.07}, alu);                       // repose-pieds
    }
    box(q, o, D(0.55, 0.735, 0), {0.72, 0.07, 0.24}, black, axisAngle(kZ, -0.05));             // selle
    box(q, o, D(0.02, 0.67, 0), {0.62, 0.025, 0.22}, white, axisAngle(kZ, 0.12));               // garde-boue AR
    // Échappement : collecteur qui part de la culasse, passe devant le moteur, silencieux à droite.
    const Vec3 e0 = C.toWorld(D(1.02, 0.40, 0.05)), e1 = C.toWorld(D(1.08, 0.16, 0.10)), e2 = C.toWorld(D(0.70, 0.30, 0.16));
    const Vec3 e3 = C.toWorld(D(0.45, 0.45, 0.17)), e4 = C.toWorld(D(-0.12, 0.62, 0.16));
    tube(e0, e1, 0.022, alu);
    tube(e1, e2, 0.024, alu);
    tube(e2, e3, 0.026, alu);
    tube(e3, e4, 0.055, {200, 200, 205, 255});

    // ---- Direction : tés, guidon, plaque numéro, garde-boue AV tournent autour de l'axe avec le bloc avant.
    const Vec3 u = C.dirToWorld(L.forkAxis), axisPt = C.toWorld(L.steerAxisPoint);
    const Quat steer = axisAngle(u, s.steerAngle);
    const Quat qs = steer * C.q;
    const Vec3 os = axisPt + rotate(steer, C.pos - axisPt);
    auto SW = [&](Vec3 local) { return os + rotate(qs, local); }; // point du repère châssis, braqué
    const Quat rake = axisAngle(kZ, p.rakeAngle);
    const Vec3 offset = L.frontAxle - L.steerAxisPoint;
    box(qs, os, L.steeringHead + offset * 0.5, {0.07, 0.05, 0.26}, alu, rake);                       // té du haut
    box(qs, os, L.steeringHead + offset * 0.5 - L.forkAxis * 0.2, {0.08, 0.06, 0.26}, alu, rake);    // té du bas
    const Vec3 barsMid = SW(L.bars), barZ = rotate(qs, kZ);
    tube(barsMid - barZ * p.barWidth, barsMid + barZ * p.barWidth, 0.014, alu);
    for (double side : {-1.0, 1.0}) tube(barsMid + barZ * (side * (p.barWidth - 0.13)), barsMid + barZ * (side * p.barWidth), 0.018, black);
    tube(SW(L.steeringHead), barsMid, 0.02, alu);
    box(qs, os, L.steeringHead + L.forkAxis * -0.12 + Vec3{0.13, 0, 0}, {0.03, 0.22, 0.27}, white, rake); // plaque numéro
    box(qs, os, L.steeringHead + L.forkAxis * -0.30 + Vec3{0.17, 0, 0}, {0.55, 0.025, 0.17}, white, axisAngle(kZ, -0.15)); // garde-boue AV
    // Fourche : fourreaux (bloc avant) + tubes plongeurs jusqu'au té du bas.
    const Vec3 axle = F.toWorld(L.forkAxle), Zf = F.dirToWorld(kZ), uf = F.dirToWorld(L.forkAxis);
    for (double side : {-1.0, 1.0}) {
        const Vec3 low = axle + Zf * (side * 0.1);
        tube(low, low + uf * 0.48, 0.037, black);
        tube(low + uf * 0.44, SW(L.steeringHead + offset + kZ * (side * 0.1)), 0.027, gold);
    }

    // ---- Pilote : bottes sur les repose-pieds, genoux et coudes en cinématique inverse, buste, casque.
    const Color jersey = team, pants = white, boots{235, 235, 235, 255}, skin = black;
    const Vec3 hip = R.toWorld(L.riderHip), shoulder = R.toWorld(L.riderShoulder), headR = R.toWorld(L.riderHead);
    const Vec3 Zr = R.dirToWorld(kZ), fwd = C.dirToWorld(kX), up = C.dirToWorld(kY);
    auto ik = [](Vec3 a, Vec3 b, double l1, double l2, Vec3 bend) {
        const Vec3 d = b - a;
        const double len = std::max(length(d), 1e-6), Lc = std::min(len, l1 + l2 - 1e-3);
        const Vec3 un = d / len;
        const double x = (l1 * l1 - l2 * l2 + Lc * Lc) / (2 * Lc);
        const Vec3 n = normalize(bend - un * dot(bend, un));
        return a + un * x + n * std::sqrt(std::max(0.0, l1 * l1 - x * x));
    };
    for (double side : {-1.0, 1.0}) {
        const Vec3 h = hip + Zr * (side * 0.11), sh = shoulder + Zr * (side * 0.19);
        const Vec3 foot = C.toWorld(L.pegs) + Z * (side * p.pegWidth) + up * 0.04;
        const Vec3 grip = barsMid + barZ * (side * (p.barWidth - 0.07));
        const Vec3 knee = ik(h, foot, 0.46, 0.46, fwd + Z * (side * 0.15)), elbow = ik(sh, grip, 0.30, 0.32, Zr * (side * 0.6) - up - fwd * 0.2);
        tube(h, knee, 0.08, pants);
        ball(knee, 0.075, team);
        tube(knee, foot, 0.068, boots);
        box(q, o, rotateInv(q, foot - o) + Vec3{0.03, -0.02, 0}, {0.28, 0.09, 0.11}, boots); // pied de botte
        tube(sh, elbow, 0.055, jersey);
        tube(elbow, grip, 0.048, jersey);
        ball(grip, 0.045, skin);
    }
    const Vec3 torsoDir = rotateInv(R.q, shoulder - hip);
    const Quat torsoRot = normalize(Quat{1 + torsoDir.y / length(torsoDir), torsoDir.z / length(torsoDir), 0, -torsoDir.x / length(torsoDir)});
    box(R.q, R.pos, (L.riderHip + L.riderShoulder) * 0.5, {0.24, length(torsoDir) + 0.1, 0.40}, jersey, torsoRot); // buste
    ball(hip, 0.15, pants);                                                                                         // bassin
    ball(headR, 0.145, white);                                                                                      // casque
    box(R.q, R.pos, L.riderHead + Vec3{0.12, -0.06, 0}, {0.08, 0.06, 0.16}, team);                                  // mentonnière
    box(R.q, R.pos, L.riderHead + Vec3{0.06, 0.12, 0}, {0.22, 0.02, 0.2}, team, axisAngle(kZ, -0.2));               // visière
    box(R.q, R.pos, L.riderHead + Vec3{0.11, 0.02, 0}, {0.06, 0.06, 0.2}, {40, 40, 45, 255});                       // masque

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

} // namespace mx
