#pragma once
// Rendu 3D du jeu de test : terrain éclairé (soleil + brume), décor de bord de piste, ombre et
// moto/pilote détaillés. Uniquement du dessin : aucune physique ici.
#include "BikeSim.h"
#include "raylib.h"

#include <vector>

namespace mx {

class Renderer {
public:
    void init(const Terrain& terrain);
    void unload();
    // À appeler entre BeginMode3D / EndMode3D.
    void drawWorld(const Camera3D& camera);
    void drawShadow(const BikeSim& sim, const Terrain& terrain);
    void drawBike(const BikeSim& sim, bool forces);

    static constexpr Color kSky{168, 200, 230, 255};

private:
    enum Prim { Cube, Cylinder, Sphere };
    struct Instance {
        Prim prim;
        Matrix m;
        Color c;
    };
    void draw(Prim prim, const Matrix& m, Color c);
    void tube(Vec3 a, Vec3 b, double r, Color c);
    void ball(Vec3 c, double r, Color col);
    void box(Quat q, Vec3 origin, Vec3 center, Vec3 size, Color c, Quat local = {});

    Shader shader_{};
    int locViewPos_ = -1, locSky_ = -1;
    Material material_{};
    Mesh meshes_[3]{};
    std::vector<Model> chunks_;
    std::vector<Instance> decor_;
};

} // namespace mx
