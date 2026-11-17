# MX Proto — physique de motocross

Jeu de motocross à physique réaliste et exigeante (esprit MX Simulator). **Rien n'est scripté** : scrub,
whip, wheelie, absorption des bosses doivent émerger de la simulation moto + pilote + sol, jamais
d'animations ou de triches. Le pilote (l'utilisateur) teste **uniquement au ressenti**, à la manette Xbox,
et donne ses retours en langage de pilote. Le projet final ira dans Unreal Engine 5 ; pour l'instant
on construit la physique dans ce prototype léger (compile + relance en quelques secondes).

## Architecture
- `physics/` — bibliothèque C++20 pure, **aucune dépendance** (ni raylib, ni ImGui, ni Unreal). Unités SI partout.
  - `Math2D.h` : Vec2, RigidBody2D (à remplacer pour la 3D).
  - `BikeParams.h/.cpp` : `BikeParams` (tous les réglages, défauts réalistes, unité en commentaire),
    `Inputs`, et `paramTable()` (libellés, bornes stables, unité d'affichage) utilisé par l'UI, les presets et les tests.
  - `Terrain.h/.cpp` : profil 2D échantillonné tous les 5 cm, spline Catmull-Rom (C1), normale, point le plus proche, piste de test.
  - `BikeSim.h/.cpp` : `reset(params, terrain, x)`, `step(inputs, terrain, dt)`, `state()`, `layout()`.
- `sandbox/main.cpp` — jeu de test : raylib (rendu + manette), Dear ImGui via rlImGui (réglages), ImPlot (courbes).
- `tests/` — doctest : terrain lisse, repos sans vibration, accélération, gaz en l'air, chaque curseur à ses bornes.
- Dépendances via FetchContent, versions épinglées : raylib 5.5, imgui 1.93.0, implot 1.1, rlImGui @5c17e70, doctest 2.5.3.

## Modèle physique (étape 1, plan vertical x/y/rotation)
- 4 corps rigides : châssis, fourche (fourreaux + masse roue AV), bras oscillant (+ masse roue AR), pilote.
  Rotation propre de chaque roue = degré de liberté séparé (gaz/frein en l'air font tourner la moto).
- Liaisons rigides par impulsions séquentielles (20 itérations, Baumgarte 0,2) : glissière de fourche, pivot de bras,
  butées de sécurité à débattement + 30 mm. Tout le reste = forces.
- Fourche : ressort préchargé, amortissement compression/détente, cône anti-talonnage k·p².
- Arrière : rapport de biellette linéaire en course roue (2,8 → 1,85), course amortisseur = ∫dx/LR, force roue = F_amorto/LR.
- Pneus : contact au point du sol le plus proche, ressort radial + amortissement + jante ; Fx = μ(glissement)·N,
  courbe terre (pic 0,85 à 20 %, puis décroissance vers 0,65) ; force bornée à celle qui annule le glissement en un pas (stabilité).
- Pilote : corps relié aux repose-pieds et au guidon par ressorts-amortisseurs ; le stick décale les points d'attache
  (±25 cm, vitesse limitée) → vrai transfert de poids.
- Moteur : courbe couple/régime, rapport unique 13:1, embrayage virtuel qui patine sous 4000 tr/min, frein moteur,
  inertie vilebrequin ramenée à la roue. Chaîne : tension = couple roue / bras de levier, tire l'axe AR vers le pignon
  de sortie de boîte (anti-squat qui émerge de la géométrie), réaction sur le cadre au pignon. Freins : frottement implicite roue/porteur (blocage propre).
- Boucle : pas fixe 1 ms (1000 Hz), Euler semi-implicite, accumulateur séparé du rendu, max 50 sous-pas par image.

## Conventions
- x vers l'avant, y vers le haut, angles en rad sens trigo (cabrage = positif). Roue qui avance : spin < 0.
- Commenter la physique (quelle force, quelle formule, pourquoi). Code et commentaires en français.
- Tout nouveau réglage : champ dans `BikeParams` + ligne dans `paramTable()` avec des bornes testées stables.
- Paramètres `needsReset` (géométrie, masses non suspendues) : la moto est reconstruite au relâchement du curseur.
- Pas de fonctionnalité hors de l'étape en cours, pas de nouvelle dépendance sans demander.
- Chaque retour de pilotage → modification physique expliquée + entrée dans `RESSENTI.md`.

## Commandes (Developer PowerShell for VS 2026, ou ouvrir le dossier dans Visual Studio 2026)
```
cmake --preset vs2026                          # configure (1re fois : télécharge les dépendances)
cmake --build --preset release                 # compile
ctest --preset release                         # tests
.\build\vs2026\sandbox\Release\mx_sandbox.exe  # lance le jeu (depuis la racine du dépôt)
```
Hors Developer PowerShell, cmake est dans
`C:\Program Files\Microsoft Visual Studio\18\Community\Common7\IDE\CommonExtensions\Microsoft\CMake\CMake\bin`.

## Commandes de jeu
Gâchette droite gaz · gâchette gauche frein AV · LB frein AR (progressif) · stick droit (ou gauche) vertical : pilote
avant/arrière. Clavier : flèches/WASD, Espace = frein AR. R départ · 1/2/3 avant saut 1 / saut 2 / whoops ·
P pause · T ralenti x0,25 · F1 panneau · molette zoom.

## Étapes futures
1. ~~Prototype 2D jouable~~ (fait)
2. 3D avec direction et inclinaison latérale
3. Pilote sur 2 axes (avant/arrière + gauche/droite, haut/bas)
4. Boîte de vitesses + embrayage (+ brin inférieur de chaîne en frein moteur)
5. Terrain déformable (ornières, terre qui se creuse)
6. Enregistrement / relecture des descentes
7. Intégration Unreal Engine 5
