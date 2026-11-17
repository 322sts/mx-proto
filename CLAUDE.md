# MX Proto — physique de motocross

Jeu de motocross à physique réaliste et exigeante (esprit MX Simulator). **Rien n'est scripté** : scrub,
whip, wheelie, absorption des bosses doivent émerger de la simulation moto + pilote + sol, jamais
d'animations ou de triches. Le pilote (l'utilisateur) teste **uniquement au ressenti**, à la manette Xbox,
et donne ses retours en langage de pilote. Le projet final ira dans Unreal Engine 5 ; pour l'instant
on construit la physique dans ce prototype léger (compile + relance en quelques secondes).

## Architecture
- `physics/` — bibliothèque C++20 pure, **aucune dépendance** (ni raylib, ni ImGui, ni Unreal). Unités SI partout.
  - `Math3D.h` : Vec3, Quat, RigidBody3D (inertie diagonale, terme gyroscopique ω×Iω) ; à remplacer pour Unreal.
  - `BikeParams.h/.cpp` : `BikeParams` (tous les réglages, défauts réalistes, unité en commentaire),
    `Inputs`, et `paramTable()` (libellés, bornes stables, unité d'affichage) utilisé par l'UI, les presets et les tests.
  - `Terrain.h/.cpp` : carte de hauteurs h(x, z) (mailles 10 cm, bicubique Catmull-Rom C1), normale, piste de test
    (bande de 10 m le long de x au milieu d'une zone plate de 80 m de large pour tourner), `makeFlat()` pour les tests.
  - `BikeSim.h/.cpp` : `reset(params, terrain, x)` (z = 0, cap +x), `step(inputs, terrain, dt)`, `state()`, `layout()`.
- `sandbox/main.cpp` — jeu de test : raylib (rendu + manette), Dear ImGui via rlImGui (réglages), ImPlot (courbes).
- `tests/` — doctest : terrain lisse, repos sans vibration, accélération, ligne droite, anti-squat, gaz en l'air,
  virage établi (tan φ = v²/gR), contre-braquage émergent, chaque curseur à ses bornes.
- Dépendances via FetchContent, versions épinglées : raylib 5.5, imgui 1.93.0, implot 1.1, rlImGui @5c17e70, doctest 2.5.3.

## Modèle physique (étape 2, 3D)
- 4 corps rigides 3D : châssis, bloc avant (fourreaux + masse roue AV ; inertie du guidon autour de l'axe de direction),
  bras oscillant (+ masse roue AR), pilote. Rotation propre de chaque roue = degré de liberté séparé (gaz/frein en
  l'air font tourner la moto) ; son moment cinétique donne le couple gyroscopique -ω×H au porteur.
- Liaisons rigides par impulsions séquentielles (lignes de jacobien, 20 itérations, Baumgarte 0,2) :
  direction + fourche = liaison cylindrique le long de l'axe de direction (déport 45 mm, chasse ~135 mm),
  pivot de bras = charnière, butées de débattement + 30 mm et de braquage (±40°). Tout le reste = forces.
- Fourche : ressort préchargé, amortissement compression/détente, cône anti-talonnage k·p².
- Arrière : rapport de biellette linéaire en course roue (2,8 → 1,85), course amortisseur = ∫dx/LR, force roue = F_amorto/LR.
- Pneus : profil arrondi (tore, rayon 5 cm : le contact passe sur le flanc en angle), sol réévalué sous le contact ;
  ressort radial + amortissement + jante. Glissement combiné (ellipse) : glissement longitudinal / 20 % et dérive /
  tan 8,5°, plus le carrossage en dérive équivalente (poussée 0,7 N/N/rad) ; μ(s) courbe terre (pic 0,89, glisse 0,68) ;
  forces bornées à celles qui annulent le glissement en un pas (stabilité).
- Pilote : corps relié aux 2 repose-pieds et aux 2 poignées par ressorts-amortisseurs + gainage roulis/lacet (genoux) ;
  le stick décale les points d'attache (±25 cm, vitesse limitée) → vrai transfert de poids.
  Direction : le stick gauche donne l'inclinaison visée (±34°) ; le pilote simulé pousse sur le guidon (couple autour
  de l'axe, max 40 N.m) pour suivre une vitesse de bascule voulue → le contre-braquage émerge, rien n'est imposé.
  Pied à terre sous 3 m/s (contacts ressort aux pieds, ne portent que si la moto penche).
- Moteur : courbe couple/régime, rapport unique 13:1, embrayage virtuel qui patine sous 4000 tr/min, frein moteur,
  inertie vilebrequin ramenée à la roue. Chaîne : tension = couple roue / bras de levier, tire l'axe AR vers le pignon
  de sortie de boîte (anti-squat qui émerge de la géométrie), réaction sur le cadre au pignon. Freins : frottement implicite roue/porteur (blocage propre).
- Boucle : pas fixe 1 ms (1000 Hz), Euler semi-implicite, accumulateur séparé du rendu, max 50 sous-pas par image.

## Conventions
- x vers l'avant, y vers le haut, z vers la droite (repère direct). Rotation + autour de z = cabrage, de x = penche
  à droite, de y = tourne à gauche. Braquage + = à gauche, inclinaison + = à droite. Roue qui avance : spin < 0.
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
Gâchette droite gaz · gâchette gauche frein AV · LB frein AR (progressif) · stick gauche horizontal : inclinaison
visée (direction) · stick droit (ou gauche) vertical : pilote avant/arrière. Clavier : W/S ou ↑/↓ gaz/frein AV,
A/D ou ←/→ direction, Q/E pilote arrière/avant, Espace frein AR. R départ · 1/2/3 avant saut 1 / saut 2 / whoops ·
C vue de côté · P pause · T ralenti x0,25 · F1 panneau · molette zoom.

## Étapes futures
1. ~~Prototype 2D jouable~~ (fait)
2. 3D avec direction et inclinaison latérale (en cours : moto 3D jouable, à peaufiner au ressenti)
3. Pilote sur 2 axes (avant/arrière + gauche/droite, haut/bas)
4. Boîte de vitesses + embrayage (+ brin inférieur de chaîne en frein moteur)
5. Terrain déformable (ornières, terre qui se creuse)
6. Enregistrement / relecture des descentes
7. Intégration Unreal Engine 5
