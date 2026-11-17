# Journal de ressenti

Une entrée par session de pilotage : ce que j'ai ressenti (en langage de pilote), puis ce qui a été
changé dans la physique et pourquoi.

## Modèle d'entrée

### AAAA-MM-JJ — titre court
**Ressenti :**
- ...

**Modifications :**
- `paramètre` : ancienne valeur → nouvelle valeur — pourquoi (effet physique attendu)
- code : ...

**À revérifier :**
- ...

---

### 2026-11-17 — Suspensions trop molles, la moto lève trop
**Ressenti :**
- Suspensions un peu trop molles.
- La moto lève un peu trop.

**Mesures avant :** sag fourche 61 mm / AR 93 mm ; à 60 % de gaz l'AR s'écrase jusqu'à 158 mm (la moto
s'assoit et cabre) ; petit saut : fourche 278/300 mm, AR 296/310 mm (presque en butée).

**Modifications :**
- `forkSpringRate` : 9,4 → 10,8 N/mm — fourche plus ferme, moins d'enfoncement.
- `forkCompDamping` 700 → 1000 N.s/m, `forkRebDamping` 1300 → 1600 N.s/m — l'amorti suit la raideur (même tenue relative), moins de pompage.
- `shockSpringRate` : 48 → 56 N/mm — l'AR s'écrase moins à l'accélération, donc la moto s'assoit et cabre moins.
- `shockCompDamping` 3000 → 4000 N.s/m, `shockRebDamping` 6500 → 8000 N.s/m — idem côté AR, détente plus retenue (moins de « coup de pied » en sortie de bosse).
- `linkRatioEnd` : 2,0 → 1,85 — biellette plus progressive en fin de course contre le talonnage.

**Mesures après :** sag fourche 53 mm / AR 76 mm ; à 60 % de gaz AR max 128 mm (au lieu de 158), cabrage 5° (au lieu de 6°) ;
petit saut : fourche 236 mm.

**À revérifier :**
- Fermeté générale : assez, trop, pas assez ?
- « La moto lève » : en accélération (wheelie) ou en sortie de bosse / lèvre de saut (coup de pied) ?
- Note : l'anti-squat (la tension de chaîne qui empêche l'arrière de s'écraser aux gaz) n'est pas encore
  modélisé (étape boîte/embrayage). C'est une part réelle du « ça s'assoit et ça lève » à l'accélération.

### 2026-11-17 — Encore trop mou partout, la moto lève aux gaz sur le plat
**Ressenti :**
- Trop mou : réceptions, whoops, freinage, accélération.
- Ça lève : aux gaz, sur le plat (wheelie, la moto s'assoit).

**Diagnostic (banc de mesure) :**
- La chaîne n'existait pas : le couple moteur était appliqué au cadre comme un couple pur qui lève le nez.
  À 60 % de gaz l'AR s'écrasait de 76 à 128 mm ; gaz à fond, la roue AV décollait en 0,04 s et la moto bouclait,
  même pilote couché sur le guidon, même lancée à 40 km/h.
- Adhérence μ 1,1 : assez pour boucler en accélération et passer par-dessus le guidon au frein AV.
- Amortissement en compression faible : chute à plat de 0,5 m = 60 % du débattement ; fourche qui plonge de 200 mm au freinage.

**Modifications :**
- code : chaîne (brin supérieur) entre la couronne et le pignon de sortie de boîte. Sa tension tire l'axe AR vers
  l'avant -> couple qui détend le bras autour du pivot (anti-squat) ; la réaction va au cadre au pignon (tend à
  baisser le nez au lieu de le lever). Nouveaux réglages : position du pignon de sortie, rayons pignon/couronne.
- `countershaftY` = 180 mm (2 cm au-dessus du pivot) : à 140 mm l'anti-squat était trop fort, l'AR se détendait en butée gaz à fond.
- `tirePeakMu` 1,10 → 0,85, `tireSlideMu` 0,80 → 0,65 — valeurs de crampons sur terre ; la roue AR patine au lieu de faire boucler la moto.
- `forkCompDamping` 1000 → 2000, `forkRebDamping` 1600 → 2500 N.s/m — la fourche plonge moins vite au freinage et à la réception.
- `shockCompDamping` 4000 → 7000, `shockRebDamping` 8000 → 11000 N.s/m — idem à l'AR ; la détente reste plus freinée que la compression.

**Mesures après :**
- Gaz 60 % depuis l'arrêt : AR 74..86 mm (sag 76) au lieu de 128 ; gaz à fond pilote neutre : cabrage 6°, plus de looping.
- Gaz à fond lancé à 40 km/h : pilote neutre = la moto lève (au pilote de se mettre sur l'avant) ; pilote en avant = 1 à 5°.
- Chute à plat 0,5 m : fourche 131 / AR 151 mm (au lieu de 183 / 181) ; 1,5 m : 190 / 228 (au lieu de 271 / 265).
- Frein AV 60 % à 50 km/h : plongée fourche 162 mm (au lieu de 200) ; frein AV 100 % : plus de passage par-dessus le guidon.

**À revérifier :**
- La moto lève-t-elle encore trop aux gaz ? Est-ce qu'elle patine trop maintenant (adhérence) ?
- Fermeté : assez ? Les grosses réceptions sont-elles devenues sèches (l'amortissement est linéaire : plus ferme
  partout, y compris sur les gros chocs ; si c'est sec, prochaine étape = amortisseur dégressif basse/haute vitesse).
- Les anciens presets (`presets/tes.ini`) gardent les anciennes valeurs.

### 2026-11-17 — Mieux, mais encore un peu mou
**Ressenti :**
- Mieux qu'avant (chaîne + amortissement), mais encore un peu mou.

**Diagnostic :** aucun rebond après une réception (l'amortissement tient) ; le « mou » = l'enfoncement total.
Essai jambes du pilote plus dures (45 N/mm) : le pilote s'écrase moins, mais la moto encaisse plus et le frein AV
à fond refait passer par-dessus le guidon -> abandonné.

**Modifications :**
- `forkSpringRate` 10,8 → 12,0 N/mm, `shockSpringRate` 56 → 62 N/mm (+10 %).
- Amortissement +5 % (en racine de la raideur, même tenue) : fourche 2100 / 2650, amortisseur 7400 / 11600 N.s/m.

**Mesures après :** sag 47 / 68 mm (au lieu de 53 / 76) ; chute 0,5 m : 121 / 144 mm (au lieu de 131 / 151) ;
1,5 m : 178 / 218 (au lieu de 190 / 228) ; freinage AV 60 % : fourche 148 mm (au lieu de 162). Anti-squat, wheelie inchangés.

**À revérifier :**
- Fermeté. On est maintenant sur des ressorts de pilote lourd / supercross. Si c'est encore mou, la piste suivante
  est un amortissement dégressif (fort à basse vitesse = moto tenue, qui s'ouvre sur les gros chocs).

### 2026-11-17 — Plus de stoppie au freinage, la moto ne lève plus assez aux gaz
**Ressenti :**
- Bien mieux en fermeté.
- Au freinage : plus du tout de stoppie ; il en faut un peu pour être réaliste.
- Atterrir trop long, sur le plat : ça rebondit, « pas trop mal » -> on garde.
- À l'accélération : la moto doit lever un peu, pas énormément.

**Diagnostic :** wheelie et stoppie sont tous les deux limités par l'adhérence (il faut ~1 g de poussée ou de
freinage pour lever une roue). À μ 0,85 le pneu décroche avant ; à μ ≥ 0,93 tout boucle, même pilote couché sur le guidon.
La transition est très serrée : c'est physique, un wheelie / stoppie est instable et se dose au pilotage.

**Modifications :**
- `tirePeakMu` 0,85 → 0,89, `tireSlideMu` 0,65 → 0,68.

**Mesures après (banc, commandes figées, 1,5 s) :**
- Frein AV à fond à 50 km/h : pilote en arrière = stoppie 13 cm ; neutre = 9 cm ; sur l'avant = passe par-dessus.
- Gaz à fond depuis l'arrêt : pilote neutre = la roue AV monte franchement ; mi-avant ou tout avant = 1-2 cm.
- Gaz à fond lancé à 40 km/h : neutre / mi-avant = lève ; tout sur l'avant = reste collé.
- Fermeté, chutes, plongée au freinage : inchangées.

**À revérifier :**
- Stoppie et wheelie dosables au ressenti (gâchettes + stick) ? Trop ou pas assez ?

### 2026-11-17 — En l'air : gaz et frein AR pas assez efficaces
**Ressenti :**
- Beaucoup mieux (stoppie / wheelie).
- En l'air : accélérer doit remonter un peu plus la moto, le frein AR doit la rabaisser un peu plus.

**Physique :** en l'air, gaz = la roue AR accélère, la moto tourne en sens inverse (nez qui monte) ; frein AR = la
roue ralentit, le nez descend. L'effet = inertie de la roue × variation de sa vitesse. Le frein AR est borné
(au mieux on arrête la roue) : seul levier honnête = l'inertie de la roue AR.

**Modifications :**
- `rearWheelInertia` 0,65 → 0,90 kg.m² (pneu lourd + mousse, haut de la plage réaliste).

**Mesures (vol libre 0,6 s à 50 km/h) :** gaz à fond +46° → +54° ; frein AR −20° → −26°. Wheelie / stoppie inchangés.

**À revérifier :**
- Assez marqué ? Note : le vilebrequin est compté comme s'il tournait avec la roue (inertie × rapport²), ce qui
  donne déjà un effet en l'air plus fort qu'une vraie moto ; à corriger avec la boîte (étape 4), il faudra alors recaler.

### 2026-11-17 — 3D : entrée de virage bien trop vive
**Ressenti :**
- Entrée en virage bien trop vive.
- Ligne droite à plat sans toucher la direction : pas trop mal. Sauts : pas mal.

**Mesures avant (stick à fond d'un coup) :** 90 % de l'inclinaison visée en 0,4-0,8 s (bascule jusqu'à 85°/s) ;
à 22 km/h stick à fond, la moto dépassait la cible et tombait.

**Modifications (pilote simulé) :**
- `riderMaxRollRate` 85 → 46°/s : il s'autorise une bascule moins brutale.
- `riderLeanResponse` 4 → 2,5 /s : il vise moins vite l'inclinaison demandée.
- `riderSteerGain` 40 → 60 N.m.s/rad : il tient plus fermement le guidon (sinon l'inclinaison finale dépasse la cible de 4-5°).

**Mesures après :** 90 % de l'inclinaison en ~1,0-1,1 s ; inclinaison finale 36-37° pour 34° visés ; plus de chute à 22 km/h.

**À revérifier :**
- Entrée de virage : bonne, encore trop vive, trop molle ?

### 2026-11-17 — 3D : encore beaucoup trop vif, guidon qui tourne seul à l'arrêt, retour trop fort
**Ressenti :**
- Entrée de virage encore beaucoup trop vive.
- Le guidon tourne tout seul à l'arrêt.
- Quand on braque d'un côté, ça revient trop fort de l'autre.

**Diagnostic :** à l'arrêt, le pilote simulé cherchait à pencher la moto en braquant (impossible sans vitesse) :
le moindre stick envoyait le guidon en butée à 40 N.m, et à basse vitesse il claquait d'une butée à l'autre.
En roulant, relâcher le stick ne fait pas dépasser de l'autre côté (mesuré : 0°).

**Modifications :**
- code : sous `footDownSpeed` (fondu sur 2 m/s), le pilote tient le guidon à l'angle demandé par le stick
  (`riderBarHold` 30 N.m/rad) au lieu de chercher l'inclinaison. Stick au repos à l'arrêt = guidon droit.
- `riderMaxRollRate` 46 → 29°/s, `riderLeanResponse` 2,5 → 1,5 /s : bascule deux fois plus calme qu'au départ.
- `riderSteerGain` 60 → 80 : tient l'inclinaison finale (36-37° pour 34° visés).

**Mesures après :** stick à fond : 90 % de l'inclinaison en ~1,6 s (1,0 avant, 0,4-0,8 à l'origine) ; relâché :
moto droite en 2-2,5 s sans dépasser ; à l'arrêt stick à fond : guidon en butée du côté demandé, couple ~0 une fois arrivé.

**À revérifier :**
- Vivacité : encore trop vif, ou trop lent maintenant ? Le « revient trop fort » est-il réglé ?
- Manœuvres à basse vitesse (départ, demi-tour).
