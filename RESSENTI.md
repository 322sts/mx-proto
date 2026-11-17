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

### 2026-11-18 — 3D : plus du tout réactif, il faut tenir le stick longtemps, et on tombe
**Ressenti :**
- Plus du tout réactif : il faut appuyer longtemps sur le stick, et au final on tombe.

**Diagnostic (banc, virages serrés stick à fond) :** chute à chaque fois entre ~5 et 18 km/h. Le mode « guidon
tenu » (prévu pour l'arrêt) prenait le relais dès 3-5 m/s : stick à fond = butée de braquage alors que la moto
roule -> elle tombe vers l'extérieur, ou vers l'intérieur quand on ralentit en gardant 34° d'inclinaison
(à basse vitesse ce virage demanderait un rayon plus serré que la butée ne le permet).

**Modifications :**
- code : le pilote équilibre par le guidon jusqu'à l'allure du pas ; « guidon tenu » seulement sous ~1,5 m/s.
- code : inclinaison visée limitée par la vitesse, tan φ ≤ v²/(g·R_min), R_min = 1,5 x empattement / tan(butée).
- `footDownSpeed` 3 → 1,5 m/s (pieds posés sous ~5 km/h seulement).
- `riderMaxRollRate` 29 → 40°/s, `riderLeanResponse` 1,5 → 2,2 : entre « trop vif » (46°/s) et « pas réactif » (29°/s).

**Mesures après :** stick à fond : 90 % de l'inclinaison en 1,1-1,2 s ; virages serrés de 11 à 36 km/h,
vitesse tenue ou gaz coupés jusqu'à l'arrêt : aucune chute (avant : chute de 11 à 25 km/h gaz coupés).

**À revérifier :**
- Réactivité : bon compromis ? (curseur F1 > Pilote > « Guidon : bascule max » pour l'ajuster en direct)
- Virages serrés lents (épingle, berm) : on passe sans tomber ?

### 2026-11-18 — 3D : tourne très mal à basse vitesse, contre-braquage « ignoble »
**Ressenti :**
- À basse vitesse, ça tourne très mal.
- Le contre-braquage est ignoble. S'inspirer de la direction de MX Bikes / MX Simulator.

**Diagnostic :** le pilote simulé ne pilotait qu'en couple au guidon à partir de l'écart d'inclinaison : grands
allers-retours du guidon (contre-braquage visible), et à basse vitesse incapable de « tourner le guidon » pour virer.

**Modifications (nouveau pilote, principe des robots d'équilibre de vélo) :**
- code : le stick demande une **courbure** de virage, plus une inclinaison : stick à fond = virage le plus serré
  tenable (butée de braquage à basse vitesse, 34° à haute vitesse).
- code : le pilote calcule l'**angle de guidon** à tenir par le modèle du pendule inversé,
  δe = L/v² (h φ''_voulu − g tan φ) = braquage du virage établi + correction d'équilibre (braquer vers la chute),
  et le tient avec ses bras (ressort-amortisseur). Le contre-braquage en sort, d'autant plus petit qu'on va vite.
  Terme d'équilibre calculé avec v plafonnée à 7 m/s (sinon trop mou à haute vitesse).
- `riderSteerGain` devient le suivi de bascule (8 /s), `riderBarHold` la fermeté des bras (250 N.m/rad),
  `riderMaxSteerTorque` 40 → 50 N.m.

**Mesures (stick à fond, vitesse tenue) :**
- 11 km/h : rayon 3,6 m (avant : tournait à peine), incl. 20°, contre-braquage bref 11°.
- 18 km/h : rayon 5,6 m, 30° en 1,0 s, contre-braquage 6°. 29 km/h : rayon 12 m, 32° en 1,1 s, 3°.
- 50 km/h : 36° en 1,5 s, contre-braquage 2°. Gaz coupés en virage jusqu'à l'arrêt : aucune chute.

**À revérifier :**
- Virages lents : tourne bien maintenant ? Le contre-braquage est-il acceptable (visible surtout sous 20 km/h) ?

### 2026-11-18 — Virages : stick à fond obligatoire, berms qui tournent peu, chute en sortie de berm
**Ressenti :**
- La physique de direction est beaucoup mieux.
- Dans les virages il faut pousser le stick à fond ; dans les berms ça tourne très peu et on tombe à la fin du berm.

**Diagnostic (banc : berm serré r = 12 m, stick à fond) :**
- Le pilote limitait l'inclinaison à 34° par rapport à la VERTICALE, même dans un berm : il ne profitait pas du
  dévers, tournait large, remontait le berm et passait la crête (32 km/h : 7,6 m d'écart, chute).
- Le berm s'aplatissait sur les 6 derniers mètres du virage, encore au rayon le plus serré : moto penchée à 45° sur
  du plat -> l'avant décroche (Fy/N = 0,89 = mu). Et la moitié intérieure de la piste était plate : plonger à la
  corde en gardant l'inclinaison du berm faisait décrocher l'avant.
- À 40 km/h, avant délesté : le pilote braquait de 25° et la moto partait en guidonnage.

**Modifications :**
- code : la limite d'inclinaison porte sur l'angle par rapport au SOL (normale sous les roues) : dans un berm
  penché de β, il penche de 34° + β et tourne bien plus serré.
- code : angle de guidon plafonné à ce qui sert à la vitesse (1,5 x virage le plus serré tenable + 3°).
- piste : berms en cuvette sur toute la largeur (peu relevés à la corde, de plus en plus vers l'extérieur),
  relevés sur 8 m avant et 8 m après le virage ; hauteurs 1,4/1,6/1,0 -> 1,8/2,0/1,3 m. Rouleaux, petit saut
  et départ décalés pour laisser la place aux raccords.

**Mesures après (stick à fond, lancé à vitesse tenue) :** 25 km/h : 44° d'inclinaison, reste dans le berm ;
32 km/h : 42°, reste sous la crête ; 40 km/h avec braquage tardif : sort large (physique : il faudrait 46° tôt).
Sur le plat : inchangé.

**À revérifier :**
- Berms : ça tourne, ça tient, on ne tombe plus en sortie ?
- Plat : faut-il encore le stick à fond ? (c'est la limite d'adhérence : 34° ~ 0,67 g sur terre plate)
