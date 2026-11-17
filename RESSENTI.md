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
