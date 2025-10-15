# Projet : Contrôle commande d’un entraînement électrique  
## Auteurs : Maria Saad, Vincent Lartaud
## Module : ES206 – ENSTA Paris  

---

## Objectif du projet

Ce projet a pour but de modéliser et de simuler différentes stratégies de commande d’une machine synchrone triphasée à l’aide de MATLAB.  
Chaque partie correspond à une étape d’amélioration du pilotage du moteur afin d’obtenir une alimentation plus stable, un couple mieux régulé et des courants plus sinusoïdaux.

Tous les scripts ont été écrits en MATLAB, et les figures ont été générées automatiquement.

---

## Structure du dépôt

```plaintext
Rendu_ES206_Lartaud_Saad/
│
├── Rapport Lartaud Saad.pdf          # rapport complet du projet
│
├── Projet 1/                         # machine synchrone – modèle de base
│   ├── main.m
│   ├── park.m
│   ├── park_1.m
│   └── Graphiques/
│
├── Projet 2/                         # commande par modulation de largeur d’impulsion
│   ├── main.m
│   ├── park.m
│   ├── park_1.m
│   └── Graphiques/
│
└── Projet 3/                         # commande en tension progressive et surveillance
    ├── main.m
    ├── park.m
    ├── park_1.m
    └── Graphiques/
```

---

## Partie 1 – Machine synchrone

La première partie consiste à modéliser le comportement d’une machine synchrone alimentée par des tensions triphasées en créneaux.  
Les grandeurs électriques et mécaniques (flux, courants, couple, vitesses) sont calculées à partir des équations du moteur et de la transformée de Park.

### Résultats principaux
- Courant crête : 25,2 A  
- Vitesse moyenne : 68,2 rad/s  
- Couple moyen : 20,5 N·m  

Le moteur atteint un régime quasi stationnaire, mais les tensions en créneaux produisent des harmoniques qui rendent le couple et la vitesse instables.  
Lorsque le couple résistant est supprimé, les grandeurs divergent et le moteur s’emballe.

---

## Partie 2 – Modulation de largeur d’impulsion (MLI)

Cette partie introduit une modulation de largeur d’impulsion pour lisser les tensions d’alimentation et réduire les vibrations mécaniques.  
Les signaux MLI reproduisent en moyenne des sinusoïdes à partir des tensions de référence.

### Résultats
- Courant crête : 40,7 A  
- Vitesse moyenne : 185 rad/s  
- Couple moyen : 39,2 N·m  

Les courants deviennent quasi sinusoïdaux et le couple est plus stable.  
Le rendement s’améliore nettement et les pertes sont réduites par rapport à la partie 1.

---

## Partie 3 – Commande en tension progressive et surveillance

Cette commande fait croître progressivement les tensions Vd et Vq afin d’éviter les surintensités au démarrage.  
Un mécanisme de surveillance des courants est également mis en place par échantillonnage.

### Résultats
- Courant crête : 84,9 A  
- Vitesse finale : 600 rad/s  
- Couple maximal : 76,1 N·m  

La montée en vitesse est rapide, mais les flux ne sont pas régulés, ce qui rend le couple instable.  
Les courants atteignent des valeurs très élevées, supérieures à celles des parties précédentes.  
La stratégie atteint son objectif de montée en régime, mais elle manque de stabilité.

---

## Partie 4 – Commande en flux

Cette dernière partie met en place une commande en flux visant à stabiliser le couple et les courants à travers une régulation active des flux magnétisants φd et φq.  
Le contrôle du couple est réalisé par une architecture proportionnelle (sans intégrale).

### Résultats
- Courant crête : 48,5 A  
- Vitesse finale : 953,9 rad/s  
- Couple établi : 19,3 N·m  

Le système reste stable même sans couple résistant, contrairement à la première partie.  
Le couple est régulier au démarrage mais décroît en régime établi, faute de correcteur intégral.  
C’est la stratégie la plus robuste parmi les quatre, bien qu’elle puisse encore être améliorée.

---

## Comparaison des résultats

| Partie | Stratégie                  | Imax (A) | Couple (N·m) | Vitesse (rad/s) |
|---------|----------------------------|-----------|---------------|-----------------|
| 1       | Créneaux                   | 25.2      | 20.5          | 68.2            |
| 2       | MLI                        | 40.7      | 39.2          | 185             |
| 3       | Tension progressive        | 84.9      | 76.1 (max)    | 600             |
| 4       | Commande en flux           | 48.5      | 19.3 (établi) | 953.9           |

La commande en flux se révèle la plus stable et la plus complète, car elle maintient la machine sous contrôle même sans couple résistant.  
Une amélioration possible serait l’ajout d’un correcteur PI complet pour compenser l’erreur statique et améliorer la précision du couple.

---

## Exécution des simulations

1. Ouvrir MATLAB.  
2. Aller dans le dossier du projet souhaité (Projet 1, 2, 3 ou 4).  
3. Exécuter le fichier `main.m`.  
4. Les résultats et figures seront enregistrés automatiquement dans le sous-dossier `Graphiques`.

---

