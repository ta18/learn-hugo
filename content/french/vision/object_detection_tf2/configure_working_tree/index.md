---
title: "Configurer l'arborescence de travail pour plusieurs projets de détection d'objets dans des images"
menu:
  main:
    name: "Configurer l'arborescence"
    weight: 2
    parent: "objectDetectionTF2"
---

|Acquis d'apprentissages visés :|
|:--|
|Savoir organiser et construire l'arborescence de travail pour supporter plusieurs projets de détection d'objets dans des images|

## Principe de l'arborescence de travail multi-projet

L'arborescence générique proposée est la suivante :

	tod_tf2
	├── <project>
	│   ├── images
	│   │   ├── test
	│   │   │   └── *.jpg, *.png ... *.xml
	│   │   ├── train
	│   │   │   └── *.jpg, *.png ... *.xml
	│   │   └── *.csv
	│   │
	│   └── training
	│       ├── <pre_trained-net>
	│       ├── train.record
	│       ├── test.record
	│       └── label_map.txt
	├── pre_trained
	│	└── <pre_trained-net>
    │	
	└── models
	    └── research
	        └── object_detection
	
* À chaque projet correspond un répertoire `<project>` spécifique.

* Le dossier `<project>/images` contient pour chaque projet :
	* les dossiers `test` et `train` qui contiennent chacun :
		* les images PNG, JPG... à analyser,
		* les fichiers d'annotation XML créés avec le logiciel `labelImg` : ils donnent, pour chacun des objets d'une image, les coordonnées de la boîte englobant l'objet et le label de l'objet.
	* les fichiers d'annotation CSV (contenu des fichiers XML converti au format CSV), qui seront à leur tour convertis au format _tensorflow record_.
* Le dossier `<project>/training` contient pour chaque projet :
	* un dossier `<pre_trained_net>` pour chaque réseau pré-entrainé utilisé : tu peux ainsi essayer plusieurs types de réseaux pré-entraînés pour comprarer leurs performances. C'est dans le dossier `<pre_trained_net>` que sont stockés les fichiers des poids du réseau entraîné.
	* les fichiers `train.reccord`  et `test.reccord` : contiennent les données labelisées d'entraînement et de test converties du format CSV au format _tensorflow record_,
	* le fichier `label_map.txt` : liste les labels correspondants aux objets à détecter.

* Le dossier `pre_trained/` contient un sous-dossier pour chacun des réseaux pré-entrainés utilisé.
	
## Exemple
	
Pour la détection des "faces des cubes" dans les images de la caméra du robot, le dossier `<project>` est nommé `faces_cubes`, ce qui donne l'arborescence de travail :

	tod_tf2
	├── faces_cubes
	│   ├── images
	│   │   ├── test
	│   │   │   └── *.jpg, *.png ... *.xml
	│   │   ├── train
	│   │   │   └── *.jpg, *.png ... *.xml
	│   │   └── *.csv	│   │
	│   └── training
	│       ├── <pre_trained-net>
	│       ├── train.record
	│       ├── test.record
	│       └── label_map.txt
	├── pre_trained
	│	└── <pre_trained-net>
	└── models
	    └── research
	        └── object_detection

## Mise en oeuvre :

Quelques commandes shell suffisent pour créer les premiers niveaux de cette arborescence :

```bash	
# From within tod_tf2
(tf2) user@host $ mkdir -p faces_cubes/images/test
(tf2) user@host $ mkdir -p faces_cubes/images/train
(tf2) user@host $ mkdir -p faces_cubes/training
(tf2) user@host $ mkdir pre_trained
s
```
Vérifions :
```bash	
# From within tod_tf2
(tf2) user@host $ tree -d . -I models
.
├── faces_cubes
│   ├── images
│   │   ├── test
│   │   └── train
│   └── training
├── pre_trained
└── tod_tf2_tools
```

