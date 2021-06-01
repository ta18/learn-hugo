---
title: "Compléter l'arborescence de travail"
menu:
  main:
    name: "Compléter l'arborescence de travail"
    weight: 3
    parent: "capsulesTF2"
---


| Classe de capsule  | &emsp;Durée recommandée |
|:-------------------|:------------------|
| Info  &emsp;  ℹ️  |&emsp; 5 min      |


## 🎒 Prérequis

* Quelques notions de base d'utilisation du terminal
* Capsule sur l'installation des modules

## 🎓 Acquis d'apprentissage

* Finaliser l'arborescence de travail pour le réseau de neurone

## Compléter l'arborescence de travail

L'arborescence générique proposée est la suivante :

	tod_tf2
	├── images
	│   └──<project>
	│       ├── test
	│       │   └── *.jpg, *.png ... *.xml
	│       ├── train
	│       │   └── *.jpg, *.png ... *.xml
	│       └── *.csv
	├── pre_trained
	│	└── <pre_trained-network>
	├── training
	│   └──<project>
	│       ├── <pre_trained-network>
	│       ├── train.record
	│       ├── test.record
	│       └── label_map.txt
	└── models
	    └── research
	        └── object_detection
	
* Tout ce qui est spécifique au projet est placé dans un répertoire `<project>` à différents niveaux.

* Le dossier `images/<project>` contient pour chaque projet :
	* les dossiers `test` et `train` qui contiennent chacun :
		* les images PNG, JPG... à analyser,
		* les fichiers d'annotation XML créés avec le logiciel `labelImg` : ils donnent, pour chacun des objets d'une image, les coordonnées de la boîte englobant l'objet et le label de l'objet.
	* les fichiers d'annotation CSV (contenu des fichiers XML converti au format CSV), qui seront à leur tour convertis au format _tensorflow record_.
* Le dossier `pre_trained/` contient un sous-dossier pour chacun des réseaux pré-entrainés utilisé.
* le dossier `training/<project>` contient pour chaque projet :
	* un dossier pour réseau pré-entrainé utilisé : c'est dans ce dossier que sont stockés les fichiers des poids du réseau entraîné,
	* les fichiers `train.reccord`  et `test.reccord` : contiennent les données labelisées d'entraînement et de test converties du format CSV au format _tensorflow record_,
	* le fichier `label_map.txt` : liste les labels correspondants aux objets à détecter.
	
Pour la détection des faces des cubes dans les images de la caméra du robot, le dossier `<project>` sera nommé `faces_cubes`, ce qui donne l'arborescence de travail :

	tod_tf2
	├── images
	│   └── faces_cubes
	│       ├── test
	│       │   └── *.jpg, *.png ... *.xml
	│       ├── train
	│       │   └── *.jpg, *.png ... *.xml
	│       └── *.csv
	├── pre_trained
	│	└── <pre_trained-network>
	├── training
	│   └── faces_cubes
	│       ├─── <pre_trained-network>
	│       ├── train.record
	│       ├── test.record
	│       └── label_map.txt
	└── models
	    └── research
	        └── object_detection

Quelques commandes shell suffisent pour créer les premiers niveaux de cette arborescence :

```bash	
# From within tod_tf2
(tf2) jlc@pikatchou $ mkdir -p images/faces_cubes/test
(tf2) jlc@pikatchou $ mkdir -p images/faces_cubes/train
(tf2) jlc@pikatchou $ mkdir pre_trained
(tf2) jlc@pikatchou $ mkdir -p training/faces_cubes
```
Vérifions :
```bash	
# From within tod_tf2
(tf2) jlc@pikatchou $ tree -d . -I models
.
├── images
│   └── faces_cubes
│       ├── test
│       └── train
├── pre_trained
├── tod_tf2_tools
└── training
    └── faces_cubes
```