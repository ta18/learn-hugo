---
title: "Installation des modules"
menu:
  main:
    name: "Installation des modules"
    weight: 3
    parent: "capsulesTF2"
---


| Classe de capsule  | &emsp;Durée recommandée |
|:-------------------|:------------------|
| Info  &emsp;  ℹ️  |&emsp; 15 min      |


## 🎒 Prérequis

* BAC+2 et +
* Bonne compréhension de Python et numpy
* Quelques notions de base d'utilisation du terminal
* Une première expérience des réseaux de neurones est souhaitable.

## 🎓 Acquis d'apprentissage

* Installation de l'API `Tensorflow Object Detection`
* Installation des outils `protobuf`

## 1. Installer l'API TOD

L'installation de l'API TOD se déroule en 5 étapes :
1. Créer et initialiser ton espace de travail
2. Cloner le dépôt `tensorflow/models`
3. Installer les outils `protobuf`
4. Installer l'API COCO
5. Installer le package `object_detection` 

Dans tout le document le _prompt_ du terminal sera noté `(tf2) jlc@pikatchou $` : le préfixe `(tf2)` est là pour bien rappeler que le travail Python pour l'API TOD se fait 
dans l'__Environnement Virtuel Python tf2__ que tu auras créé au préalable (cf les Prérequis).


## 2. Créer et initialiser son espace de travail

La première étape consiste à créer le répertoire de travail `tod_tf2`, dans lequel seront créés tous les fichiers, et de te positionner dans ce répertoire qui sera __le dossier racine du projet__.
:
```bash
(tf2) jlc@pikatchou $ cd <quelque_part>   # choisis le répertoire où créer `tod_tf2`, par exemple "cd ~/catkins_ws"
(tf2) jlc@pikatchou $ mkdir tod_tf2
(tf2) jlc@pikatchou $ cd tod_tf2/
```
📥 Ensuite, tu clones le dépôt github `cjlux/tod_tf2_tools.git` et tu copies les fichiers `*.py` et `*.ipynb` du dossier `tod_tf2_tools` dans le dossier `tod_tf2` : 
```bash
# From tod_tf2/
(tf2) jlc@pikatchou $ git clone https://github.com/cjlux/tod_tf2_tools.git
(tf2) jlc@pikatchou $ cp tod_tf2_tools/*.py .
(tf2) jlc@pikatchou $ cp tod_tf2_tools/*.ipynb .
```

## 3. Cloner le dépôt `tensorflow/models`

📥 Dans le dossier de travail `tod_tf2` clone le dépôt github `tensorflow/models` (~ 635 Mo) :
```bash
# From tod_tf2/
(tf2) jlc@pikatchou $ git clone https://github.com/tensorflow/models.git
```

Tu obtiens un dossier `models`. L’API TOD est dans le dossier `models/research/object_detection` :
```bash	
(tf2) jlc@pikatchou $ tree -d -L 2 .
.
└── models
    ├── community
    ├── official
    ├── orbit
    └── research
```	

📥 Complète ton installation avec quelques paquets Python utiles pour le travail avec l'API TOD :

```bash
(tf2) jlc@pikatchou $ conda install cython contextlib2 pillow lxml
(tf2) jlc@pikatchou $ pip install labelimg rospkg
```
Mets à jour la variable d’environnement `PYTHONPATH` en ajoutant à la fin du fichier `~/.bashrc` les deux lignes :
```bash
export TOD_ROOT="<chemin absolu du dossier tod_tf2>"
export PYTHONPATH=$TOD_ROOT/models:$TOD_ROOT/models/research:$PYTHONPATH
```
remplace `"<chemin absolu du dossier tod_tf2>"` par le chemin absolu du dossier `tod_tf2` sur ta machine.

* Lance un nouveau terminal pour activer le nouvel environnement shell : tout ce qui suit sera fait dans ce nouveau terminal.
* ⚠️ n'oublie pas d'activer l'EVP `tf2` dans ce nouveau terminal :
```bash
jlc@pikatchou $ conda activate tf2
(tf2) jlc@pikatchou $
 ```

## 4. Installer les outils `protobuf`

L’API native TOD utilise des fichiers `*.proto` pour la configuration des modèles et le stockage des paramètres d’entraînement. 
Ces fichiers doivent être traduits en fichiers `*.py` afin que l’API Python puisse fonctionner correctement : 

* Installe d'abord le paquet debian `protobuf-compile` qui donne accès à la commande `protoc` :
```bash
(tf2) jlc@pikatchou $ sudo apt install protobuf-compiler
```
* Tu peux ensuite te positionner dans le dossier `tod_tf2/models/research` et taper :
```bash
# From tod_tf2/models/research/
(tf2) jlc@pikatchou $ protoc object_detection/protos/*.proto  --python_out=.
```
Cette commande travaille de façon muette.

## 5. Installer l'API COCO

COCO est une banque de données destinée à alimenter les algorithmes de détection d’objets, de segmentation… <br>
Voir [cocodataset.org](https://cocodataset.org) pour les tutoriels, publications… 

📥 Pour installer l’API Python de COCO, clone le site `cocoapi.git` (~ 15 Mo) dans le dossier `/tmp`, tape la commande `make` dans le dossier `cocoapi/PythonAPI`, puis recopie le dossier `pycococtools` dans ton dossier `.../models/research/` :
```bash
(tf2) jlc@pikatchou $ cd /tmp
(tf2) jlc@pikatchou $ git clone  https://github.com/cocodataset/cocoapi.git
(tf2) jlc@pikatchou $ cd cocoapi/PythonAPI/
(tf2) jlc@pikatchou $ make
(tf2) jlc@pikatchou $ cp -r pycocotools/ <chemin absolu du dossier tod_tf2>/models/research/
```
remplace `"<chemin absolu du dossier tod_tf2>"` par le chemin absolu du dossier `tod_tf2` sur ta machine (par exemple `~/catkins_ws/tod_tf2`).

## 6. Installer le package `object_detection` 

Pour finir l'installation, place-toi dans le dossier  `models/research/` et tape les commandes :
```bash
# From tod_tf2/models/research/
(tf2) jlc@pikatchou $ cp object_detection/packages/tf2/setup.py .
(tf2) jlc@pikatchou $ python setup.py build
(tf2) jlc@pikatchou $ pip install .
```

## 7. Tester l'installation de l'API TOD

Pour tester ton installation de l’API TOD, place-toi dans le dossier `models/research/` et tape la commande :
```bash	
# From within tod_tf2/models/research/
(tf2) jlc@pikatchou $ python object_detection/builders/model_builder_tf2_test.py
```
Le programme déroule toute une série de tests et doit se terminer par un OK sans fiare apparaître d'erreur :

	...
	[       OK ] ModelBuilderTF2Test.test_invalid_second_stage_batch_size
	[ RUN      ] ModelBuilderTF2Test.test_session
	[  SKIPPED ] ModelBuilderTF2Test.test_session
	[ RUN      ] ModelBuilderTF2Test.test_unknown_faster_rcnn_feature_extractor
	INFO:tensorflow:time(__main__.ModelBuilderTF2Test.test_unknown_faster_rcnn_feature_extractor): 0.0s
	I0505 18:19:38.639148 140634691176256 test_util.py:2075] time(__main__.ModelBuilderTF2Test.test_unknown_faster_rcnn_feature_extractor): 0.0s
	[       OK ] ModelBuilderTF2Test.test_unknown_faster_rcnn_feature_extractor
	[ RUN      ] ModelBuilderTF2Test.test_unknown_meta_architecture
	INFO:tensorflow:time(__main__.ModelBuilderTF2Test.test_unknown_meta_architecture): 0.0s
	I0505 18:19:38.640017 140634691176256 test_util.py:2075] time(__main__.ModelBuilderTF2Test.test_unknown_meta_architecture): 0.0s
	[       OK ] ModelBuilderTF2Test.test_unknown_meta_architecture
	[ RUN      ] ModelBuilderTF2Test.test_unknown_ssd_feature_extractor
	INFO:tensorflow:time(__main__.ModelBuilderTF2Test.test_unknown_ssd_feature_extractor): 0.0s
	I0505 18:19:38.641987 140634691176256 test_util.py:2075] time(__main__.ModelBuilderTF2Test.test_unknown_ssd_feature_extractor): 0.0s
	[       OK ] ModelBuilderTF2Test.test_unknown_ssd_feature_extractor
	----------------------------------------------------------------------
	Ran 21 tests in 53.105s

	OK (skipped=1)

Pour finir, tu peux vérifier l’installation en utilisant le notebook IPython `object_detection_tutorial.ipynb` présent dans le dossier `tod_tf2`.<br>
(note : c'est une copie du notebook `tod_tf2/models/research/object_detection/colab_tutorials/object_detection_tutorial.ipynb` dans laquelle on a enlevé les cellules d'installation de l'API_TOD et quelques autres cellules qui peuvent générer des erreurs...).

* ⚠️ Avant d'exécuter les cellules du notebook, il faut corriger une erreur dans le fichier `.../tod_tf2/models/research/object_detection/utils/ops.py`, ligne 825 :
remplace `tf.uint8` par `tf.uint8.as_numpy_dtype`

* Dans le dossier `tod_tf2` lance la commande `jupyter notebook` et charge le notebook `object_detection_tutorial.ipynb`.
* Exécute les cellules une à une, tu ne dois pas avoir d’erreur :

	* La partie "__Detection__" (qui dure de quelques secondes à  plusieurs minutes suivant ton CPU…) utilise le réseau pré-entraîné `ssd_mobilenet_v1_coco_2017_11_17` pour détecter des objets dans les   images de test :	
![notebook_test_TOD_image1et2.png](img/notebook_test_TOD_image1et2.png)

	* La partie "__Instance Segmentation__" est plus gourmande en ressources (jusqu'à 8 Go de RAM) et dure de quelques dizaines de secondes à plusieurs dizaines de minutes suivant ton CPU ; elle utilise le réseau pré-entraîné `mask_rcnn_inception_resnet_v2_atrous_coco_2018_01_28` pour détecter les objets et leurs masques, par exemple :
![notebook_test_TOD_image-mask1.png](img/notebook_test_TOD_image-mask1.png)
