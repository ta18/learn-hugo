---
title: "Détection d'objets avec tensorflow2"
menu:
  main:
    name: "Détection d'objets avec tf2"
    weight: 3
    parent: "vision"
---

Dans cette section nous proposons d'utiliser l'API __Tensorflow Object Detection__ (_a.k.a_ TOD) qui propose :
* une collection de réseaux déjà entraînés spécialement conçus pour pour la détection d'objets dans des images (__Object Detection__),
* le mécanisme de _transfert learning_ pour continuer l'entraînement des réseaux pré-entraînés avec nos propres images labellisées, 
pour obtenir la détection des objets qui nous intéressent.

Contrairement à la stratégie de __Classification__ présentée dans dans la section [Classification tf2](https://learn.e.ros4.pro/fr/vision/classification_tf2/), 
la __Détection d'objets__ permet de trouver directement les boîtes englobantes des objets "face de cube avec un 1" et "face de cube avec un 2".

Cette approche évite la phase de traitement d'image classique pour extraire puis classifier les images des faces des cubes. 

Le traitement d'image basé sur une approche traditionnelle de manipulation des pixels de l'image (seuillage, extraction de contour, segmentation...) reste assez fragile : en particulier il est  sensible à la luminosité, à la présence ou non d'un fond noir... Un avantage attendu de l'approcje Object Detection est de fournir directement les boîtes englobantes des faces des cubes sans passer par une étapde de traitement d'image.

## Prérequis

* Bonne compréhension de Python et numpy
* Une première expérience des réseaux de neurones est souhaitable.

L'entraînement des réseaux de neurones avec le module `tensorflow` se fera de préférence dans un environnement virtuel Python (EVP) qui permet de travailler dans un environnement Python dédié.
Vous pouvez vous réferrer à la [FAQ Python : environnement virtuel](https://learn.e.ros4.pro/fr/faq/python_venv/) 

## 1. Documentation

Documentation générale sur numpy :
* [Numpy cheatsheet](https://s3.amazonaws.com/assets.datacamp.com/blog_assets/Numpy_Python_Cheat_Sheet.pdf)
* [NumPy quickstart](https://numpy.org/devdocs/user/quickstart.html)

Documentation sur l'_API TOD_ pour `tensorflow2` :
* Le tutoriel officiel complet : [TensorFlow 2 Object Detection API tutorial](https://tensorflow-object-detection-api-tutorial.readthedocs.io/en/latest/index.html)
* Le dépôt git : [models/research/object_detection](https://github.com/tensorflow/models/tree/master/research/object_detection)

Ce tutoriel peut être consulté pour aller chercher des détails qui ne sont pas développés dans l'activité proposée, mais il est préferrable de suivre 
les indications du paragraphe suivant pour installer une version récente de tensorflow2. 

## 2. Installation de l'API TOD

### 2.1 Clonage du dépôt `tensorflow/models`

Créé un dossier spécifique pour le travail avec l'API TOD en clonant le dépôt github `cjlux/tod_tf2.git` :

```bash
(tf2) jlc@pikatchou~$ cd <quelque_part>   # choisis le répertoire où cloner `tod_tf2.git`, par exemple : ~/catkin_ws/
(tf2) jlc@pikatchou~$ git clone https://github.com/cjlux/tod_tf2.git
```
Le clonage créé le dossier `tod_tf2` contenant des scripts Python qui seront utilisés plus tard. Ce dossier est la racine du projet.<br>
Dans le dossier `tod_tf2` clone le dépôt github `tensorflow/models` :
```bash
(tf2) jlc@pikatchou~$ cd tod_tf2
(tf2) jlc@pikatchou~$ git clone https://github.com/tensorflow/models.git
```

Tu obtiens un dossier `models` : l’API TOD est dans le dossier `models/research/object_detection` :
```bash	
(tf2) jlc@pikatchou~$ tree -d -L 2 .
.
└── models
    ├── community
    ├── official
    ├── orbit
    └── research
```	

Complète ton installation avec quelques paquets Python utiles pour le travail avec l'API TOD :
```bash
(tf2) jlc@pikatchou $ conda install cython contextlib2 pillow lxml
(tf2) jlc@pikatchou $ pip install labelimg rospkg
```
Mets à jour la variable d’environnement `PYTHONPATH` en ajoutant à la fin du fichier ~/.bashrc les deux lignes :
```bash
export TOD_TF2="<chemin absolu du dossier tod_tf2>"
export PYTHONPATH=$TOD_TF2/models:$TOD_TF2/models/research:$PYTHONPATH
```
remplace `"<chemin absolu du dossier tod_tf2>"` par le chemin absolu du dossier `tod_tf2` sur ta machine.

Lance un nouveau terminal pour activer le nouvel environnement shell ; tout ce qui suit sera fait dans ce nouveau terminal.

### 2.2 Installer les outils `protobuf`

L’API native TOD utilise des fichiers `*.proto` pour la configuration des modèles et le stockage des paramètres d’entraînement. 
Ces fichiers doivent être traduits en fichiers `*.py` afin que l’API Python puisse fonctionner correctement.  

Du dois installer en premier la commande `protoc` :
```bash
(tf2) jlc@pikatchou $ sudo apt install protobuf-compiler
```
Tu peux ensuite te positionner dans le dossier `tod_tf2/models/research` et taper :
```bash
# From tod_tf2/models/research/
(tf2) jlc@pikatchou $ protoc object_detection/protos/*.proto  --python_out=.
```
Cette commande travaille de façon muette.

### 2.3 Installer l'API COCO

COCO est une banque de données destinée à alimenter les algorithmes de détection d’objets, de segmentation… voir [cocodataset.org](https://cocodataset.org) pour les tutoriels, publications… 

💻 Pour installer l’API Python de COCO, clone le site cocoapi dans le dossie `/tmp`, tape la commande `make` dans le dossier `cocoapi/PythonAPI`, puis recopie le dossier `pycococtools` dans `models/research/` :
```bash
(tf2) jlc@pikatchou~$ cd /tmp
(tf2) jlc@pikatchou~$ git clone  https://github.com/cocodataset/cocoapi.git
(tf2) jlc@pikatchou~$ cd cocoapi/PythonAPI/
(tf2) jlc@pikatchou~$ make
(tf2) jlc@pikatchou~$ cp -r pycocotools/ <chemin absolu du dossier tod_tf2>/models/research/
```
### 2.4 Finalisation 

💻 Pour finir l'installation, place-toi dans le dossier  `models/research/` et tape les commandes :
```bash
# From tod_tf2/models/research/
(tf2) jlc@pikatchou $ cp object_detection/packages/tf2/setup.py .
(tf2) jlc@pikatchou $ python setup.py build
(tf2) jlc@pikatchou $ pip install .
```

### 2.5 Tester l'installation de l'API TOD

💻 Pour tester ton installation de l’API TOD, place-toi dans le dossier `models/research/` et tape la commande :
```bash	
# From within tod_tf2/models/research/
(tf2) jlc@pikatchou~$ python object_detection/builders/model_builder_tf2_test.py
```
Le programme déroule toute une série de tests et doit se terminer par un OK :

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

💻 Pour finir, tu peux vérifier l’installation en utilisant un cahier IPython de l'API TOD  : dans le dossier `models/research/object_detection` lance la commande jupyter notebook et charge le cahier `colab/object_detection_tutorial.ipynb` : 

* ⚠️ Avant d'exécuter les cellules du notebook, il faut corriger une erreur dans le fichier `.../miniconda3/envs/tf2/lib/python3.8/site-packages/object_detection/utils/ops.py`, ligne 825 :
remplacer `tf.uint8` par `tf.uint8.as_numpy_dtype`
* ⚠️ Attention à ne pas excécuter les cellules du notebook comportant les commandes `!pip install ...` ou qui contiennent la _magic chain_ `%%bash` :	

![notebook_test_TOD.png](img/notebook_test_TOD.png)

Exécute les cellules une à une, tu ne dois pas avoir d’erreur :

* La partie "__Detection__" (qui dure quelques secondes ou quelques minutes suivant ton CPU…) utilise le réseau pré-entraîné `ssd_mobilenet_v1_coco_2017_11_17` pour détecter des objets dans les   images de test :	

![notebook_test_TOD_image1et2.png](img/notebook_test_TOD_image1et2.png)

* La partie "__Instance Segmentation__" utilise le réseau pré-entraîné `mask_rcnn_inception_resnet_v2_atrous_coco_2018_01_28` pour détecter les objets et leurs masques, par exemple :

![notebook_test_TOD_image-mask1.png](img/notebook_test_TOD_image-mask1.png)

La suite du travail se décompose ainsi :
* Créer l'arborescence de travail
* Télécharger le réseau pré-entraîné
* Créer la banque d'images labellisées pour l'entraînement supervisé du réseau choisi
* Entraîner le réseau avec la banque d'images labellisées.
* Évaluer les inférences du réseau avec les images de test
* Intégrer de l'exploitation du réseau dans l'environnement ROS.

## 3. Créer l'arborescence de travail

L'arborescence générique de travail proposée pour cette activité est la suivante :

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
	│   ├──<project>
	│   │   └── <pre_trained-network>
	│   ├── train.record
	│   ├── test.record
	│   └── label_map.txt
	└── models
	    └── research
	        └── object_detection
	
	

* Le dossier `images/` contient un dossier pour chaque projet, avec dedans :
	* les dossiers `test` et `train` qui contiennent chacun :
		* les images (\*.png, \*.jpg) à analyser,
		* les fichiers d'annotation (\*.xml) faits avec le logiciel `labelImg` : ils donnent, pour chacun des objets d'une image, les coordonnées de la boîte englobante et le label de l'objet.
	* les fichiers d'annotation \*.cvs (convertis au format CSV), qui seront à leur tour convertis au format _tensorflow record_.
* Le dossier `pre_trained/` contient un sous-dossier pour chacun des réseaux pré-entrainés utilisé.
* le dossier `training/` contient  également un dossier pour chaque projet, avec dedans :
	* un dossier pour chacun des réseaux pré-entrainés utilisé : c'est dans ce dossier que seront stockés les fichiers des poids du réseau entraîné,
	* les fichiers `train.reccord`  et `test.reccord` : contiennent les données labelisées d'entraînement et de test converties du format CSV au format _tensorflow record_,
	* le fichier `label_map.txt` : liste les labels correpondants aux objets à détecter.
	
Pour la détection des faces des cubes dans les images faites par le robot Poppy Ergo Jr, le dossier `<project>` sera nommé `faces_cubes`, ce qui donne l'arborescence de travail :

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
	│   ├── faces_cubes
	│   │   └── <pre_trained-network>
	│   ├── train.record
	│   ├── test.record
	│   └── label_map.txt
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
	 
## 4. Télécharger le réseau pré-entraîné

Deux grandes familles de réseaux dédiées à la détection d’objets dans des images sont proposés sur Le dépôt git [TensorFlow 2 Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md) :

* Les réseaux __R-CNN__ (_Region-based Convolutional Neural Network_) : basés sur le concept de __recherche ciblée__ (_selective search_). Au lieu d’appliquer une seule fenêtre à toutes les positions possibles de l’image, l’algorithme de recherche ciblée génère 2000 propositions de régions d’intérêts où il est le plus probable de trouver des objets à détecter. Cet algorithme se base sur des éléments tels que la texture, l’intensité et la couleur des objets qu’il a appris à détecter pour proposer des régions d’intérêt. Une fois les 2000 régions choisies, la dernière partie du réseau produit la probabilité que l’objet dans la région appartienne à chaque classe.
Il existe également des versions __Fast R-CNN__ et __Faster R-CNN__ qui permettent de rendre l’entraînement plus rapide;

* Les réseaux __SSD__ (_Single Shot Detector_) : font partie des détecteurs considérant la détection d’objets comme un problème de régression les plus connus. L'algorithme __SSD__ utilise d’abord un réseau de neurones convolutif pour produire une carte des points clés dans l’image puis, comme __Faster R-CNN__, utilise des cadres de différentes tailles pour traiter les échelles et les ratios d’aspect.

La différence entre Faster R-CNN et SSD est qu’avec R-CNN on réalise une classification sur chacune des 2000 fenêtres générées par l’algorithme de recherche ciblée, alors qu’avec SSD on cherche à prédire la classe ET la fenêtre de l’objet, en même temps. SSD apprend les décalages à appliquer sur les cadres utilisées pour encadrer au mieux l’objet plutôt que d’apprendre les fenêtres en elle-mêmes (ce que fait Faster R-CNN). Cela rend SSD plus rapide que Faster R-CNN, mais également moins précis.

📥 Pour le travail de reconnaissance des faces des cubes dans les images fournies par la caméra du robot Ergo Jr tu peux télécharger le réseau `SSD MobileNet V1 FPN 640x640` sur le site [TensorFlow 2 Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md).

Une fois téléchargé, il faut extraire l'archive TGZ au bon endroit de l'arborescence de travail :
```bash
# From within tod_tf2
(tf2) jlc@pikatchou $ tar xvzf ~/Téléchargements/ssd_mobilenet_v1_fpn_640x640_coco17_tpu-8.tar.gz -C pre_trained
```
puis créer le dossier `ssd_mobilenet_v1_fpn_640x640_coco17_tpu-8` dans le dossier `training/faces_cubes` :
```bash	
(tf2) jlc@pikatchou $ mkdir training/faces_cubes/ssd_mobilenet_v1_fpn_640x640_coco17_tpu-8
```
On vérifie :

	# From within tod_tf2
	(tf2) jlc@pikatchou $ tree -d pre_trained
	pre_trained
	└── ssd_mobilenet_v1_fpn_640x640_coco17_tpu-8
	    ├── checkpoint
	    └── saved_model
	        └── variables
	        
	(tf2) jlc@pikatchou $ tree -d training
	training
	└── faces_cubes
	    └── ssd_mobilenet_v1_fpn_640x640_coco17_tpu-8


## 4. Création des données pour l'apprentissage supervisé

Ce travail se décompose en plusieurs étapes

1. Création des images avec la caméra du robot -> fichiers \*.jpg, \*.png
2. Annotation des images avec le logiciel labelImg -> fichiers \*.xml
3. Conversion des fichiers annotés \*.xml au format CSV
4. Conversion des fichiers annotés CSV au format _tensorflow record_
5. Créer du fichier `label_map.pbtxt` qui contient les labels des objets à reconnaître.


### 4.1 Création des images avec la caméra du robot  

Les images des faces des cubes peuvent être faites en utilisant le service ROS `/get_image` proposé par le robot Poppy Ergo Jr.

image001.png               |  image002.png
:-------------------------:|:-------------------------:
![image1](img/image.png)   |  ![image2](img/image001.png)


🤖 Rappels : lancement du ROS Master et des services ROS sur le robot :
 
* allumer le robot Poppy Ergo Jr,
* se connecter sur la carte RPi du robot : `ssh pi@poppy.local` (mdp: `raspberry`) 
* ✅ vérifier que `ROS_MASTER_URI` pointe bien vers `poppy.local:11311` :
```
pi@poppy:~ $ env|grep ROS_MASTER
ROS_MASTER_URI=http://poppy.local:11311
```	
* si `ROS_MASTER_URI` n'est pas bon, édite le fichier `~/.bahrc` du robot et mets la bonne valeur...
* Lance le ROS Master et les services ROS sur le robot avec la commande : `roslaunch poppy_controllers control.launch`

💻 Et maintenant dans un terminal sur ton PC :
* ✅ vérifie que `ROS_MASTER_URI` pointe bien vers `poppy.local:11311` :
```bash
(tf2) jlc@pikatchou:~ $ env|grep ROS_MASTER
ROS_MASTER_URI=http://poppy.local:11311
```	
* si `ROS_MASTER_URI` n'est pas bon, édite ton fchier `~/.bahrc` entmets la bonne valeur...


🐍 Tu peux maintenant utiliser le programme Python `get_image_from_ergo.py` pour créer des images nommées `imagesxxx.png` (`xxx` = `001`, `002`...) avec l'appui sur la touche Enter pour passer d'une prise d'image à l'autre :
```python
import cv2, rospy
from poppy_controllers.srv import GetImage
from cv_bridge import CvBridge

i=1
while True:
    get_image = rospy.ServiceProxy("get_image", GetImage)
    response = get_image()
    bridge = CvBridge()
    image = bridge.imgmsg_to_cv2(response.image)
    cv2.imwrite(f"image{i:03d}.png", image)
    cv2.imshow("Poppy camera", image)
    rep = input("Enter...")
    #cv2.waitKey(0)
    i += 1
```

si tu obtiens l'erreur : `ModuleNotFoundError: No module named 'rospkg'`, il faut simplement ajouter le module Python `rospkg` à ton EVP `tf2` :
```bash
(tf2) jlc@pikatchou:~ $ pip install rospkg
```


Chaque équipe peut faire quelques dizaines d'images variant les faces des cubes visibles puis les images peuvent être déposées sur un serveur pour servir à toutes les équipes.

Une fois collectées toutes les images, il faut mettre environ 90 % des images dans le dossier `images\faces_cubes\train` et le reste des images dans le dossier `images\faces_cubes\test`.

### 4.2 Annoter des images avec le logiciel labelImg

L'annotation des images peut être faite de façon très simple avec le logiciel `labelImg`.
C’est une étape du travail qui prend du temps et qui peut être réalisée à plusieurs en se répartissant les images.

L'installation du module Python `labelImg` faite dans l'EVP `tf2` (cf section 2.) permet de lancer le logiciel `labelImg` en tapant :
```bash
(tf2) jlc@pikatchou:~ $ labelImg
```

Utilise les boutons [Open Dir] et [Change Save Dir] pour te positionner dans le dossier `images/face_cubes/train/`.<br>
La première image est automatiquement chargée dans l'interface graphique :

![labelImg_2.png](img/labelImg_2.png)

Pour chaque image, tu dois annoter les objets à reconnaître :
* avec le bouton [Create RectBox], tu entoures une face de cube,
* la boite des label s'ouvre alors et tu dois écrire le blabel `one` ou `two` en fonction de la face sélectionnée,
* itère le processus pour chacune des faces de cubes présente dans l'image...

    première face          |  deuxième face            |  fin
:-------------------------:|:-------------------------:|:-------------------------:
![1](img/labelImg_3.png)   |  ![2](img/labelImg_4.png) | ![3](img/labelImg_5.png)

* quand c'est fini, tu cliques sur le bouton [Save] et tu passes à l'image suivante avec le bouton [Next Image].
* Une fois toutes les images annotées, utilise les boutons [Open Dir] et [Change Save Dir] pour travailler avec les images du dossier `images/face_cubes/test/` et annote toutes les images de test.

### 4.3 Convertir les fichiers XML annotés au format CSV

Cette étape permet de synthétiser dans un fichier CSV unique les données d’apprentissage contenues dans les différents fichiers XML crées à l’étape d'annotation. 
Le programme `tod_tf2/xml_to_csv_tt.py` permet de générer les deux fichiers CSV correspondant aux données d’apprentissage et de test. 
Depuis le dossier `tod_tf2` tape la commande suivante :

```bash
# From within tod_tf2
(tf2) jlc@pikatchou:~ $ python xml_to_csv_tt.py -p faces_cubes
Successfully converted xml data in file <images/faces_cubes/train_labels.csv>.
Successfully converted xml data in file <images/faces_cubes/test_labels.csv>.
```
Les fichiers `train_labels.csv` et `test_labels.csv` sont créés dans le dossier  `images/faces_cubes/` :

### 4.4 Convertir des fichiers CSV annotés au format _tfrecord_

Pour cette étape, on utilise le programme `tod_tf2/generate_tfrecord_tt.py`. Depuis le dossier `tod_tf2` tape la commande :
```bash
# From within tod_tf2
(tf2) jlc@pikatchou:~ $ python generate_tfrecord_tt.py --project faces_cubes
Successfully created the TFRecord file: ./training/faces_cubes/train.record
Successfully created the TFRecord file: ./training/faces_cubes/test.record
```
Avec cette commande on vient de créer les 2 fichiers `train.record` et `test.record` dans le dossier `training/faces_cubes` : ce sont ces deux fichiers qui serviront pour l’entraînement du réseau.

### 4.5 Créer le fichier label_map.pbtxt
 
La dernière étape consiste a créer le fichier `label_map.pbtxt` dans le dossier `training/faces_cubes`. Ce fichier décrit la « carte des labels » (label map) nécessaire à l’entraînement du réseau. 
La carte des labels permet de connaître l’ID (nombre entier) associé à chaque étiquette (_label_) identifiant les objets à reconnaître. La structure type du fichier est la suivante :


	 item {
	   id: 1
	   name: ‘objet_1’
	 }
	 item {
	   id: 2
	   name: ‘objet_2’
	 }
	 ...

Pour le projet `face_cubes`, le contenu du fichier `training/faces_cubes/label_map.pbtxt` à créer est :

	 item {
	   id: 1
	   name: ‘one’
	 }
	 item {
	   id: 2
	   name: ‘two’
	 }

## 5. Entraînement supervisé du réseau pré-entraîné

Ce travail se décompose en deux étapes

1. Créer le fichier de configuration de l'entraînement.
2. Lancer l'entraînement supervisé.
3. Exporter les poids du réseau entrainé dans un format utilisable.



### 5.1 Création du fichier de configuration de l'entraînement.

C’est la dernière étape avant de pouvoir lancer l’entraînement…

Le fichier de configuration `pipeline.config` présent dans le dossier du réseau pré-entraîné `pre_trained/ssd_mobilenet_v1_fpn_640x640_coco17_tpu-8` doit être copié dans le dossier `training/faces_cubes\ssd_mobilenet_v1_fpn_640x640_coco17_tpu-8`. 

Il faut ensuite modifier les paramètres pour l’entraînement et le test en fonction : 
* ligne 003 -- remplacer `num_classes: 90` par `num_classes: 2`, puisqu'on n'a que deux classes pour le projet `faces_cubes`
* ligne 103 -- remplacer `max_detections_per_class: 100` par `max_detections_per_class: 5:set nu
`
* ligne 104 -- remplacer `max_total_detections: 100` par `max_total_detections: 5`

* ligne 131 -- remplacer `batch_size: 64` par `batch_size: 4` : c'est le nombre d'image traitée avant de mettre à jour les poids du réseau de neurones.<br>
Si cette valeur est trop grande, le calcul risque de dépasser la capacité mémoire RAM de ta machine... à régler en focntion de la quantité de RAM de ta machine.
* ligne 161 -- remplacer `fine_tune_checkpoint: "PATH_TO_BE_CONFIGURED"` par `fine_tune_checkpoint: "pre_trained/ssd_mobilenet_v1_fpn_640x640_coco17_tpu-8/checkpoint/ckpt-0", c'est le chemin du dossier pré-entrainé qui contient les poids du réseau après le pré-entraînement.
* ligne 162 -- remplacer `num_steps: 25000` par `num_steps: 1000`; c'est le nombre max d'itérations de l'entraînement, pour des machines avec un CPU peu puissant on n'ira peut être même pas jusqu'à 1000, ce serait trop long...
* ligne 165 -- remplacer `max_number_of_boxes: 100` par `max_number_of_boxes: 5`; on n'a pas plus de 5 faces de cubes sur une images, ça économise de la RAM.
* ligne 167 -- remplacer `fine_tune_checkpoint_type: "classification"` par `fine_tune_checkpoint_type: "detection"` pour activer l'algorithme de détection.
* ligne 172 -- remplacer `input_path: "PATH_TO_BE_CONFIGURED"` par `input_path: "./training/faces_cubes/"`, c'est le chemin du dossier contenant le fichier `train.record`.
* ligne 181 -- remplacer `label_map_path: "PATH_TO_BE_CONFIGURED"` par `label_map_path: "./training/faces_cubes/"`, c'est le chemin du dossier contenant le fichier `label_map.pbtxt`.
* ligne 185 -- remplacer `input_path: "PATH_TO_BE_CONFIGURED"` par `input_path: "./training/faces_cubes/"`, c'est le chemin du dossier contenant le fichier `test.record`.

## 5.2 Lancer l'entraînement

* Copie le fichier `models\research\object_detection\model_main_tf2.py` dans la racine `tod_tf2`.
* Place toi à la racine du projet dans le dossier `tod_tf2`.
* Tape la commande :
```bash
# From within tod_tf2
(tf2) jlc@pikatchou:~ $ python model_main_tf2.py --model_dir=training/faces_cubes/ssd_mobilenet_v1_fpn_640x640_coco17_tpu-8/checkpoint1  --pipeline_config_path=training/faces_cubes/ssd_mobilenet_v1_fpn_640x640_coco17_tpu-8/pipeline.config 
```
Les fichiers des poids entraînés seront écrits dans le dossier `.../ssd_mobilenet_v1_fpn_640x640_coco17_tpu-8/checkpoint1` : si tu relances l'entraînement, tu peux utiliser `.../checkpoint2`, `.../checkpoint3` pour séparés des essais successifs.

Le programme Python lancé est très verbeux... <br>
au bout d'un temps qui peut être assez long (plusieurs minutes avec un petit CPU), les logs de l'entraînement apparaissent à l'écran :

	...
	...
	W0507 00:24:41.010936 140206908888832 deprecation.py:531] From /home/jlc/miniconda3/envs/tf2/lib/python3.8/site-packages/tensorflow/python/util/deprecation.py:605: calling map_fn_v2 (from tensorflow.python.ops.map_fn) with dtype is deprecated and will be removed in a future version.
	Instructions for updating:
	Use fn_output_signature instead
	INFO:tensorflow:Step 100 per-step time 22.002s loss=0.825
	I0507 01:01:11.942076 140208909420352 model_lib_v2.py:676] Step 100 per-step time 22.002s loss=0.825
	INFO:tensorflow:Step 200 per-step time 20.926s loss=0.813
	I0507 01:36:04.090147 140208909420352 model_lib_v2.py:676] Step 200 per-step time 20.926s loss=0.813
	INFO:tensorflow:Step 300 per-step time 20.803s loss=0.801
	I0507 02:10:44.351419 140208909420352 model_lib_v2.py:676] Step 300 per-step time 20.803s loss=0.801
	INFO:tensorflow:Step 400 per-step time 20.946s loss=0.812
	I0507 02:45:38.927271 140208909420352 model_lib_v2.py:676] Step 400 per-step time 20.946s loss=0.812
	INFO:tensorflow:Step 500 per-step time 20.960s loss=0.794
	I0507 03:20:34.990385 140208909420352 model_lib_v2.py:676] Step 500 per-step time 20.960s loss=0.794
	INFO:tensorflow:Step 600 per-step time 21.045s loss=0.802
	I0507 03:55:39.516442 140208909420352 model_lib_v2.py:676] Step 600 per-step time 21.045s loss=0.802
	INFO:tensorflow:Step 700 per-step time 20.863s loss=0.786
	I0507 04:30:25.868283 140208909420352 model_lib_v2.py:676] Step 700 per-step time 20.863s loss=0.786
	INFO:tensorflow:Step 800 per-step time 20.744s loss=0.799
	I0507 05:05:00.163027 140208909420352 model_lib_v2.py:676] Step 800 per-step time 20.744s loss=0.799
	INFO:tensorflow:Step 900 per-step time 20.825s loss=0.837
	I0507 05:39:42.691898 140208909420352 model_lib_v2.py:676] Step 900 per-step time 20.825s loss=0.837
	INFO:tensorflow:Step 1000 per-step time 20.789s loss=0.778
	I0507 06:14:21.503472 140208909420352 model_lib_v2.py:676] Step 1000 per-step time 20.789s loss=0.778

Dans l'exemple ci-dessus, on voit des logs tous les 100 pas, avec environ 20 secondes par pas, soit environ 35 minutes entre chaque affichage et environ 6h de calcul pour les 1000 pas.

En cas d'arrêt brutal du programme avec le message "Processus arrêté", ne pas hésiter à diminer la valeur du paramètre `batch_size` jusquà 2 si nécessaire.... <br>
Même avec un `batch_size` de 2, le processus Python peut nécessiter jusqu'à 2 ou 3 Go de RAM pour lui tout seul, ce qui peut mettre certains portables en difficulté...

Une fois l'entraînement terminé tu peux analyser les statistiques d'entraînement avec `tensorflow` en tapant la commande :
```bash
# From within tod_tf2
(tf2) jlc@pikatchou:~ $ tensorflow --log_dir=training/faces_cubes/ssd_mobilenet_v1_fpn_640x640_coco17_tpu-8/checkpoint1/train
```
`tensorflow` lance un serveur HHTP en local sur ta machine, et tu peux ouvrir la page `http://` avec un navigateur pour voir les courbes d'analyse :

![tensorflow]()

### 5.3 Exporter les poids du réseau entraîné

On utilise le script Python `exporter_main_v2.py` du dossier `models/reasearch/object_detection/` pour extraire le __graph d'inférence__ entraîné et le sauvegarder dans un fichier `saved_model.pb` qui pourra être rechargé ultérieurement pour exploiter le réseau entraîneé :
```bash
# From within tod_tf2
p(tf2) jlc@pikatchou:~ $ python models/research/object_detection/exporter_main_v2.py --input_type image_tensor --pipeline_config_path training/faces_cubes/ssd_mobilenet_v1_fpn_640x640_coco17_tpu-8/pipeline.config --trained_checkpoint_dir training/faces_cubes/ssd_mobilenet_v1_fpn_640x640_coco17_tpu-8/checkpoint1 --output_directory training/faces_cubes/ssd_mobilenet_v1_fpn_640x640_coco17_tpu-8/saved1

```
Le script Python créé le fichier `saved_model.pb` dans le dossier `.../ssd_mobilenet_v1_fpn_640x640_coco17_tpu-8/saves1/saved_model` :





## 6. Évaluation du réseau entraîné

On va vérifier que le réseau entraîné est bien capables de détecter les faces des cubes en discriminant correctement les numéros écrits sur les faces.

Le script Python `plot_object_detection_saved_model.py` permet d'exploiter le réseau entraîné sur des images, les arguments supportés sont :
* `-p` : le nom du projet
* `-m` : le nom du modèle du réseau
* `-i` : le chemin du fichier image à analyser
* `-n` : optionnel, le nombre de classes d'objets à détecter (valeur par défaut : )

```bash
# From within tod_tf2
p(tf2) jlc@pikatchou:~ $ python plot_object_detection_saved_model.py -p faces_cubes -m ssd_mobilenet_v1_fpn_640x640_coco17_tpu-8 -i images/faces_cubes/test/image017.png -n 2
```


## 7. Intégration

Une fois le réseau entraîné et évalué, si les résultats sont bon, il ne reste plus qu'à modifier le fichier `xxx.py` pour réliser les traitements suivants :

## 7.1 

1. Instancier un réseau en chargeant les poids du réseau entraîné. 
2. Utiliser le service ROS `/get_image` pour obtenir l'image faite par la caméra du robot Ergo Jr,
2. Détecter avec le réseau entraîné les faces des cubes avec leur numéro
3. Faire afficher le résulat de la détection


* augmenter/diminuer `BATCH_SIZE` peut modifier les temps de calcul et la qualité du réseau entraîné...

Pour confirmer la qualité de votre réseau entraîné vous pouvez enregistrer vos propres fichiers PNG avec les images faites avec la caméra du robot en utilisant le service ROS `/get_image`. 

Lancer le programme et observer les performances de votre réseau opérant sur vos propres images.

