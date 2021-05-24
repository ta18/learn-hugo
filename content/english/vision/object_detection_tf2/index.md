---
title: "Object Detection with tensorflow2"
menu:
  main:
    name: "Object Detection with tensorflow2"
    weight: 3
    parent: "vision"
---

In this section we will use the __Tensorflow Object Detection__ (_a.k.a_ TOD) API which offers:

* a collection of pre-trained networks, specially designed for object detection in images (__Object Detection__),
* a _transfer learning_ mechanism to continue training pre-trained networks with our own labeled images,
to obtain the detection of the objects of interest.

Unlike the __Classification__ strategy presented in the [Classification tf2] section (<https://learn.e.ros4.pro/en/vision/classification_tf2/>),
the __Object detection__ can directly find the bounding boxes of objects such as "face with a 1" and "face with a 2":
this approach avoids the usage of conventional image processing to extract the faces of the cubes at first, then to classify the images of the cube faces.

The image processing used for the classification is based on a traditional approach for manipulating the pixels of the image (thresholding, contour extraction, segmentation, etc.).
It is quite fragile: sensitive to brightness, to the presence or not of a black background ...

An expected advantage of the Object Detection approach is to provide the bounding boxes of the faces of the cubes directly, without going through the image processing step.

## Prerequisites

* BAC + 2 and +
* Good understanding of Python and numpy
* A first experience of neural networks is desirable.

The training of neural networks with the `tensorflow` module will preferably be done in a Python virtual environment (PVE) which allows working in a Python environment separate from the existing one for working under ROS.

💻 Use the [Python FAQ: virtual environment] (<https://learn.e.ros4.pro/en/faq/python_venv/>) to create an PVE:

* named `tf2`,
* with a version of Python equal to `3.8`.

## 1. Documentation

1. General documentation on numpy:

  * [Numpy cheatsheet](https://s3.amazonaws.com/assets.datacamp.com/blog_assets/Numpy_Python_Cheat_Sheet.pdf)
  * [NumPy quickstart](https://numpy.org/devdocs/user/quickstart.html)

2. Documentation on the_TOD_ API for `tensorflow2`:

* The full official tutorial: [TensorFlow 2 Object Detection API tutorial](https://tensorflow-object-detection-api-tutorial.readthedocs.io/en/latest/index.html)
* The git repository: [models/research/object_detection](https://github.com/tensorflow/models/tree/master/research/object_detection) <br> <br>
The tutorial can be consulted to find details that are not developed in the proposed activity, but it is best to follow
the instructions in the present document to quickly install and use a recent version of tensorflow2.

3. Further reading:

* [1] [Zero to Hero: Guide to Object Detection using Deep Learning: Faster R-CNN, YOLO, SSD](https://cv-tricks.com/object-detection/faster-r-cnn-yolo-ssd /)
* [2] [mAP (mean Average Precision) for Object Detection](https://jonathan-hui.medium.com/map-mean-average-precision-for-object-detection-45c121a31173)
* [3] [Understanding SSD MultiBox - Real-Time Object Detection In Deep Learning](https://towardsdatascience.com/understanding-ssd-multibox-real-time-object-detection-in-deep-learning-495ef744fab) 

## 2. Install the TOD API

The installation of the TOD API takes place in 5 steps:

1. Create and initialize your workspace
2. Clone the `tensorflow/models` repository
3. Install the `protobuf` tools
4. Install the COCO API
5. Install the `object_detection` package.

Throughout the document the _prompt_ of the terminal will be noted `(tf2) user@host $`: the prefix `(tf2)` is there to remind you that the Python work for the TOD API is done
in the __Virtual Python tf2__ environment previously created (see the Prerequisites).

### 2.1 Create and initialize your workspace

The first step is to create the working directory `tod_tf2`, in which all the files will be created, and to position yourself in this directory which will be __the root folder of the project__:

```bash
(tf2) user@host $ cd <some_part> # choose the directory to create `tod_tf2`, for example" cd ~/catkins_ws "
(tf2) user@host $ mkdir tod_tf2
(tf2) user@host $ cd tod_tf2/
```

📥 Next, you clone the `cjlux/tod_tf2_tools.git` github repository and copy the `*.py` and `*.ipynb` files from the `tod_tf2_tools` folder to the `tod_tf2` folder:

```bash
# From tod_tf2/
(tf2) user@host $ git clone https://github.com/cjlux/tod_tf2_tools.git
(tf2) user@host $ cp tod_tf2_tools/*.py  .
(tf2) user@host $ cp tod_tf2_tools/*.ipynb  . 
```

### 2.2 Clone the `tensorflow/models` repository

📥 In the working directory `tod_tf2` clone the github repository `tensorflow/models` (~ 635 MB):

```bash
# From tod_tf2/
(tf2) user@host $ git clone https://github.com/tensorflow/models.git
```

You get a `models` folder. The TOD API is in the folder `models/research/object_detection/`:

```bash	
(tf2) user@host $ tree -d -L 2 .
.
└── models
    ├── community
    ├── official
    ├── orbit
    └── research
```

📥 Complete your installation with some Python packages useful for working with the TOD API:

```bash
(tf2) user@host $ conda install cython contextlib2 pillow lxml
(tf2) user@host $ pip install labelimg rospkg
```

Update the environment variable `PYTHONPATH` by adding the two lines at the end of the `~/.bashrc` file:

```bash
export TOD_ROOT="<absolute path to tod_tf2>"
export PYTHONPATH=$TOD_ROOT/models:$TOD_ROOT/models/research:$PYTHONPATH
```

replace `"<absolute path to tod_tf2>"` by th absolute path to the `tod_tf2` folder on your workstation.

* Launch a new terminal to activate the new shell environment: all the following will be done in this new terminal.
* ⚠️ don't forget to activate the `tf2` PVE in this new terminal:

```bash
user@host $ conda activate tf2
(tf2) user@host $
 ```

### 2.3 Install the `protobuf` tools 

The native TOD API uses `*.proto` files for configuring models and storing training parameters.
These files must be translated into `*.py` files in order for the Python API to work properly:

* First install the debian `protobuf-compile` package which gives access to the `protoc` command:

```bash
(tf2) user@host $ sudo apt install protobuf-compiler
```

* You can then go into the `tod_tf2/models/research` directory and enter:

```bash
# From tod_tf2/models/research/
(tf2) user@host $ protoc object_detection/protos/*.proto  --python_out=.
```

This command works silently.

### 2.4 Install the COCO API

COCO is a database intended to supply algorithms for object detection, segmentation... 
See [cocodataset.org](https://cocodataset.org) for tutorials and publications.

📥 To install COCO's Python API, clone the `cocoapi.git` site (~ 15 MB) in the `/tmp` folder, type the `make` command in the `cocoapi/PythonAPI` folder, then copy the folder `pycococtools` in your `.../models/research/` folder:

```bash
(tf2) user@host $ cd /tmp
(tf2) user@host $ git clone  https://github.com/cocodataset/cocoapi.git
(tf2) user@host $ cd cocoapi/PythonAPI/
(tf2) user@host $ make
(tf2) user@host $ cp -r pycocotools/ <absolute path to tod_tf2>/models/research/
```

replace `<absolute path to tod_tf2>` by th absolute path to the `tod_tf2` folder on your workstation (for example: `~/catkins_ws/tod_tf2`).

### 2.5 Install the package `object_detection`

At last go into the `models/research/` directory and enter:

```bash
# From tod_tf2/models/research/
(tf2) user@host $ cp object_detection/packages/tf2/setup.py .
(tf2) user@host $ python setup.py build
(tf2) user@host $ pip install .
```

### 2.6 Test tthe TOD API installation

To test your installation of the TOD API, go to the `models/research /` directory and type the command:

```bash	
# From within tod_tf2/models/research/
(tf2) user@host $ python object_detection/builders/model_builder_tf2_test.py
```

The program runs a whole series of tests and should end with an OK without any error:

```
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
```

Finally, you can verify the installation using the IPython notebook `object_detection_tutorial.ipynb` present in the `tod_tf2` directory. <br>
(note: this is a copy of the `tod_tf2/models/research/object_detection/colab_tutorials/object_detection_tutorial.ipynb` notebook in which we removed the installation cells of the TOD API and some other cells that can generate errors...).


* ⚠️ Before running the notebook cells, you must correct an error in the file `.../tod_tf2/models/research/object_detection/utils/ops.py`, line 825:
replace `tf.uint8` with `tf.uint8.as_numpy_dtype`

* In the `tod_tf2` directory run the `jupyter notebook` command and load the `object_detection_tutorial.ipynb` notebook.
* Run the cells one by one, you shouldn't get any mistakes:
	* The "__Detection__" part (which lasts from a few seconds to several minutes depending on your CPU…) uses the pre-trained network `ssd_mobilenet_v1_coco_2017_11_17` to detect objects in the test images:
![notebook_test_TOD_image1et2.png](img/notebook_test_TOD_image1et2.png)

	* The "__Instance Segmentation__" part is more resource intensive (up to 8 GB of RAM) and lasts from a few tens of seconds to several tens of minutes depending on your CPU; it uses the pre-trained `mask_rcnn_inception_resnet_v2_atrous_coco_2018_01_28` network to detect objects and their masks, for example:
![notebook_test_TOD_image-mask1.png](img/notebook_test_TOD_image-mask1.png) 

The rest of the work breaks down as follows:

* Complete the working tree
* Download the pre-trained network
* Create the labeled image bank for the supervised training of the chosen network
* Train the network with the labeled image bank.
* Evaluate network inferences with test images
* Integrate network operation into the ROS environment.

## 3. Complete the work tree

The proposed generic tree structure is as follows: 

```
tod_tf2
├── images
│   └──<project>
│       ├── test
│       │   └── *.jpg, *.png ... *.xml
│       ├── train
│       │   └── *.jpg, *.png ... *.xml
│       └── *.csv
├── pre_trained
│   └── <pre_trained-network>
├── training
│   └──<project>
│       ├── <pre_trained-network>
│       ├── train.record
│       ├── test.record
│       └── label_map.txt
└── models
    └── research
        └── object_detection
```

* Everything that is specific to the project is placed in a `<project>` directory at different levels.

* The `images/<project>` folder contains for each project:

  * the `test` and `train` folders which each contain:
    * the PNG, JPG ... images to analyze,
    * XML annotation files created with the `labelImg` software: they give, for each of the objects of an image, the coordinates of bounding box  and the label of the object.
  * CSV annotation files (content of XML files converted to CSV format), which will in turn be converted to _tensorflow record_ format.
* The `pre_trained/` folder contains a subfolder for each of the pre-trained networks used.
* the `training/<project>` folder contains for each project:
  * a folder for the pre-trained network used: it is in this folder that the weight files of the trained network are stored,
  * the `train.reccord` and `test.reccord` files: contain the labeled training and test data converted from CSV format to _tensorflow record_ format,
  * the `label_map.txt` file: lists the labels corresponding to the objects to be detected.

For the detection of the faces of the cubes in the images of the robot's camera, the `<project>` folder will be named `faces_cubes`, which gives the working tree:

```bash
tod_tf2
├── images
│   └── faces_cubes
│       ├── test
│       │   └── *.jpg, *.png ... *.xml
│       ├── train
│       │   └── *.jpg, *.png ... *.xml
│       └── *.csv
├── pre_trained
│   └── <pre_trained-network>
├── training
│   └── faces_cubes
│       ├─── <pre_trained-network>
│       ├── train.record
│       ├── test.record
│       └── label_map.txt
└── models
    └── research
        └── object_detection
```

A few shell commands are enough to create the first levels of this tree:

```bash
# From within tod_tf2
(tf2) user@host $ mkdir -p images/faces_cubes/test
(tf2) user@host $ mkdir -p images/faces_cubes/train
(tf2) user@host $ mkdir pre_trained
(tf2) user@host $ mkdir -p training/faces_cubes
```

Verification:

```bash
# From within tod_tf2
(tf2) user@host $ tree -d . -I models
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

## 4. Télécharger le réseau pré-entraîné

Plusieurs familles de réseaux dédiés à la détection d’objets sont proposés sur le site  [TensorFlow 2 Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md), parmi lesquelles :

* Les réseaux __R-CNN__ (_Region-based Convolutional Neural Network_) : basés sur le concept de __recherche ciblée__ (_selective search_). 
![R-CNN](img/R-CNN.png)(source: https://arxiv.org/pdf/1311.2524.pdf)<br>
Au lieu d’appliquer la sous-fenêtre d'analyse à toutes les positions possibles dans l’image, l’algorithme de recherche ciblée génère 2000 propositions de régions d’intérêts où il est le plus probable de trouver des objets à détecter. Cet algorithme se base sur des éléments tels que la texture, l’intensité et la couleur des objets qu’il a appris à détecter pour proposer des régions d’intérêt. Une fois les 2000 régions choisies, la dernière partie du réseau calcule la probabilité que l’objet dans la région appartienne à chaque classe. Les versions __Fast R-CNN__ et __Faster R-CNN__ rendent l’entraînement plus efficace et plus rapide.

* Les réseaux __SSD__ (_Single Shot Detector_) : font partie des détecteurs considérant la détection d’objets comme un problème de régression. L'algorithme __SSD__ utilise d’abord un réseau de neurones convolutif pour produire une carte des points clés dans l’image puis, comme __Faster R-CNN__, utilise des cadres de différentes tailles pour traiter les échelles et les ratios d’aspect.

La différence entre "Faster R-CNN" et SSD est qu’avec R-CNN on réalise une classification sur chacune des 2000 fenêtres générées par l’algorithme de recherche ciblée, alors qu’avec SSD on cherche à prédire la classe ET la fenêtre de l’objet en même temps. Cela rend SSD plus rapide que "Faster R-CNN", mais également moins précis.

Dans le tableau du site [TensorFlow 2 Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md), les performances des différents réseaux sont exprimées en _COCO mAP (Mean Average Precision)_, métrique couramment utilisée pour mesurer la précision d’un modèle de détection d’objets. Elle consiste à mesurer la proportion de détections réussies sur des images déjà annotées du dataset COCO (Common Object in CONtext)
qui contient 200 000 images annotées avec 80 objets différents. Cette mesure sert de référence pour comparer la précision de différentes architectures de détection d’objets (plus d’informations sur _mAP_ dans la lecture [2]).


📥 Pour le travail de détection des faces des cubes dans les images fournies par la caméra du robot Ergo Jr tu peux télécharger le réseau `Faster R-CNN ResNet50 V1 640x640` sur le site [TensorFlow 2 Detection Model Zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md) (~203 Mo).

Une fois téléchargée, il faut extraire l'archive TGZ au bon endroit dans l'arborescence de travail :
```bash
# From within tod_tf2/
(tf2) user@host $ tar xvzf ~/Téléchargements/faster_rcnn_resnet50_v1_640x640_coco17_tpu-8.tar.gz -C pre_trained
```
puis créer le dossier correspondant `faster_rcnn_resnet50_v1_640x640_coco17_tpu-8` dans le dossier `training/faces_cubes` :
```bash	
# From within tod_tf2/
(tf2) user@host $ mkdir training/faces_cubes/faster_rcnn_resnet50_v1_640x640_coco17_tpu-8
```
On vérifie :
```bash
# From within tod_tf2/
(tf2) user@host $ tree -d pre_trained
pre_trained
└── faster_rcnn_resnet50_v1_640x640_coco17_tpu-8
    ├── checkpoint
    └── saved_model
        └── variables
        
(tf2) user@host $ tree -d training
training
└── faces_cubes
    └── faster_rcnn_resnet50_v1_640x640_coco17_tpu-8
```

## 5. Créer les données pour l'apprentissage supervisé

Cette étape du travail comporte cinq tâches :

1. Créer des images avec la caméra du robot -> fichiers \*.jpg, \*.png
2. Annoter les images avec le logiciel `labelImg` -> fichiers \*.xml
3. Convertir les fichiers annotés XML au format CSV
4. Convertir les fichiers annotés CSV au format _tensorflow record_
5. Créer le fichier `label_map.pbtxt` qui contient les labels des objets à reconnaître.


### 5.1 Créer les images avec la caméra du robot  

Les images des faces des cubes peuvent être obtenues en utilisant le service ROS `/get_image` proposé par le robot Poppy Ergo Jr.

image001.png               |  image002.png
:-------------------------:|:-------------------------:
![image1](img/image000.png)   |  ![image2](img/image001.png)


🤖 Rappels : lancement du ROS Master et des services ROS sur le robot :
 
* allumer le robot Poppy Ergo Jr,
* se connecter sur la carte RPi du robot : `ssh pi@poppy.local` (mdp: `raspberry`) 
* ✅ vérifier que `ROS_MASTER_URI` pointe bien vers `poppy.local:11311` :

```bash
(tf2) jlc@pikatchou: $ ssh pi@poppy.local
pi@poppy.local password:
...

pi@poppy:~ $ env|grep ROS_MASTER
ROS_MASTER_URI=http://poppy.local:11311
```

* si `ROS_MASTER_URI` n'est pas bon, édite le fichier `~/.bashrc` du robot, mets la bonne valeur et tape `source ~\.bashrc`...
* Lance le ROS Master et les services ROS sur le robot avec la commande :

```bash
pi@poppy:~ $ roslaunch poppy_controllers control.launch
...
```

💻 Et maintenant dans un terminal sur ton PC, avec l'PVE `(tf2)` désactivé :
* ✅ vérifie que `ROS_MASTER_URI` pointe bien vers `poppy.local:11311` :

```bash
(tf2) jlc@pikatchou: $ env|grep ROS_MASTER
ROS_MASTER_URI=http://poppy.local:11311
```

* si `ROS_MASTER_URI` n'est pas bon, édite le fchier `~/.bashrc`, mets la bonne valeur et tape `source ~\.bashrc`...


🐍 Tu peux utiliser le programme Python `get_image_from_robot.py` du dossier `tod_tf2` pour enregistrer les images des cubes dans des fichiers nommées `imagesxxx.png` (`xxx` = `001`, `002`...). 
Un appui sur une touche clavier permet de passer d'une image à l'autre, un appui sur la touche `Q` permet de quitter le programme :

```python
import cv2, rospy
from poppy_controllers.srv import GetImage
from cv_bridge import CvBridge

i=1
while True:
    get_image = rospy.ServiceProxy("get_image", GetImage)
    response  = get_image()
    bridge    = CvBridge()
    image     = bridge.imgmsg_to_cv2(response.image)
    cv2.imwrite(f"image{i:03d}.png", image)
    cv2.imshow("Poppy camera", image)
    key = cv2.waitKey(0)
    if key==ord('q') or key==ord("Q"): break
    cv2.destroyAllWindows()
    i += 1
cv2.destroyAllWindows()
```

📍  En cas de conflit grave "ROS / PVE tf2 / PyQT" en utilisant le programme `get_image_from_robot.py` tu peux désactiver temporairement l'PVE tf2 :

* soit en lançant un nouveau terminal,
* soit en tapant la commande `conda deactivate`


Chaque équipe doit faire une dizaine d'images en variant les faces des cubes visibles, puis les images pourront être partagées sur un serveur pour servir à toutes les équipes.

Une fois collectées toutes les images, il faut mettre environ 90 % des images dans le dossier `images\faces_cubes\train` et le reste dans le dossier `images\faces_cubes\test`.

### 5.2 Annoter les images avec le logiciel labelImg

L'annotation des images peut être faite de façon très simple avec le logiciel `labelImg`.
C’est une étape du travail qui prend du temps et qui peut être réalisée à plusieurs en se répartissant les images à annoter...

L'installation du module Python `labelImg` faite dans l'PVE `tf2` (cf section 2.) permet de lancer le logiciel `labelImg` en tapant :
```bash
(tf2) jlc@pikatchou:~ $ labelImg
```

Utilise les boutons [Open Dir] et [Change Save Dir] pour te positionner la lecture ET l'écriture des fichiers dans le dossier `images/face_cubes/train/`.<br>
La première image est automatiquement chargée dans l'interface graphique :

![labelImg_2.png](img/labelImg_2.png)

Pour chaque image, tu dois annoter les objets à reconnaître :

* avec le bouton [Create RectBox], tu entoures une face d'un cube,
* la boîte des labels s'ouvre alors et tu dois écrire le blabel `one` ou `two` en fonction de la face entourée,
* itère le processus pour chacune des faces de cubes présente dans l'image...

    première face          |  deuxième face            |  fin
:-------------------------:|:-------------------------:|:-------------------------:
![1](img/labelImg_3.png)   |  ![2](img/labelImg_4.png) | ![3](img/labelImg_5.png)

* quand c'est fini, tu cliques sur le bouton [Save] et tu passes à l'image suivante avec le bouton [Next Image].
* Une fois toutes les images annotées, utilise les boutons [Open Dir] et [Change Save Dir] pour annoter de la même façon les images de test du dossier `images/face_cubes/test/`.

### 5.3 Convertir les fichiers XML annotés au format CSV

Cette étape permet de synthétiser dans un fichier CSV unique les données d’apprentissage contenues dans les différents fichiers XML crées à l’étape d'annotation. 
Le programme `xml_to_csv_tt.py` permet de générer les deux fichiers CSV correspondant aux données d’apprentissage et de test. <br>
Depuis le dossier `tod_tf2` tape la commande suivante :

```bash
# From within tod_tf2
(tf2) jlc@pikatchou:~ $ python xml_to_csv_tt.py -p faces_cubes
Successfully converted xml data in file <images/faces_cubes/train_labels.csv>.
Successfully converted xml data in file <images/faces_cubes/test_labels.csv>.
```

Les fichiers `train_labels.csv` et `test_labels.csv` sont créés dans le dossier  `images/faces_cubes/`.

### 5.4 Convertir les fichiers CSV annotés au format _tfrecord_

Pour cette étape, on utilise le programme `generate_tfrecord_tt.py`.<br>
Depuis le dossier `tod_tf2` tape la commande :

```bash
# From within tod_tf2
(tf2) jlc@pikatchou:~ $ python generate_tfrecord_tt.py --project faces_cubes
Successfully created the TFRecord file: ./training/faces_cubes/train.record
Successfully created the TFRecord file: ./training/faces_cubes/test.record
```

Avec cette commande tu viens de créer les 2 fichiers `train.record` et `test.record` dans le dossier `training/faces_cubes` : ce sont les fichiers qui serviront pour l’entraînement et l'évaluation du réseau.

### 5.5 Créer le fichier label_map.pbtxt

La dernière étape consiste a créer le fichier `label_map.pbtxt` dans le dossier `training/faces_cubes`.

Ce fichier décrit la « carte des labels » (_label map_) nécessaire à l’entraînement du réseau.
La carte des labels permet de connaître l’ID (nombre entier) associé à chaque étiquette (_label_) identifiant les objets à reconnaître. La structure type du fichier est la suivante :

```yaml
item {
   id: 1
   name: 'objet_1'
 }
 item {
   id: 2
   name: 'objet_2'
 }
 ...
 ```

Pour le projet `face_cubes`, le contenu du fichier `training/faces_cubes/label_map.pbtxt` à créer est :

```yaml
item {
  id: 1
  name: 'one'
}
item {
  id: 2
  name: 'two'
}
```

## 6. Lancer l'entraînement supervisé du réseau pré-entraîné

Ce travail se décompose en plusieurs étapes :

1. Modifier le fichier de configuration du réseau pré-entraîné pour décrire la configuration d'entraînement.
2. Lancer l'entraînement supervisé.
3. Exporter les poids du réseau entrainé dans un format utilisable.

### 6.1 Modifier le fichier de configuration

C’est la dernière étape avant de lancer l’entraînement…

* Le fichier de configuration `pipeline.config` présent dans le dossier `pre_trained/faster_rcnn_resnet50_v1_640x640_coco17_tpu-8` doit être copié dans le dossier cible `training/faces_cubes/faster_rcnn_resnet50_v1_640x640_coco17_tpu-8`. 

* Il faut ensuite modifier les paramètres du fichier `training/faces_cubes/faster_rcnn_resnet50_v1_640x640_coco17_tpu-8` pour les adpater à l'entraînement :

|n° | paramètre                     | Description                                                            | Valeur initiale  | valeur à donner |  explication                    |
|:--|:------------------------------|:-----------------------------------------------------------------------|:----------------:|:---------------:|:--------------------------------|
|010| `num_classes`                 | nombre de classe d'objets                                              | 90               | 2               | les deux classes 'one' et 'two' |
|077| `max_detections_per_class`    | nombre max de détection par classe                                     | 100              | 4               | 4 cubes          | 
|078| `max_total_detections`        | nombre max total de détections                                         | 100              | 4               | 4 cubes          | 
|093| `batch_size`                  | nombre d'images à traiter en lot avant mise à jour des poids du réseau | 64               | 1, 2,...        | une valeur trop élevée risque de faire dépasser la capacité mémoire RAM de ta machine... à régler en fonction de la quantité de RAM de ta machine.  |
|097| `num_steps`                   | Nombre max d'itérations d'entraînement                                 | 25000             | 1000           | une valeur trop grande donne des temps de calcul prohibitifs et un risque de sur-entraînement 
|113| `fine_tune_checkpoint`        | chemin des fichiers de sauvegarde des poids du réseau pré-entraîné     | "PATH_TO_BE_<br>CONFIGURED" | "pre_trained/faster_rcnn_resnet50_v1_<br>640x640_coco17_tpu-8/checkpoint/ckpt-0" | se termine par `/ckpt-0` qui est le préfixe des fichiers dans le dossier `.../checkpoint/` |
|114| `fine_tune_checkpoint_type`   | Choix de l'algorithme : "classification" ou "detection"                | "classification"| "detection"  | on veut faire de la detection d'objets |
|120| `max_number_of_boxes`         | Nombre max de boîtes englobantes  dans chaque image                    | 100               | 4               | les faces des cubes sur une image |
|122| `use_bfloat16`                | `true` pour les architectures TPU, `false` pour CPU                    | true              | false           | |
|126| `label_map_path`              | chemin du fichier des labels                                           | "PATH_TO_BE_<br>CONFIGURED" | "training/faces_cubes/label_map.pbtxt" | utilisé pour l'entraînement |
|128| `input_path`                  | fichier des données d'entrée d'entraînement au format `tfrecord`       | "PATH_TO_BE_<br>CONFIGURED" | "training/faces_cubes/train.record"    | utilisé pour l'entraînement |
|139| `label_map_path`              | chemin du fichier des labels                                           | "PATH_TO_BE_<br>CONFIGURED" | "training/faces_cubes/label_map.pbtxt" | utilisé pour l'évaluation|
|128| `input_path`                  | fichier des données d'entrée de test au format `tfrecord`              | "PATH_TO_BE_<br>CONFIGURED" | "training/faces_cubes/test.record"    | utilisé pour l'évaluation|

## 6.2 Lancer l'entraînement

* Copie le fichier `models\research\object_detection\model_main_tf2.py` dans la racine `tod_tf2`.
* Tape la commande :

```bash
# From within tod_tf2
(tf2) user@host $ python model_main_tf2.py --model_dir=training/faces_cubes/faster_rcnn_resnet50_v1_640x640_coco17_tpu-8/checkpoint1  --pipeline_config_path=training/faces_cubes/faster_rcnn_resnet50_v1_640x640_coco17_tpu-8/pipeline.config
```

Les fichiers des poids entraînés seront écrits dans le dossier `.../faster_rcnn_resnet50_v1_640x640_coco17_tpu-8/checkpoint1` : si tu relances l'entraînement, tu peux utiliser `.../checkpoint2`, `.../checkpoint3` pour séparer des essais successifs.

Le programme Python lancé est très verbeux...<br>
au bout d'un temps qui peut être assez long (plusieurs minutes avec un petit CPU), les logs de l'entraînement apparaissent à l'écran :

```bash
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
```

Dans l'exemple ci-dessus, on voit des logs tous les 100 pas, avec environ 20 secondes par pas, soit environ 35 minutes entre chaque affichage et environ 6h de calcul pour les 1000 pas.

En cas d'arrêt brutal du programme avec le message "Processus arrêté", ne pas hésiter à diminer la valeur du paramètre `batch_size` jusquà 2 voire 1 si nécessaire.... <br>
Même avec un `batch_size` de 2, le processus Python peut nécessiter jusqu'à 2 ou 3 Go de RAM pour lui tout seul, ce qui peut mettre certains portables en difficulté...

Une fois l'entraînement terminé tu peux analyser les statistiques d'entraînement avec `tensorboard` en tapant la commande :

```bash
# From within tod_tf2
(tf2) jlc@pikatchou:~ $ tensorboard --logdir=training/faces_cubes/faster_rcnn_resnet50_v1_640x640_coco17_tpu-8/checkpoint1/train
Serving TensorBoard on localhost; to expose to the network, use a proxy or pass --bind_all
TensorBoard 2.4.0 at http://localhost:6006/ (Press CTRL+C to quit)
...
```

`tensorflow` lance un serveur HHTP en local sur ta machine, et tu peux ouvrir la page `http://` avec un navigateur pour voir les courbes d'analyse en faisant CTRL + clic avec le curseur de la souris positionné sur le mot `http://localhost:6006/` :

![tensorflow](img/tensorboard.png)

Le logiciel tensorboard permet d'examiner l'évolution de statistiques caractéristiques de l'apprentissage.

### 6.3 Exporter les poids du réseau entraîné

On utilise le script Python `exporter_main_v2.py` du dossier `models/reasearch/object_detection/` pour extraire le __graph d'inférence__ entraîné et le sauvegarder dans un fichier `saved_model.pb` qui pourra être rechargé ultérieurement pour exploiter le réseau entraîneé :

```bash
# From within tod_tf2
(tf2) user@host $ cp models/research/object_detection/exporter_main_v2.py .
(tf2) user@host $ python exporter_main_v2.py --input_type image_tensor --pipeline_config_path training/faces_cubes/faster_rcnn_resnet50_v1_640x640_coco17_tpu-8/pipeline.config --trained_checkpoint_dir training/faces_cubes/faster_rcnn_resnet50_v1_640x640_coco17_tpu-8/checkpoint1 --output_directory training/faces_cubes/faster_rcnn_resnet50_v1_640x640_coco17_tpu-8/saved_model1
```

Le script Python créé le fichier `saved_model.pb` dans le dossier `.../faster_rcnn_resnet50_v1_640x640_coco17_tpu-8/saved_model1/saved_model` :

```bash
# From within tod_tf2
(tf2) jlc@pikatchou:~ $ tree training/
training/
└── faces_cubes
    └── faster_rcnn_resnet50_v1_640x640_coco17_tpu-8
        ├── checkpoint1
        │   ├── checkpoint
        │   ├── ckpt-1.data-00000-of-00001
        │   ├── ckpt-1.index
        │   └── train
        │       └── events.out.tfevents.1620391994.pikatchou.30554.1504.v2
        ├── pipeline.config
        ├── saved_model1
        │   ├── checkpoint
        │   │   ├── checkpoint
        │   │   ├── ckpt-0.data-00000-of-00001
        │   │   └── ckpt-0.index
        │   ├── pipeline.config
        │   └── saved_model
        │       ├── assets
        │       ├── saved_model.pb
        │       └── variables
        │           ├── variables.data-00000-of-00001
        │           └── variables.index
```

## 7. Évaluation du réseau entraîné

On va vérifier que le réseau entraîné est bien capable de détecter les faces des cubes en discriminant correctement les numéros écrits sur les faces.

Le script Python `plot_object_detection_saved_model.py` permet d'exploiter le réseau entraîné sur des images, les arguments sont :

* `-p` : le nom du projet
* `-m` : le chemin du dossier `.../saved/` contenant les fichiers des poids du réseau entraîné
* `-i` : le chemin du dossier des images ou le chemin du fichier image à analyser
* `-n` : le nombre max d'objets à détecter
* `-t` : le seuil (_threshold_) de détection exprimé en % (optionnel, valeur par défaut : 50 %).

Par exemple pour faire la détection des cubes des images de test avec le réseau qu'on vient d'entraîner :

```bash
# From within tod_tf2
(tf2) jlc@pikatchou: $ python plot_object_detection_saved_model.py -p faces_cubes -s training/faces_cubes/faster_rcnn_resnet50_v1_640x640_coco17_tpu-8/saved_model1/saved_model -i images/faces_cubes/test/ -n 4

Loading model...Done! Took 11.77 seconds

Running inference for images/faces_cubes/test/image016.png... [2 1 1 2]
[0.999408   0.99929774 0.9985869  0.99794155]
[[0.4046488  0.13016616 0.6338345  0.31058723]
 [0.40798646 0.56277716 0.63340956 0.7373474 ]
 [0.40612057 0.3360289  0.63908    0.5120028 ]
 [0.40730068 0.7692113  0.6340802  0.9632611 ]]

Running inference for images/faces_cubes/test/image018.png... [2 2 1 1]
[0.9995958  0.99956626 0.99756575 0.9960402 ]
[[0.4140944  0.62948036 0.6388739  0.7997428 ]
 [0.41462958 0.40451866 0.6399791  0.5834095 ]
 [0.41448513 0.19922832 0.63370967 0.36855492]
 [0.40865567 0.         0.63875306 0.16509269]]

Running inference for images/faces_cubes/test/image019.png... [2 2 1 1]
[0.99956614 0.99939644 0.9977343  0.99497354]
[[0.41152024 0.43164197 0.6439534  0.6055011 ]
 [0.41524586 0.65218127 0.6408151  0.8317957 ]
 [0.40619218 0.20989983 0.6326463  0.40074167]
 [0.40634462 0.         0.6389088  0.18785618]]

Running inference for images/faces_cubes/test/image017.png... [2 2 1 1]
[0.999482   0.99903905 0.9982924  0.99810815]
[[0.4101084  0.70229053 0.63589627 0.8758344 ]
 [0.4053984  0.05132582 0.6378698  0.23081933]
 [0.41257906 0.49104023 0.63610333 0.66249573]
 [0.40499112 0.29253355 0.63419634 0.46947986]]
```

Pour chaque image traitée on affiche ici :

* la liste des 4 labels des objets trouvé (1 ou 2)
* la liste des 4 probabilités de détection des objets
* la liste des 4 jeux de coordonnées normalisées des boîtes englobantes [ y x coin haut gauche puis y x coin bas droit]. 

Les images produites sont :

|   image016.png           |   image018.png               |            image019.png    |    image017.png
:-------------------------:|:----------------------------:|:--------------------------:|:------------------------------:
![1](img/infere_img01.png) |  ![2](img/infere_img02.png)  | ![3](img/infere_img03.png) | ![4](img/infere_img04.png)

## 8. Intégration

Une fois le réseau entraîné et évalué, si les résultats sont bons, "il ne reste plus qu'à" créer le fichier `nn.py` pour réaliser les traitements nécessaires à l'exploitation du réseau entraîné pour ton projet : le but est d'intégrer le réseau de neurones `nn`  dans le contexte ROS :

![intégration ROS](../../integration/ergo-tb-tf2/img/UML_integration.png)

1. Attendre que le paramètre ROS  `takeImage` passe à `True` et le remettre à `False`
2. Obtenir le fichier de l'image prise par la caméra du robot grâce au service ROS `/get_image`
3. Traiter l'image pour obtenir les labels et les boîtes englobantes des faces des cubes (penser à remettre les cubes dans le bon ordre...)
4. Et pour chaque cube : donner au paramètre ROS`label` la valeur du label du cube, mettre le paramètre ROS `RobotReady` à `False` et attendre que le paramètre rOS `RobotReady` repasse à True
