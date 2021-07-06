---
title: "Convertir un réseau Tensorflow au format TFLite"
menu:
  main:
    name: "Convertir un réseau Tensorflow au format TFLite"
    weight: 3
    parent: "capsulesRSP"
---


| Classe de capsule  | &emsp;Durée recommandée |
|:-------------------|:------------------|
| Task  &emsp;  ⚙️  |&emsp; 10 min      |

## 🎒 Prérequis

* BAC+2 et +
* Bonne compréhension de Python et numpy
* Une première expérience des réseaux de neurones est souhaitable
* Une raspberry Pi avec caméra fonctionnelle
* Capsule sur la **Mise en place des modules sur la Raspberry Pi**
* Capsule sur la **Détection d'objet sur la Raspberry Pi**

## 🎓 Acquis d'apprentissage

* savoir convertir un réseau Tensorflow ré-entraîné vers un réseau au format Tensorflow Lite.

## 📗 Documentation

Credit : 
* https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/running_on_mobile_tf2.md
* https://www.tensorflow.org/lite/convert
* https://coral.ai/docs/edgetpu/models-intro/

## Introduction


TensorFlow Lite est une version allégée de TensorFlow, conçue pour les mobiles 
et les objets embarqués. TensorFlow Lite permet une inférence à faible latence
avec une faible taille binaire.
La taille des réseaux de neurones peut être encore réduite grâce à la quantification, qui 
convertit les paramètres de 32 bits en des représentations sur 8 bits. 

Dans la capsule **Détection d'objet sur la Raspberry Pi**, un réseau au format Tensorflow Lite est utilisé :
il possède un fichier **detect.tflite** et un **labelmap.txt**.

Pour l'entraînement personnalisé d'un réseau Tensorflow, l'état de l'art consite à utiliser une machine avec des 
ressources importantes (RAM, CPU, GPU, acccélérateur graphique...), puis de convertir le réseau entrainé 
au format Lite afin de pouvoir l'utiliser sur une Raspberry Pi ou autre. On n'a aucun intérêt à entraîner un 
réseau directement sur une Raspberry Pi, du fait du manque de mémoire et du processeur ARM.

La conversion fournit deux fichiers `detect.tflite` et `labelmap.txt`.
Ces deux fichiers doivent ensuite être intégrés dans le répertoire du projet 
sur la Raspberry Pi.

La conversion au format TFlite peut s'accompagner d'une dégradation des performances de détection qui peut conduire
à utiliser plus d'images labellisées pour compenser cette dégradation.

Dans la capsule sur la reconnaissance d'objet avec Tensorflow, un réseau pré-entrainé Faster R-CNN est utilisé. 
Ce format n'est pas supporté par la conevrsion TFLite.

Parmi les réseaux existant ([TensorFlow 2 Detection Model Zoo]{https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/tf2_detection_zoo.md} on retient les réseaux SSD, ou le réseau [YOLO]{https://pjreddie.com/darknet/yolo/}
spécifiques à la détection d'objet et qui sont supportés par TFLite. Il existe aussi _YOLO/Tiny YOLO_ est plus rapide mais moins précis.

## Convertir le réseau

Première étape :  réalise la capsule **Detection d'objet avec Tensorflow** pour entraîner ton réseau avec tes images.

__Attention ! Dans la capsule **Téléchargement du réseau pré-entrainé**, si le réseau doit être exploité sur 
des architectutes légères (RPI, TPU, ...) il faut choisir un réseau de départ adapté.__

Le réseau utilisé dans les prochains exemples et capsules est le réseau 
**SSD MobileNet V2 FPNLite 640x640**.

![Convert](img/convert.png)<br>(source: https://coral.ai/docs/edgetpu/models-intro/#compatibility-overview)<br>

### Exporter le graphe d'inférence 

La commande ci-dessous génère un fichier d'extension **.pb** (_frozen_graph_) qui va être utilisé en entrée pour la conversion TFLite.

```python 
python <path_to_models/research>/object_detection/export_tflite_graph_tf2.py \
    --pipeline_config_path <path_to_ssd_model>/pipeline.config \
    --trained_checkpoint_dir <path_to_ssd_model>/checkpoint \
    --output_directory <path_to_exported_model_directory>
```
Le fichier `saved_model.pb` est crée dans le chemin `<path_to_exported_model_directory>/saved_model/`.

Par exemple, avec les capsules déjà faites, le dossier de travail où a été installé l'API TOD est `tod_tf2`, et la commande ci-dessus devient :
```python 
# From the tod_tf2 directory:
python ./models/research/object_detection/export_tflite_graph_tf2.py \
    --pipeline_config_path training/<project>/<ssd_model_dir>/pipeline.config \
    --trained_checkpoint_dir training/<project>/<ssd_model_dir>/checkpoint \
    --output_directory training/<project>/<ssd_model_dir>/
```

### Convertir avec le convertisseur TFLite

Pour convertir le fichier  **saved_model.pb** au format TFLite, la commande est la suivante : 

```python 
tflite_convert \
  --saved_model_dir <path_to_"saved_model"_dir> \
  --output_file <path>/<name>.tflite
```
Par exemple :
```python 
# From the tod_tf2 directory:
tflite_convert \
  --saved_model_dir training/<project>/<ssd_model_dir>/saved_model \
  --output_file training/<project>/<ssd_model_dir>/tagada.tflite



