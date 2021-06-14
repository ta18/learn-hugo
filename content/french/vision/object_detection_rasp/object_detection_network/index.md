---
title: "Convertir un réseau Tensoflow en TFLite"
menu:
  main:
    name: "Convertir un réseau Tensorflow en TFLite"
    weight: 3
    parent: "capsulesRSP"
---


| Classe de capsule  | &emsp;Durée recommandée |
|:-------------------|:------------------|
| Info  &emsp;  ℹ️  |&emsp; 10 min      |

## 🎒 Prérequis

* BAC+2 et +
* Bonne compréhension de Python et numpy
* Une première expérience des réseaux de neurones est souhaitable
* Une raspberry Pi avec caméra mise en place
* Capsule sur la **Mise en place des modules sur la Raspberry Pi**
* Capsule sur la **Détection d'objet sur la Raspberry Pi**

## 🎓 Acquis d'apprentissage

* Conversion d'un réseau Tensorflow en un réseau Tensorflow Lite

## 📗 Documentation

Credit : 
* https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/running_on_mobile_tf2.md
* https://www.tensorflow.org/lite/convert


## Introduction

Dans la capsule, **Détection d'objet sur la Raspberry Pi** un réseau sous Tensorflow Lite est utilisé.
C'est à dire qu'il possède un fichier **detect.tflite** et un **labelmap.txt**.
Cependant, customiser son réseau, c'est à dire l'entraîner sur des images spécifiques, 
est peu adapté sur une Raspberry Pi, du fait du manque de mémoire et du processeur ARM.
L'idée est donc d'entraîner un réseau Tensorflow avec une base de données spécifiques 
sur des machines avec des CPU et/ou GPU conséquents. Puis, de convertir ce réseau entrainé 
sous Tensorflow Lite afin de pouvoir l'utiliser sur la Raspberry Pi ou autre.

Afin de convertir son réseau, on réalise dans un premier temps l'ensemble des capsules **Detection d'objet avec Tensorflow**.


__Attention ! Dans la capsule **Téléchargement du réseau pré-entrainé**, si le réseau doit être porté par la suite
sur des architectutes plus légères (TPU, ...), il faut choisir un réseau de départ adapté.__


Ici, les capsules s'appuient sur l'utilisation d'une Raspberry Pi. Par conséquent, seuls les réseaux SSD sont adaptés.
De plus, comme on souhaite obtenir l'extension **.tflite**, le réseau doit avoir une annotation __FPNLite__.
**FPN** correspond à **Feature Pyramid Network**, c'est un sous-réseau qui génère des **feature maps** de
différentes résolutions.
Le réseau utilisé dans les prochains exemples et capsules est le réseau 
**SSD MobileNet V2 FPNLite 640x640**.


## Convertir le réseau

### Exporter le graphe d'inférence TFLite 

Cette commande génère un **savedModel** intermédiaire qui va être ensuite utilisé avec le convertisseur TFLite.
Cela devrait génèrer un dossier saved_model avec un ficher **saved_model.pb**.


```python 
# From the tensorflow/models/research/ directory
python object_detection/export_tflite_graph_tf2.py \
    --pipeline_config_path path/to/ssd_model/pipeline.config \
    --trained_checkpoint_dir path/to/ssd_model/checkpoint \
    --output_directory path/to/exported_model_directory
```

### Convertir avec le convertisseur TFLite

Pour convertir un **SavedModel**, la commande est la suivante : 


```python 
tflite_convert \
  --saved_model_dir=/tmp/mobilenet_saved_model \
  --output_file=/tmp/mobilenet.tflite
```

Si la commande ne renvoie pas d'erreur, elle devrait génèrer un fichier avec l'extension **.tflite**.




