---
title: "Détection d'objet sur la Raspberry Pi avec un réseau au format TFLite"
menu:
  main:
    name: "Détection d'objet sur la Raspberry Pi avec un réseau au format TFLite"
    weight: 3
    parent: "capsulesRSP"
---


| Classe de capsule  | &emsp;Durée recommandée |
|:-------------------|:------------------|
| Task  &emsp;  ⚙️  |&emsp; 15 min      |


## 🎒 Prérequis

* BAC+2 et +
* Bonne compréhension de Python et numpy
* Une première expérience des réseaux de neurones est souhaitable
* Une raspberry Pi avec une caméra activée
* Capsule sur **la mise en place et l'utilisation d'un l'environnement virtuel Python**
* Capsule sur **la conversion d'un réseau au format TFLite**

## 🎓 Acquis d'apprentissage

* Mise en place de l'environnement sur la Raspberry Pi
* Utilisation d'un modèle entraîné pour la détection d'objet 
* Utilisation de Tensorflow Lite pour la détection d'objet sur une Raspberry Pi

## 📗 Documentation

Credit : 
* https://github.com/EdjeElectronics/TensorFlow-Lite-Object-Detection-on-Android-and-Raspberry-Pi/blob/master/Raspberry_Pi_Guide.md


## Mise en place de l'environnement sur la Raspberry 

Les commandes suivantes mettent en place et activent un nouvel environnement virtuel Python (EVP):

```python 
conda create -n tflite-env python=3.8
conda activate tflite-env
```

Après avoir créé un nouveau dossier, on clone le dépôt GitHub d'[EdjeElectronics](https://github.com/EdjeElectronics) : 

```python 
mkdir tflite-detection
cd tflite-detection
git clone https://github.com/EdjeElectronics/TensorFlow-Lite-Object-Detection-on-Android-and-Raspberry-Pi.git
```

Ce dépôt contient les scripts pour faire tourner un réseau au format TFLite sur la Raspberry Pi. 
Il contient aussi un script pour installer facilement tout les paquets nécéssaires. 
__Attention !__ Ce script installe automatiquement les dernières versions des paquets, notamment
celle de Tensorflow. 

```python 
cd TensorFlow-Lite-Object-Detection-on-Android-and-Raspberry-Pi-master
bash get_pi_requirements.sh
```

## Choisir le réseau entraîné TFLite 

Ici, on peut soit télécharger un modèle TFLite fournit par Google en exemple, 
soit utiliser son propre modèle TFLite entraînée, obtenus éventuellement en suivant 
les capsules d'entraînements d'un réseau customisé et la capsule portant sur 
**la conversion d'un réseau au format TFLite**.

### Utiliser le modèle TFLite de Google

Google fournit un modèle quantifié pour la détection d'objet SSDLite-MobileNet-v2. 
Il est entrainé avec la base de données MSCOCO et convertit au format TFLite. 

```python 
wget https://storage.googleapis.com/download.tensorflow.org/models/tflite/coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip
unzip coco_ssd_mobilenet_v1_1.0_quant_2018_06_29.zip -d Sample_TFLite_model
```

### Utiliser un réseau TFLite customisé

La capsule **Convertir un réseau Tensorflow au format TFLite** fournit 
un fichier **detect.tflite** et un fichier **labelmap.txt**.
Place ces fichiers dans un dossier et importe ce dernier sur la Raspberry 
dans le dossier **TensorFlow-Lite-Object-Detection-on-Android-and-Raspberry-Pi-master**. 

## Tester le réseau TFLite

Pour tester le réseau TFLite, on lance le script de détection en temps réel
avec la caméra grâce à la commande suivante:

N'oublie pas d'activer ton EVP avant de lancer le programme !

```python 
python3 TFLite_detection_webcam.py --modeldir=Sample_TFLite_model
```
L'option **Sample_TFLite_model** correspond au dossier où sont placés 
le modèle TFLite et la carte des labels.
