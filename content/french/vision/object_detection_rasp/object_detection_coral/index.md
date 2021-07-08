--- 
title: "Détection d'objet sur la Raspberry Pi avec un accélérateur USB Coral"
menu:
  main:
    name: "Détection d'objet sur la Raspberry Pi avec un accélérateur USB Coral"
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
* Capsule sur **Détection d'objet sur la Raspberry Pi avec un réseau au format TFLite**

## 🎓 Acquis d'apprentissage

* Mise en place de l'environnement sur la Raspberry Pi pour l'utilisation de Coral
* Utilisation d'un modèle au format TFLite pour la détection d'objet sur une Raspberry Pi
avec un accélérateur USB Coral


## 📗 Documentation

Credit : 
* https://github.com/EdjeElectronics/TensorFlow-Lite-Object-Detection-on-Android-and-Raspberry-Pi/blob/master/Raspberry_Pi_Guide.md


## Installation des bibliothèques pour l'accélérateur sur la Raspberry 

Après avoir mis en place l'environnement et lancer la détection d'objet
avec des modèles TFLite grâce à la capsule **Détection d'objet sur la Raspberry Pi avec un réseau au format TFLite**, 
on installe maintenant les librairies nécessaires pour utiliser l'accélérateur USB Coral.
Celui-ci permet d'accélérer les modèles TensorFlow grâce à un coprocesseur Edge TPU, ce qui 
permet des inférences d'apprentissage automatique à grande vitesse.

Place toi dans ton dossier et active l'environnement virtuel : 

```python 
cd tflite-detection/TensorFlow-Lite-Object-Detection-on-Android-and-Raspberry-Pi-master
conda activate tflite-env
```

Dans un premier temps, on ajoute le référenciel des paquets Coral à ta liste de distribution apt-get : 

```python 
echo "deb https://packages.cloud.google.com/apt coral-edgetpu-stable main" | sudo tee /etc/apt/sources.list.d/coral-edgetpu.list
curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | sudo apt-key add -
sudo apt-get update
```

Puis on installe la librairie **libedgetpu** : 

```python 
sudo apt-get install libedgetpu1-std
```

## Choisir le réseau entraîné Edge TPU

Les modèles Edge TPU sont des réseaux au format TFLite ayant été quantifiés avec
la technique dite de **full integer quantization** puis convertis dans un format supporté 
par l'accélérateur USB Coral avec le compilateur Edge TPU.
Ces modèles possèdent aussi l'extension .tflite.
Dans ce projet, le modèle Edge TPU nommé **edgetpu.tflite** est placé dans le même dossier que le modèle TFLite.

De la même manière que dans la capsule **Détection d'objet sur la Raspberry Pi avec un réseau au format TFLite**, 
on peut soit télécharger un modèle Edge TPU fournit par Google en exemple, 
soit utiliser son propre modèle Edge TPU entraîné.

### Utiliser le modèle Edge TPU de Google

Google fournit un modèle quantifié pour la détection d'objet SSDLite-MobileNet-v2. 
Il est entrainé avec la base de données MSCOCO et convertit au format Edge TPU. 

Les commandes suivantes téléchargent le réseau et le déplace dans le dossier
contenant le modèle TFLite et la carte des labels tout en le renommant : 

```python 
wget https://dl.google.com/coral/canned_models/mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite
mv mobilenet_ssd_v2_coco_quant_postprocess_edgetpu.tflite Sample_TFLite_model/edgetpu.tflite
```

### Utiliser un réseau Edge TPU customisé

De la même manière que dans la capsule **Détection d'objet sur la Raspberry Pi avec un réseau au format TFLite**,
on peut utiliser son propre modèle customisé.
Place les fichiers dans un dossier et importe ce dernier sur la Raspberry 
dans le dossier **TensorFlow-Lite-Object-Detection-on-Android-and-Raspberry-Pi-master**. 

**voir capsule Théa**


## Tester le réseau Edge TPU

Branche ton accélérateur USB Coral sur un des ports USB de ta Raspberry Pi. 
Si cette dernière correspond à la Pi 4, branche l'accélérateur sur un des ports de couleur bleue USB 3.0.

Vérifie que l'EVP est bien activé avant de lancer le script de détection d'objet en temps réel : 

```python 
python3 TFLite_detection_webcam.py --modeldir=Sample_TFLite_model --edgetpu
```

L'argument **--edgetpu** permet au script d'utiliser l'accélérateur et le modèle Edge TPU. 
N'oublie pas de changer l'argument **--modeldir** si le nom de ton dossier n'est pas le même.

Tu peux aussi lancer d'autres scripts pour la détection d'objets notamment sur des images ou 
sur des vidéos avec les commandes suivantes : 

```python 
python3 TFLite_detection_video.py --modeldir=Sample_TFLite_model --edgetpu
python3 TFLite_detection_image.py --modeldir=Sample_TFLite_model --edgetpu
```
