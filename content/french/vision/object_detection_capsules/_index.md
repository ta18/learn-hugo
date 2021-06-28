---
title: "Détection d'objets avec tensorflow"
menu:
  main:
    name: "Détection d'objets avec tensorflow"
    weight: 2
    identifier: "capsulesTF2"
    parent: "vision"
---
## Validation
- JLC
- rentrée 2021 par Bachelors 2A

Dans cette section nous allons utiliser l'API __Tensorflow Object Detection__ (_a.k.a_ TOD) qui propose :
* une collection de réseaux pré-entraînés, spécialement conçus pour pour la détection d'objets dans des images (__Object Detection__),
* un mécanisme de _transfert learning_ pour continuer l'entraînement des réseaux pré-entraînés avec ses propres images labellisées, 
pour obtenir la détection des objets qui nous intéressent.

Contrairement à la stratégie de __Classification__ présentée dans la section [Classification tf2](https://learn.e.ros4.pro/fr/vision/classification_tf2/), 
la __Détection d'objets__ permet de trouver directement les boîtes englobantes des objets "face avec un 1" et "face avec un 2" : 
cette approche évite de faire appel au traitement d'image classique pour extraire les faces des cubes dans un premier temps, puis de classifier les images des faces des cubes dans un deuxième temps. 

Le traitement d'image utilisé pour la classification est basé sur une approche traditionnelle de manipulation des pixels de l'image (seuillage, extraction de contour, segmentation...).
Il est assez fragile : sensible à la luminosité, à la présence ou non d'un fond noir...

Un avantage attendu de l'approche __Object Detection__ est de fournir directement les boîtes englobantes des faces des cubes, sans passer par l'étape de traitement d'image.


## 🎒 Prérequis

* BAC+2 et +
* Bonne compréhension de Python et numpy
* Une première expérience des réseaux de neurones est souhaitable.

L'entraînement des réseaux de neurones avec le module `tensorflow` se fera de préférence dans un environnement virtuel Python (EVP) qui permet de travailler dans un environnement Python  séparé de celui existant pour le travail sous ROS.

💻 Utilise la [FAQ Python : environnement virtuel](https://learn.e.ros4.pro/fr/faq/venv/)  pour créer un EVP :
* nommé `tf2`, 
* avec une version de Python égale à `3.8`.

## 🎓 Acquis d'apprentissage

* Mise en place des outils et de l'arborescence pour la détection d'objets
* Utiliser un réseau pré-entrainé
* Créer des nouvelles images pour les inclure dans l'apprentisage supervisé
* Réaliser un entraînement supervisé d'un réseau pré-entraîné
* Extraire les informations du réseau entraîné afin de l'évaluer

## 📗 Documentation

1. Documentation générale sur numpy :
	* [Numpy cheatsheet](https://s3.amazonaws.com/assets.datacamp.com/blog_assets/Numpy_Python_Cheat_Sheet.pdf)
	* [NumPy quickstart](https://numpy.org/devdocs/user/quickstart.html)

2. Documentation sur l'_API TOD_ pour `tensorflow2` :
	* Le tutoriel officiel complet : [TensorFlow 2 Object Detection API tutorial](https://tensorflow-object-detection-api-tutorial.readthedocs.io/en/latest/index.html)
	* Le dépôt git : [models/research/object_detection](https://github.com/tensorflow/models/tree/master/research/object_detection)<br><br>
Le tutoriel peut être consulté pour aller chercher des détails qui ne sont pas développés dans l'activité proposée, mais il est préferrable de suivre 
les indications du document présent pour installer et utiliser rapidement une version récente de tensorflow2. 

3. Lectures complémentaires :
	* [1] [Zero to Hero: Guide to Object Detection using Deep Learning: Faster R-CNN,YOLO,SSD](https://cv-tricks.com/object-detection/faster-r-cnn-yolo-ssd/)
	* [2] [mAP (mean Average Precision) for Object Detection](https://jonathan-hui.medium.com/map-mean-average-precision-for-object-detection-45c121a31173)
	* [3] [Understanding SSD MultiBox — Real-Time Object Detection In Deep Learning](https://towardsdatascience.com/understanding-ssd-multibox-real-time-object-detection-in-deep-learning-495ef744fab)


Les différentes sous-parties de cette section sont : 

* [Mise en place de l'environnement](object_detection_env/)
* [Compléter l'arborescence de travail](object_detection_arbo/)
* [Télécharger le réseau pré-entraîné](object_detection_download/)
* [Créer les données pour l’apprentissage supervisé](object_detection_data/)
* [Lancer l’entraînement supervisé du réseau pré-entraîné](object_detection_training/)
* [Évaluation du réseau entraîné](object_detection_evaluation/)
