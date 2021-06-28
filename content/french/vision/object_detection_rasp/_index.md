---
title: "Détection d'objets avec tensorflow lite et Raspberry Pi"
menu:
  main:
    name: "Détection d'objets avec tensorflow lite et Raspberry Pi"
    weight: 2
    identifier: "capsulesRSP"
    parent: "vision"
---

## Validation : 

__Faire valider retrée 2021 par des Bachelors 2A__

Lorsqu'on implémente et entraîne un modèle en utilisant `TensorFlow`, on obtient un modèle qui requiert beaucoup 
d'espace en mémoire et une certaine puissance de calcul notamment sur GPU. Ce genre de matériel n'est, 
en général, pas disponible sur des appareils mobiles. 
Pour résoudre cette problématique, il existe un ensemble d'outils nommé `Tensorflow Lite`.
Ce dernier permet une inférence de machine learning sur l'appareil avec une faible latence et une petite taille binaire.

Afin de réaliser la détection d'objet sur une `Raspberry Pi`, nous allons donc utiliser `Tensorflow Lite`.

## 🎒 Prérequis

* BAC+2 et +
* Bonne compréhension de Python et numpy
* Une première expérience des réseaux de neurones est souhaitable
* Une raspberry Pi avec caméra mise en place

## 🎓 Acquis d'apprentissage

* Installation des modules nécéssaires sur la Rasperry Pi
* Utilisation de tensorflow lite pour la détection d'objet

## 📗 Documentation

Credit : https://www.digikey.com/en/maker/projects/how-to-perform-object-detection-with-tensorflow-lite-on-raspberry-pi/b929e1519c7c43d5b2c6f89984883588
