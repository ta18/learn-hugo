---
title: "Mise en place des modules sur la Raspberry Pi"
menu:
  main:
    name: "Mise en place des modules sur la Raspberry Pi"
    weight: 3
    parent: "capsulesRSP"
---

## 🎒 Prérequis

* BAC+2 et +
* Bonne compréhension de Python et numpy
* Une première expérience des réseaux de neurones est souhaitable
* Une raspberry Pi avec caméra mise en place

## 🎓 Acquis d'apprentissage

* Installation de Tensorflow Lite sur la Raspberry Pi


## Mettre en place l'environnement virtuel 

Raspbian utilise par défaut Python 2. Cependant, ici, nous devons utiliser Python 3. 
Les commandes suivantes à taper dans le terminal créent un alias, c'est à dire
que lorsque nous écrirons une commande avec Python sans spécifier la version, 
Python 3 sera utilisé par défaut. De même pour pip.
On modifie par conséquent le `.bashrc`.

```python 
cd ~
nano .bashrc 
```

On rajoute ces deux lignes à la fin du document : 

```python 
alias python=python3
alias pip=pip3 
```

Sauvergarde et quite l'éditeur en utilisant le raccourci clavier crtl-x (attention ces raccourcis dépendent 
de l'éditeur de texte utilisé, ici nano). Tu peux quitter le terminal et en relancer un nouveau pour que les
changements soient pris en compte ou tu peux utiliser la commande suivante dans le terminal :

```python 
source .bashrc
```

Par la suite, il est préférable d'utiliser un environnement virtuel. 
Pour cela tu peux consulter la capsule correspondante qui utilise l'environnement virtuel `miniconda`.
Une autre solution est d'utiliser l'outil `vitualenv` qui fonctionne de manière similaire. 
Les commandes suivantes permettent l'installation de l'outil et la création 
d'un environnement virtuel `tflite-env`.

```python 
mkdir -p Projects/Python/tflite
cd Projects/Python/tflite
python -m pip install virtualenv
python -m venv tflite-env
```

Dans le répertoire tflite, on trouve donc l'environnement virtuel `tflite-env`. 
Lorsqu'on nous vondrons travailler avec `Tensorflow Lite`, il faudra activer 
`tflite-env` avec la commande :

```python 
source tflite-env/bin/activate
```

## Installer les modules nécéssaires 

Dans cet environnement, il faut installer quelques librairies pour le traitement des images et des vidéos : 

```python 
sudo apt -y install libjpeg-dev libtiff5-dev libjasper-dev libpng12-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev
```

Puis, il faut installer quelques librairies pour l'interface graphique : 

```python 
sudo apt -y install qt4-dev-tools libatlas-base-dev libhdf5-103 
```

Il faut ensuite intaller la librairie OpenCV. Ici, on spécifie une version particulière.


```python 
python -m pip install opencv-contrib-python==4.1.0.25
```

En tapant les commandes suivantes, on détermine quel processeur et quelle version de Python on utilise.

```python 
uname -m
python --version
```

Sur des Raspberry Pi 3 ou 4, le processeur devrait être un `armv7l` et ici Python 3.7.
`armv7l` est un 32-bit ARM processeur.

Sur la Raspberry Pi, ouvre un navigateur internet et cherche sur le site de tensorflow les .whl 
qui correspond à ton système d'exploitation et ton processeur. Ici, c'est Linux (ARM 32). 
Récupère le lien et tape dans le terminal  : 

```python 
python -m pip install <paste in .whl link>
```
Pour Linux (ARM 32) en juin 2021, la commande correspond à :

```python 
python -m pip install https://dl.google.com/coral/python/tflite_runtime-2.1.0.post1-cp37-cp37m-linux_armv7l.whl
```
