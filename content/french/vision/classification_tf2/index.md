---
title: "Classification avec tensorflow & keras"
menu:
  main:
    name: "Classification avec tf2"
    weight: 2
    parent: "vision"
---

**tensorflow** et **keras** sont deux modules Python qui permettent de construire des réseaux de neurones apprenants. Nous allons les utiliser pour entraîner un réseau de neurones à reconnaître des chiffres écrits manuellement au feutre avec différentes calligraphies, ce que l'on appelle aussi **classifier**.

## Prérequis

* BAC+2 et +
* Bonne compréhension de Python et numpy


## Diapositives

{{<pdf src="https://files.ros4.pro/perception.pdf" >}}

## 1. Documentation

Suivant ton expérience de Python et des modules nécessaires, tu pourras utiliser ces ressources :

1. Documentation générale sur numpy :
	* [Numpy cheatsheet](https://s3.amazonaws.com/assets.datacamp.com/blog_assets/Numpy_Python_Cheat_Sheet.pdf)
	* [NumPy quickstart](https://numpy.org/devdocs/user/quickstart.html)

2. Pour la partie extraction des faces des cubes et pré-processing, nous utiliserons le module `scikit-image`:
	* [Scikit-Image Documentation](https://scikit-image.org/docs/stable)

3. Enfin, pour la classification des images, nous utiliserons le module `keras` inclus dans le module `tensorflow` depuis sa version 2. Un point d'entrée sur l'API Séquentielle de keras peut être consulté sur cette page :
	* [keras API Sequential](https://www.tensorflow.org/guide/keras/sequential_model?hl=fr)

## 2. Installation

L'entraînement des réseaux de neurones avec le module `tensorflow2` se fera de préférence dans un environnement virtuel Python (EVP) qui permet de travailler dans une environnement dédié.

💻 Utilise la [FAQ Python : environnement virtuel](https://learn.e.ros4.pro/fr/faq/python_venv/)  pour créer un EVP :
* nommé `tf2`, 
* avec une version de Python égale à `3.8`.

Dans tout le document le _prompt_ du terminal sera noté `(tf2) jlc@pikatchou $` : le préfixe `(tf2)` est là pour bien rappeler que le travail Python se fait 
dans l'__Environnement Virtuel Python tf2__.

📥 Le code source à télécharger se trouve [ici](https://github.com/cjlux/ros4pro_perception) : télécharge l'archive zip, extrait le dossier `ros_perception-master` par exemple dans ton dossier `~/catkin_ws` renome le sous le nom `ros_perception` et installe les paquets Python complémentaires : 

```bash
(tf2) jlc@pikatchou $ cd ~/catkin_ws
(tf2) jlc@pikatchou $ unzip ~/Téléchargements/ros4pro_perception-master.zip 
(tf2) jlc@pikatchou $ mv ros4pro_perception-master ros4pro_perception
(tf2) jlc@pikatchou $ cd ros4pro_perception
(tf2) jlc@pikatchou $ pip install -r requirements.txt
```
## 3. Partie apprentissage

### 3.1 Travail préliminaire avec les notebooks Jupyter 📒

Tape la commande `jupyter notebook` dans le dossier `ros4pro_perception` ; tu peux alors charger les deux notebooks *à trous* pour la prise en main du *machine learning* avec **tensorflow** et **keras** :

* `notebook/TP1_MNIST_dense.ipynb` : utilise ce notebook pour l'acquisition des bases sur le *machine learning*
	* chargement et utilisation de la banque d'images MNIST utilisée pour l'entraînement des réseaux,
	* construction, entraînement et exploitation d'un réseau de neurones dense conduisant à un taux de reconnaissance des images MNIST voisin de 98 %.
	
* `notebook/TP2_MNIST_convol.ipynb` : utilise ensuite ce notebook pour la construction d'un réseau convolutif, son entraînement avec les images MNIST et son exploitation, conduisant à un taux de reconnaissance voisin de 99 %.

### 3.2 Travail avec les fichiers Python du dossier `src/`

Une fois familiarisé avec les principes de construction des réseaux denses et convolutifs, tu peux utiliser les programmes Python du répertoire `ros4pro_perception/src/`.

1. Chargement des images MNIST<br>
Ouvre maintenant le fichier `src/learning.py`, prends connaissance du code, puis lance le programme.<br>
Avant d'appuyer sur la touche ENTER, assure-toi que tu sais répondre aux questions :
	* Que contiennent les variables `x_train` et `y_train` ?
	* Pourquoi la fonction `load_data` renvoie-t-elle également les données `x_test` et `y_test` ?
	* Quelles sont les formes (_shape_) respectives de `x_train` et `y_train` ?

2. Prévisualisation des données brutes<br>
Appuye sur la touche ENTER pour continuer, et observe les images :
	* Quelles sont les valeurs des pixels blancs et noirs ?
	* Observe les données et leurs labels. Toutes les images sont elles simples à classifier correctement ?

3. Préparation des données<br>
Ferme la fenêtre et appuye à nouveau sur la touche ENTER :
	* Quelles sont les formes de `x_train` et `y_train` maintenant ?
	* Pourquoi ces changements ?

4. Prévisualisation des données préparées<br>
Appuye à nouveau sur la touche ENTER et observe les images :
	* Quelles sont les valeurs des pixels blanc et noirs maintenant ?
	* Regarde la fonction `prepare_input` : quelle transformation des images est effectuée ?

5. Le modèle du réseau convolutif<br>
	* Arrête le script. Dans le fichier source `learning.py` modifie la fonction `build_model` pour implémenter un réseau convolutif semblable à celui implémenté dans le notebook `TP2_MNIST_convol.ipynb`.
	* Relance le script et fait défiler jusqu'à la partie 5) (tu peux modifier `SHOW_SAMPLES` pour ne pas afficher toutes les fenêtres...) : vérifie les informations des couches sur le résumé du modèle...

6. La fonction de coût et l'optimiseur<br>
Arrête le script et vérifie :
	* la fonction de coût et l'optimiseur utilisés dans l'appel à `modele.compile(...)`
	
7. Entraînement :
	* Observe la fonction `train_model` : vérifie la présence et le paramétrage de la gestion de l'_over-fit_.
	* Relance le code jusqu'au déclenchement de la partie 7) : tu devrais voir les itérations d'entraînement se succéder et s'arrêter sur un événement __early stopping__.

8. Poids appris<br>
Appuye sur la touche ENTER pour visualiser les noyaux convolutifs appris par le réseau de neurones :
	* noyaux de la première couche : arrives-tu à distinguer le genre de _features_ qui seront extraites par chacun ?
	* Peux-tu faire de même pour la deuxième couche ?	

9. Activations<br>
Appuye sur la touche ENTER, puis entre un indice (un entier inférieur à 12000 (pourquoi 1200 ?)) :
	* Après la première couche de convolution, les _features_ extraites correspondent-elles à celles que tu imaginais ?
	* Après la première couche de _pooling_, les _features_ présentes auparavant sont-elles conservées ?
	* Après la deuxième couche de _pooling_, l'information spatiale est toujours présente ? Autrement dit, les activations ressemblent elles toujours à des images ?

10. Entraînement final<br>
Arrête le script. Jusqu'à présent, nous avons travaillé sur l'ensemble des images montrant des chiffres de '0' à '9', mais pour la suite nous n'aurons besoin que des images de '1' et de '2' :
	* Change la valeur de la variable `CLASSES` pour ne garder que les classes qui nous intéressent.
	* Change `SHOW_SAMPLES`, `SHOW_WEIGHTS` et `SHOW_ACTIV` pour sauter les affichage graphiques...
	* Entraînne le réseau avec le nouveau jeu de données réduites, puis sauvegarde-le en donnant le nom d'un répertoire où stocker les fichiers du réseau entraîné.

Tu peux passer maintenant à la **Partie Vision** qui permettra, une fois achevée, d'observer les inférences du réseau avec les images des cubes correctement traitées...

## 4. Partie Vision

Le but de la partie Vision est de traiter les images fournies par la caméra du robot :

![212.png](img/212.png)

pour trouver les contours des cubes :

![212_contours.png](img/212_contours.png)

et extraire des images compatibles MNIST :

![212_contours.png](img/2.png)

qui seront envoyées au réseau de neurone pour classification en '1' ou '2'...

### 4.1 Présentation des données

Ouvre le fichier `src/detection.py` et lance le script. Une des images exemple issue de la caméra du robot apparaît :

* Observe les valeurs de pixels ? Quelles sont les valeurs de pixels blancs et noirs ?

* De manière générale, la face des cubes est-elle semblable aux images MNIST ?

### 4.2 Binarisation de l'image

Appuye sur la touche ENTER pour afficher l'image binarisée :

* Peux-tu penser à un algorithme permettant d'arriver à un résultat similaire ?

Dans le code, observe la fonction `binarize` :

* À quoi sert la fonction `threshold_otsu` ? (voir au besoin la documentation  `scikit-image`).

En commentant successivement les lignes les utilisant, observe l'impact de chacune des fonctions suivantes :

* `closing`
* `clear_border`
* `convex_hull_object`

Pourquoi faut-il éviter d'avoir des cubes qui touchent les bords de l'image ?

### 4.3 Recherche des contours des cubes

Appuye sur la touche ENTER pour faire défiler quelques images dont les contours ont été détectés.

Observe la fonction `get_box_contours`:

* À quoi sert la fonction `label` ?
* À quoi sert le paramètre `area` ?
* À quoi sert la fonction numpy `argsort` utilisée à la fin pour le ré-arragement des contours ?
Pourquoi cette opération est elle importante ?

### 4.4 Extraction des vignettes

Appuye sur la touche ENTER pour faire défiler quelques images dont les vignettes ont été extraites.

Observe la fonction `get_sprites`: qu'est ce qu'une "transformation projective" ?

### 4.5 Préparation des images

Pendant la phase d'apprentissage, nous avons étudié la préparation qui était faite des images.

Les vignettes présentées au réseau de neurones doivent aussi être traitées pour avoir les mêmes caractéristiques que les images d'entrainement MNIST :

* complète la fonction `preprocess_sprites` pour effectuer ce traitement...

Une fois fait, exécute le script jusqu'à la fin et conclue sur l'allure des images traitées.

Tu peux maintenant ouvrir le fichier `main.py` pour tester l'intégration de la détection et de la reconnaissance par réseau apprenant...

## 5. Intégration

Il est maintenant temps d'intégrer les deux parties du pipeline pour l'utilisation finale. Ouvre le fichier `main.py` à la racine du projet.

Pour que les deux parties du pipeline s'adaptent correctement, tu as complété la fonction `preprocess_sprites` pour mettre les vignettes renvoyées par la partie détection dans un format compatible avec celui des images MNIST.

Exécute maintenant le programme `main.py` : donne le chemin d'un dossier qui contient les fichiers du réseau entraîné et tu devrais commencer à obtenir la reconnaissance des chiffres '1' et '2' dans les images fournies.

Il faudra certainement refaire plusieurs fois l'entraînement du réseau en jouant sur plusieurs paramètres avant d'obtenir un réseau entraîné qui fonctionne correctement :

* la valeur de la graine `SEED` peut conduire à un état initial des poids du réseau qui donne un entraînement meilleur ou pas...

* augmenter/diminuer `BATCH_SIZE` peut modifier les temps de calcul et la qualité du réseau entraîné...

* augmenter/diminuer le paramètre `patience` du callback `EarlyStopping`...

* enfin, tous les paramètres qui définissent les couches de convolution et de __spooling__ du réseau convolutif sont autant de possibilités d'améliorer (ou pas) les performances du réseau entraîné....

À toi de jouer pour obtenir un réseau entraîné classifiant le mieux possible les chiffres '1' et '2' dans les images fournies par la caméra du robot...

Pour confirmer la qualité de ton réseau entraîné tu peux enregistrer tes propres fichiers PNG avec les images faites avec la caméra du robot en utilisant le service ROS `/get_image`. 

Aide-toi des indications du paragraphe __2.4. Récupérer les images de la caméra en Python__ dans la section [Manipulation/Poppy Ergo Jr](https://learn.ros4.pro/fr/manipulation/ergo-jr/) : tu peux ajouter une instruction `cv2.imwrite(<file_name>, image)` pour écrire tes propres fichiers PNG dans le répertoire `data/ergo_cubes/perso` et modifier en conséquence la variable `img_dir` du fichier `main.py`.

Lance le programme et observe les performances de ton réseau opérant sur tes propres images.

