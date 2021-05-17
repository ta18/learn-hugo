---
title: "Poppy Ergo Jr + Turtlebot + TensorFlow"
menu:
  main:
    name: "Poppy Ergo Jr + Turtlebot + TensorFlow"
    weight: 2
    parent: "integration"
---



## Prérequis

* Avoir suivi les TP *Introduction*, *Manipulation*, *Navigation*, et *Perception*
* Avoir réalisé une ébauche de nœud Python pour chacun des TP *Manipulation*, *Navigation*, et *Vision*

## Diapositives

{{<pdf src="https://files.ros4.pro/integration.pdf" >}}

## 1. Définition du problème du scenario de tri

L'intégration consiste à intégrer dans une même cellule robotique les 3 briques logicielles travaillées les autres jours, à savoir :

* La manipulation par le bras robotique
* La navigation avec le robot roulant
* La vision avec le réseau de neurones

Le scenario de l'intégration est un système de tri robotisé de pièces dans un bac 1 ou un bac 2 selon leur marquage au feutre.

### Les prérequis
* L'arène est équipée de deux points de tri 1 et 2 ainsi que d'un sas de départ comprenant :
  * Le point de départ du Turtlebot
  * Le point de fixation de Poppy
  * 3 points correspondants à 3 emplacements A, B, C des cubes avant tri
* Des cubes en papier ont été imprimés, découpés et assemblés à partir de [ce patron](http://files.ros4.pro/cuves.pdf) 
* Le Turtlebot est équipée d'une cuve en carton ou bois permettant de recevoir un cube durant son transport
* Le réseau de neurones a été entrainé et sait faire des inférences avec un taux de réussite acceptable
* 3 trajectoires de saisie d'un cube aux 3 emplacements A, B, C ont été enregistrées pour Poppy
* 1 trajectoire de dépose d'un cube à un emplacement correspondant à la dépose dans la cuve

### Simplifications du problème
Le problème est simplifié par les choix suivants :
* La localisation des cubes dans l'image caméra (la *bounding box* de Tensorflow) n'est pas utilisée. A la place :
  * Les cubes sont triés de gauche à droite par coordoonnée x croissante (ne pas oublier de trier aussi leur labels)
  * Les mouvements de Poppy sont préenregsitrés et rejoués pour éviter les planifications peu efficaces vue l'imprécision de Poppy
* Il n'y a pas de procédure de dépot du cube à l'arrivée au bac de tri. Mais un opérateur doit retirer le cube avant le passage au cube suivant.
* La communication entre les noeuds est opérée par des paramètres grâce à `rospy.get_param(key)` et `rospy.set_param(key, value)`, plutôt que des services et topics afin d'éviter de créer et compilés de nouveaux types de messages 

## 2. Implémentation de l'intégration 
Nous proposons une architecture avec 3 noeuds :
* `manipulation.py` (contrôleur) : ce noeud est en charge de rejouer les trajectoires préenregistrées de Poppy permettant la saisie et la dépose des cubes. Il possède également le rôle primordial de contrôleur : c'est lui qui déclenche le processus de tri et l'orchestre
* `nn.py` : ce noeud est en charge de prendre une photo et faire une inférence des numéros de cubes annotés
* `navigate_waypoints.py` : ce noeud est en charge d'atteindre le bac 1 ou bac 2 avec le Turtlebot

Votre scenario se déroule selon les étapes suivantes :
1. `manipulate.py` initialise les paramètres
2. `manipulate.py` informe `nn.py` qu'il doit prendre une image et faire une inférence
3. `nn.py` prend une image en appelant les services Poppy (`control.launch`) et fait une inférence
4. `nn.py` trie les résultats de l'inférence par ordre croissant des `x`, et trie les labels associés **de la même manière**
5. `nn.py` informe `manipulation.py` de la liste de cubes et de leur label
6. `manipulate.py` exécute le mouvement préenregistré de saisie du cube A (le plus à gauche) ; puis le mouvement préenregistré de dépose sur le Turtlebot
7. `manipulate.py` informe `navigate.py` du label de ce cube (1 ou 2)
8. `navigate.py` navigue jusqu'à l'emplacement de l'arène correspondant au label (1 ou 2)
9. la dépose du cube peut être effectuée par une main humaine à l'arrivée à 1 ou 2
10. `navigate.py` navigue jusqu'à sa zone de départ
11. `navigate.py` informe `manipulate.py` qu'il est prêt pour le cube suivant (cube B)

Le diagramme d'activité suivant représente graphiquement ces mêmes interactions des 3 noeuds les uns avec les autres au cours du temps, ainsi que le rôle du serveur de paramètres et des services Poppy.

![Activité des noeuds](img/UML_integration.png)

Cette architecture et interaction est simplement une proposition, vous pouvez l'éditer comme bon vous semble pour adapter votre code à vos objectifs.

### 2.1. Préparer les prérequis
Balayez un-à-un les prérequis précédents du scneario de tri et entreprenez les actions nécessaires pour qu'ils soient atteints, par exemple que votre réseau soit entrainé ou bien que vous ayez défini les points sur votre arène et enregistré les trajectoires de Poppy, etc.

Pour être plus efficient, commencez par considérer uniquement le tri d'un unique cube avant de généraliser à tous les emplacements de cubes.

### 2.2. Créer un squelette de noeuds dans un package
Vous avez déjà créé un package `ros4pro_custom` que vous pouvez réutiliser pour l'intégration.

Dans ce package, pour chaque noeud, créer un nouveau fichier Python à partir d'un des fichiers déjà utilisé dans les TP précédents et ne conserver que le code qui vous semble pertinent pour respecter le diagramme d'activité.
Selon votre niveau, les squelettes de fichiers peuvent vous être fournis par les formateurs. Rendez vos fichiers exécutables avec `chmod +x`.

### 2.3. Gérer la synchronisation (attente active)

Les noeuds doivent s'attendre les uns les autres à plusieurs reprises, selon le diagramme d'activité ici-dessus.
La communication par paramètres ne permet pas de gérer directement ce problème de synchronisation entre noeuds. Pour contourner ce problème il est possible d'effectuer une attente active à chaque fois qu'un noeud est censé attendre qu'un paramètre possède la valeur souhaitée.

Le code suivant est une boucle d'attente active infinie. Il faut encore lui ajouter la lecture du paramètre attendu avec `get_param`, et de quitter la boucle dans le cas où la valeur du paramètre indique que l'attente est terminée.

```python
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    rate.sleep()
```

Placez une attente active dans chacun de vos 3 noeuds à chaque fois qu'il est nécessaire d'attendre qu'un paramètre change de valeur avant de passer à la suite du code Python.

### 2.4. Gérer le cas particulier de l'initialisation

Au tout début du démarrage du système de tri, aucun paramètre utilisé ne possède de valeur, la lecture de l'un d'eux va donc déclencher une erreur.
Pour résoudre ce problème, nous proposons que le contrôleur (`manipulation.py`) initialise tous les paramètres à une valeur remarquable, par exemple `-1` pour signifier que nous en sommes au démarrage.

Ajoutez les intiialisations de tous vos paramètres dans votre code.

### 2.5. (Optionnel) Créer des launchfiles pour démarrer le système

Vous pouvez créer de novueaux launchfiles pour simplifier le démarrage de votre système de tri robotisé, par exemple un launchfile unique par machine. Consultez la [documentation des launchfiles](http://wiki.ros.org/roslaunch/XML/node) et/ou procédez par imiation d'une launchfile existant.

## 3. Scenario de tri final

Votre système de tri doit fonctionne en autonomie (sauf la dépose finale du cube triée) depuis sa station de départ comme sur le tweet ci-dessous.

{{< tweet 1356167907865145345 >}}

Il est conseillé d'utiliser le robot Poppy comme ROS master, dans la mesure où ce robot est constamment branché sur secteur c'est celui qui est le plus souvent en foncitonnement et permets donc de garder un ROS master démarré le plus souvent possible. Modifiez les variables `ROS_MASTER_URI` de tous les terminaux sur toutes les machines et rechargez votre `.bashrc` pour les actualiser.

Poppy étant le ROS master, son launchfile doit démarrer en premier. Préparez des terminaux pour chaque launchfile ou noeud individuel à démarrer.

## 3.1 Tester un scenario de tri d'un seul cube
Un jalon important, avant le tri de tous les cubes, est le tri d'un seul cube : assurez-vous que le tri fonctionne pour 1 cube avant d'étendre votre code pour fonctionner avec tous les cubes.

### 3.2 Critères de succès du scenario final
Pour être considéré comme un succès, votre cellule doit permettre de trier au moins 3 cubes de manière complètement autonome une fois que vous avez démarré les `roslaunch` et `rosrun` nécessaires.

L'objectif est que ce scenario de tri puisse fonctionner dans une cellule en production. Cependant vous constaterez de nombreux défauts.

Relevez et adressez un à un ces défauts pour améliorer le taux de succès de votre cellule de tri.


## 4. (Optionnel) Challenges additionnels
### 4.1. Transformer les noeuds en machines à états

Utiliser [smach](http://wiki.ros.org/smach) pour que chaque noeud représente son état (par exemple **Etat 1 : en attente de cube**, **Etat 2 : en cours de navigation**, ...) par une machine à états.
La machine à état facilite les futures améliorations logicielles apportées au système robotique.

### 4.2. Trier tous les cubes à tout emplacement

Nous souhaitons que les cubes n'aient pas à être pré-positionnés dans 3 emplacements A, B, C mais que le système puisse trier tout cube qui se trouve à la fois dans son champs de caméra et à la fois à la portée du robot.
Ce nouveau problème nécessite de remplacer les 3 trajectoires de saisie par une planification dans l'espace cartésien avec MoveIt, laquelle nécessite également que les coordonnées 2D `x, y` des cubes de l'image puissent être transposées en coordonnées 3D `x, y, z` dans l'arbre des transformations `tf`. Une solution possible est d'utilisation le package de [calibration extrinsèque de caméra](http://wiki.ros.org/camera_calibration).

### 4.3. Ajouter un Poppy Ergo Jr pour la dépose des cubes dans le bac de tri

Le second Poppy doit être isolé dans un espace de nom pour ne pas créer de conflit avec le premier Poppy.
