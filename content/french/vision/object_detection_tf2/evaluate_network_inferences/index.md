---
title: "Évaluer les inférences du réseau ré-entrainé"
menu:
  main:
    name: "Évaluer le réseau ré-entraîné"
    weight: 7
    parent: "objectDetectionTF2"
---

## Évaluer les inférences du réseau ré-entrainé

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

