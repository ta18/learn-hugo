---
title: "Reachy expressif"
menu:
  main:
    name: "Reachy expressif"
    weight: 9
    parent: "capsules"
---
| Classe de capsule  | &emsp;durée recommandée |
|:-------------------|:------------------|
| Task  &emsp;  ⚙️  |&emsp; 20 min      |


## 🎒 Prérequis

- Lycée et +
- 

## Acquis d'apprentissage
A l'issue de cette activité, l'apprenant saura : 
- Créer une classes 
- Définir un constructeur
- définir des attributs
- créer des méthodes  

## 📗 Documentation

Les informations de cette capsule sont tirées des liens suivants :
[Qu'est-ce-que l'autisme ?](https://www.craif.org/quest-ce-que-lautisme-44)


## Introduction 

### Définition de l'autisme ❓

![autisme](img/autism.png)

Les caractéristiques de l’autisme sont très variées d’un individu à l’autre. C’est pourquoi on parle de troubles du spectre autistique (TSA) : chaque personne se situe à un degré différent dans ce spectre.

Deux types de manifestation sont cependant identifiées. C’est ce qu’on appelle la dyade autistique :

* des difficultés dans la communication et les interactions sociales
Peu ou pas de langage, communication non-verbale inadaptée, répétitions de certains mots ou expressions, faible compréhension des sous-entendus, de l’humour, du langage imagé, difficulté à exprimer ses émotions et à comprendre celles des autres, malaise dans les interactions sociales…
* des comportements répétitifs et des intérêts restreints
Mouvements répétitifs ou compulsifs, intolérance aux changements ou à l’imprévu, intérêts ou activités obsessionnels…

Ces signes s’accompagnent souvent d’une hyper- ou d’une hypo-réactivité sensorielle. Les personnes autistes peuvent, en effet, réagir de manière intense aux stimuli sensoriels (bruit, lumière, odeur, toucher…) ou, au contraire, s’y montrer peu sensibles.

### Autisme et émotions 😀😥😠

Beaucoup de personnes autistes, enfants comme adultes, ont souvent du mal à reconnaître, interpréter, verbaliser et contrôler leurs émotions. De façon générale, elles ont du mal à partager les émotions de manière juste et appropriée. 
Les expressions faciales émotionnelles sont une des modalités expressives dont dispose l’enfant dès la naissance. Elles vont très rapidement constituer le support de l’interaction avec autrui, dans la mesure où elles vont se combiner avec d’autres comportements au fur et à mesure du développement, de telle sorte que la communication devienne pleinement multimodale.
Certaines personnes nées avec une condition neurodéveloppementale telle que le trouble du spectre de l’autisme (TSA), sont plus lentes à développer la capacité à identifier les émotions.

Depuis les travaux du psychologue américain Paul Ekmen, on peut considérer que l’être humain a six émotions de base : la peur, la tristesse, la colère, le dégoût, la joie et la surprise. Toutes sont reconnaissables par des expressions faciales et corporelles qui peuvent leur être associées. Ces expressions sont dites individuelles, puisqu’elles peuvent se déclencher même sans interaction sociale.
Si vers l’âge de 5–7 ans, la majorité des enfants autistes peuvent reconnaître la joie et la tristesse, ils peuvent avoir plus de difficultés avec certaines autres émotions plus subtiles telles que la peur et la colère. À l’adolescence, certains d’entre eux ont toujours de la difficulté à reconnaître la peur, la colère, la surprise et le dégoût. Cette difficulté les poursuivra une fois adulte alors qu’ils auront toujours des difficultés à reconnaître certaines émotions de base.

## Etat de l'art 📚
 
Le projet JEMImE vise à concevoir de nouveaux algorithmes de reconnaissance multimodale d’émotions pour évaluer la qualité des émotions produites par des enfants. L’objectif est d’aider les enfants avec autisme à apprendre à imiter et à mimer des émotions faciales et vocales afin d’exprimer l’expression appropriée à un contexte donné.  
Ils ont centré le jeu sur 3 émotions : la colère, la tristesse et la joie.

L’équipe PIRoS a également créé un dispositif ludique et engageant pour stimuler les enfants en leur demandant d’imiter les postures et les gestes du robot. Après quelques minutes, les rôles s’inversent et le robot peut à son tour imiter, grâce à l’intelligence artificielle, les mouvements réalisés par l’enfant.

Les chercheurs d'EngageME qui travaillent sur le projet EngageME, financé par l’UE, viennent de créer un cadre d’apprentissage automatique personnalisé pour les robots utilisés lors du traitement de l’autisme (pendant les sessions de thérapie). Comme ils l’expliquent dans leur communiqué publié dans «Science Robotics», ce cadre aide les robots à percevoir automatiquement l’affect – comportement gestuel, vocal et facial – et à intéresser les enfants pendant qu’ils interagissent avec eux.

documentation : 

[Travaux de l'équipe PIRos](https://www.sorbonne-universite.fr/actualites/la-robotique-au-service-des-enfants-autistes)   
[JEMIme](http://www.innovation-alzheimer.fr/jemime/)   
[EngageME](https://cordis.europa.eu/article/id/123847-teaching-robots-how-to-interact-with-children-with-autism/fr)   

## Activités 👩‍💻

Tu est ingénieur dans une équipe de recherche sur les maladies neurodéveloppementale qui concoit des serious game afin d'améliorer le quotidien des personnes en situation de handicap.
Ici vous aimeriez profiter du côté très émotionnel de Reachy qui provient principalement du mouvement de la tête et des antennes.  
Vous avez décider de créer un jeu mémoriel à l'intention des enfants avec autisme qui leur permettrait à terme de mémoriser un certain nombre d'émotions afin de pouvoir détecter les émotions d'autrui et permettre des interactions sociales plus adaptées.

### Déroulement du jeu mémoriel :   
1. Le robot produit des émotions par des expressions faciales et corporelles (posture et geste) et des expressions vocales. Il annonce via ses hauts parleurs l'émotion qu'il va faire.   
2. L'enfant doit retenir les émotions et le nom de l'émotion annoncée (on peut aussi associer l'émotion à une couleur mais il faut vérifier si ca ne biaise pas l'apprentissage + comment annoncer l'émotions ?). 
3. Le robot va refaire les émotions dans un ordre aléatoire et l'enfant devra les reconnaitre et restituer la séquences dans l'ordre. 

Tu dois fournir des séquences de 5 émotions. Voici les émotions que le robot doit faire et leurs principales caractéristiques : 
* La peur : mouvements courts, rapides, compulsifs, abrupts et maladroits / difficultés à rester immobile / croissement des bras / poings fermés ou mains cachées / tortillement 
* La tristesse : tête penchée vers l'avant / tête sur le coté droit / corps mou / corps recroquevillé / mouvements lents
* La colère : bras et mains raides / gestes rapides, lourds et brusques / voix forte / muscles contractés
* La joie : corps détendu / geste fluide / mouvements amples et grandissants / exclamations
* La surprise : main à la bouche / yeux écarquillés et sourcils haussées / exclamations
Tu peux t'inspirer de ta perception des émotions pour créer ces émotions sur Reachy. 

### Consignes 
Créer les émotions sur Reachy en lui faisant bouger les bras et la tête et en utilisant les hauts parleurs pour produire des sons. Le nom de l'émotion doit être énoncé à la personne qui est entrain d'apprendre.   
Créer une partie de code qui permet de lancer des séquences de 5 émotions faites dans un ordre aléatoire.   
Créer une partie de code où la personne qui apprendre pourra rentrer la séquences d'émotions qu'il a vu afin de valider la justesse de son apprentissage.   
Créer une partie de code qui permettra au robot d'apprendre de nouvelles émotions qu'il ne connait pas déjà (avec trajectory recorder).   

**Recommandations :** 
- Utiliser la création de classe pour créer la classe Emotion.
- Utiliser le trajectoire recorder pour enregistrer les trajectoires. 
- Utiliser les input pour permettre a l'apprennant de restituer la séquence jouée. 
- Utiliser le haut parleur de Nemo avec la librairie [pygame](https://devdocs.io/pygame-pygame-mixer-music/) pour jouer des sons sur Reachy. 

Partie avancée :   
Reconnaissance des émotions des enfants avec détection de visage et corps. 