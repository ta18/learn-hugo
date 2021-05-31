---
title: "Premier pas sur Reachy"
menu:
  main:
    name: "Premier pas sur Reachy"
    weight: 3
    parent: "capsules"
---

| Classe de capsule  | &emsp;durée recommandée |
|:-------------------|:------------------|
| Task  &emsp;  ⚙️  |&emsp; 10 min      |


## 🎒 Prérequis

- Lycée et +
- Notions de Python

## 🎓 Acquis d'apprentissage visés 
A l'issue de cette activité, l'apprenant saura : 
* utiliser un notebook Jupyter pour faire bouger le robot 
* faire bouger la tête 
* faire bouger un bras  
* mettre le robot compliant ou non-compliant


## 📗 Documentation

Si tu souhaites d'autres informations sur le robot et sa mis en route tu peux consulter ces liens :
[Doc Pollen Robotics](https://pollen-robotics.github.io/reachy-2019-docs/docs/program-your-robot/)
[Doc API Reachy](https://pollen-robotics.github.io/reachy-2019/index.html)


Une fois connecté au robot, tu peux commencer à programmer les mouvements de Nemo. 
Dans Jupyter, tu trouvera un fichier *premier pas.ipynb* dans le dossier *Nautilus*. Tu dois copier ce fichier et renommer la copie sous la forme *aa-mm-jj.prenom.nom*. 

## 1. Instancier l'objet Reachy 

Regardons en détail le code :

`from reachy import Reachy, parts`  
On import l'objet reachy de l'API Reachy. 

On spécifie les pièces du robot sur lesquels on va travailler :
```
reachy = Reachy( 
    right_arm=parts.RightArm(io='/dev/ttyUSB*', hand='force_gripper'),
    head=parts.Head(io='/dev/ttyUSB*'), 
    )
```
Ici, nous spécifions que nous voulons ajouter une partie bras droit et une partie tête. Ces pièces doivent être trouvées sur des ports séries USB de type «/ dev / ttyUSB \*». 
Pour le bras, nous définissons la main comme «force_gripper» (main en forme de pince).

Si tu ne vois aucune erreur au lancement de ces lignes de code, bonne nouvelle, tu es maintenant connecté au Robot et tous les systèmes ont bien été trouvés !

## 2. Compliant ou pas ?

Les servomoteurs utilisés dans le bras de Reachy ont deux modes de fonctionnement:

- **compliant** : les servomoteurs sont libres et peuvent être tournés à la main. Ils ne peuvent pas être contrôlés.
- **non compliant** : les servomoteurs sont actifs et résistent au déplacement à la main. Ils peuvent être contrôlés en définissant une nouvelle position cible.

Pour que Reachy conserve sa position et te permette de contrôler ses moteurs, tu dois les rendre non compliant en utilisant l'attribut `compliant` : 

Rendre le bras non-compliant : 
`reachy.right_arm.compliant = False`
Maintenant, le bras doit résister, tu ne peux plus le bouger à la main.

Rendre le bras compliant : 
`reachy.right_arm.compliant = True`

⚠️ **Attention** : il ne faut surtout pas forcer les moteurs lorsque le robot est en mode "non compliant" cela pourrait endommager les moteurs. 


## 3. Les méthodes pour faire bouger les moteurs :

Pour bouger des parties du robot, tu va utiliser les méthodes des classes Head() et Right_Arm() :
Documentation des classes et méthodes : [ici](https://pollen-robotics.github.io/reachy-2019/autoapi/reachy/index.html#)

### goto(goal_position, duration, wait)

Pour faire bouger notre moteur, nous utiliserons la méthode goto. Nous définirons une position cible en degrés et une durée de déplacement en seconde.
Ici on défini une position pour chaque partie du bras et une durée de déplacement total :

```
reachy.goto({ 
    'right_arm.shoulder_pitch': 0, 
    'right_arm.shoulder_roll': 0, 'right_arm.arm_yaw': 0, 
    'right_arm.elbow_pitch': -90, 
    'right_arm.hand.forearm_yaw': 0, 
    'right_arm.hand.wrist_pitch': 0, 
    'right_arm.hand.wrist_roll': 0, 
    'right_arm.hand.gripper': 0, 
    }, 
    duration=3, 
    wait=True)
```

On peut utiliser cette méthode pour une seule partie du bras. Par exemple, pour le coude :
`reachy.right_arm.elbow_pitch.goto( goal_position=90, duration=2, wait=True, )`

💡 Le `wait=True` permet d'attendre que le déplacement soit fini avant d'effectuer un autre déplacement. Si tu souhaite, par exemple, bouger le bras en même temps que la tête il faut mettre `wait=False`

⚠️ **Attention à ne pas avoir deux trajectoires tournant sur le même moteur en parallèle ! Cela peut entraîner un comportement imprévu.**


### goto(thetas, duration, wait)

Pour la tête on utilise également la méthode goto() avec thetas les positions cibles des 3 parties en dégrés :
`reachy.head.neck.goto(thetas=(-10,-10,-10), duration=3, wait=True)`

### look_at(x, y, z, duration, wait)

Cette méthode permet de bouger la tête en fonction d'un point 3D dans l'espace (Nemo regarde ce point 3D) :
`reachy.head.look_at(1, 0, 0, duration=1, wait=True)`

⚠️ **Attention à la durée d'atteinte des positions : ne pas mettre des durée trop courte.**

### goal_position

Pour faire bouger les antennes on utilise une méthode inférieur à la méthode goto() pour contrôler le moteur. Tu dois être prudent en utilisant cette méthode car le moteur essaiera d'atteindre cette nouvelle position d'objectif aussi vite que possible. Une solution de contournement consiste également à utiliser la propriété moving_speed pour définir la vitesse maximale que le moteur peut atteindre.

```
for m in reachy.head.motors:
    m.moving_speed = 50  # en degres par seconde
for m in reachy.head.motors:
    m.goal_position = 0
```

## 4. Enregistrer une trajectoire et la reproduire

Jusqu'à présent, nous vous avons fait bouger le robot en utilisant goto et une position cible. Cela fonctionne bien pour un mouvement simple mais parfois, pour des mouvements complexes, il semble plus agréable de pouvoir enregistrer un mouvement et l'enregistrer.

Avec cette approche, tu vas effectuer des trajectoires entières avec Reachy en le déplaçant à la main (en utilisant le mode compliant) et enregistrer les positions des différents moteurs. Selon ce que tu veux, tu peux enregistrer un seul moteur ou plusieurs à la fois. Un objet TrajectoryRecorder va rendre ce processus vraiment simple.

Pour enregistrer un mouvement sur le bras droit :
`from reachy.trajectory import TrajectoryRecorder, TrajectoryPlayer`

On créer une variable recorder :
`recorder = TrajectoryRecorder(reachy.right_arm.motors)`

Lorsque tu est pret effectuer shift + entrer sur la cellule :
`recorder.start()`

et Lorsqu'on veut stopper l'enregistrement :
`recorder.stop()`

Ensuite pour rejouer la trajectoire :
```
player = TrajectoryPlayer(reachy, recorder.trajectories) 
player.play(wait=True, fade_in_duration=3)
```

## 5. Création de trajectoire 

Pour effectuer une trajectoire 3 options : 
* trajectoire point par point 
* trajectoire aléatoire 
* trajectoire qui suit une courbe 


