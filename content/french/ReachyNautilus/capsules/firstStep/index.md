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
[Doc SDK Reachy](https://pollen-robotics.github.io/reachy-sdk/index.html)


Une fois connecté au robot, tu peux commencer à programmer les mouvements de Nemo. 
Dans Jupyter, tu trouvera un fichier *premier pas.ipynb* dans le dossier *Nautilus*. Tu dois copier ce fichier et renommer la copie sous la forme *aa-mm-jj.prenom.nom*. 

## 1. Instancier l'objet Reachy 

Regardons en détail le code :

```python 
from reachy_sdk import ReachySDK
```  
On import l'objet reachy du sdk Reachy. 

On se connecte au robot :
```python
reachy = ReachySDK('localhost')
```

Si tu ne vois aucune erreur au lancement de ces lignes de code, bonne nouvelle, tu es maintenant connecté au Robot et tous les systèmes ont bien été trouvés !

L'objet reachy possède 8 attributs et 2 méthodes que nous allons rapidement présenter ici : 
* `reachy.fans` : permet d'accéder au ventilateur du bras et de la tête 
* `reachy.force_sensors` : permet d'accéder au capteurs de force présent dans la pince 
* `reachy.head` : permet d'accéder au informations des trois articulations composant la liaison Orbita ainsi que des méthodes pour sa cinématique ou pour la contrôler.
* `reachy.joints` : permet d'accéder aux informations (par exemple la position) sur toutes les jointures
* `reachy.r_arm` : permet d'accéder à chaque jointure du bras droit (épaule, coude, poignet ...etc)
* `reachy.left_camera` : permet de récupérer la dernière image capturée par la caméra de gauche et également de contrôler le zoom motorisé attaché à la caméra
* `reachy.right_camera` : pareil que pour la caméra gauche 

* `reachy.turn_on()` : méthode pour rendre rigide une partie du robot, c'est-à-dire mettre toutes les articulations de cette partie en mode rigide. 
* `reachy.turn_off()` : méthode pour rendre libre une partie du robot, c'est-à-dire mettre toutes les articulations de cette partie en mode libre. 

## 2. Compliant ou pas ?

Les servomoteurs utilisés dans le bras de Reachy ont deux modes de fonctionnement:

- **libre (OFF)** : les servomoteurs sont libres et peuvent être tournés à la main. Ils ne peuvent pas être contrôlés.
- **rigide (ON)** : les servomoteurs sont actifs et résistent au déplacement à la main. Ils peuvent être contrôlés en définissant une nouvelle position cible.

Pour que Reachy conserve sa position et te permette de contrôler ses moteurs, tu dois les mettre en ON en utilisant la méthode `reachy.turn_on()`. Pour rendre les moteurs "libre" tu utilisera la méthode `reachy.turn_off()`. Tu peux rendre rigide tout reachy d'un coup ou juste spécifier quelle partie tu veux rigidifier. 

`reachy.turn_off_smoothly('reachy')` permet de passer le robot en mode libre plus doucement 

⚠️ **Attention** : il ne faut surtout pas forcer les moteurs lorsque le robot est en mode rigide cela pourrait les endommager. 

Si l'on souhaite mettre une jointure en particulier en mode rigide ou libre tu peux utiliser l'attribut compliant d'une jointure : 
`reachy.r_arm.r_shoulder.compliant = True`
*True met la joiture en mode libre, False en mode rigide.*

## 3. Les méthodes pour faire bouger les moteurs :

Pour commander par programme le robot, tu va utiliser les méthodes goto(), look_at() et goal_position. 
Documentation des classes et méthodes : [ici](https://pollen-robotics.github.io/reachy-2019/autoapi/reachy/index.html#)

### goto(goal_position, duration, starting_positions=None , sampling_freq=100 , interpolation_mode=<function linear> )

Pour commander par programme le robot, nous utiliserons la fonction goto() (majoritairement pour commander le bras). La fonction goto() prend comme argument la position cible en degrés, une durée de déplacement en seconde et un mode de déplcament. Tu peux également renseigner la position de départ si elle est différente de la position actuelle du robot. 
Ici on défini une position pour chaque partie du bras, une durée de déplacement total et le mode de déplacement "interpolation_mode":

```python 
goto(
    goal_positions= {
        reachy.r_arm.r_shoulder_pitch: 0,
        reachy.r_arm.r_shoulder_roll: 0,
        reachy.r_arm.r_arm_yaw: 0,
        reachy.r_arm.r_elbow_pitch: -90,
        reachy.r_arm.r_forearm_yaw: 0,
        reachy.r_arm.r_wrist_pitch: 0,
        reachy.r_arm.r_wrist_roll: 0,
    }
    duration=1.0,
    interpolation_mode=InterpolationMode.MINIMUM_JERK
)
```

On peut utiliser cette méthode pour une seule partie du bras. Par exemple, pour le coude :
```python
goto(goal_positions={reachy.r_arm.r_elbow_pitch: -90}, duration=1.0, interpolation_mode=InterpolationMode.MINIMUM_JERK)
```

💡 Pour effectuer des trajectoires en simultanées, on utilise la fonction goto_async()

⚠️ **Attention à ne pas avoir deux trajectoires tournant sur le même moteur en parallèle ! Cela peut entraîner un comportement imprévu.**

### look_at(x, y, z, duration, wait)

Cette méthode permet de commander la tête en fonction d'un point 3D dans l'espace (Reachy regarde ce point 3D) :
```python
reachy.head.look_at(1, 0, 0, duration=1, wait=True)
```

⚠️ **Attention à la durée d'atteinte des positions : ne pas mettre des durée trop courte.**

### goal_position

Pour commander les moteurs des antennes on utilise une méthode inférieur à la méthode goto(). Tu dois être prudent en utilisant cette méthode car le moteur essaiera d'atteindre cette nouvelle position d'objectif aussi vite que possible. Une solution de contournement consiste à utiliser la propriété moving_speed pour définir la vitesse maximale que le moteur peut atteindre.

```python
reachy.head.l_antenna.speed_limit = 50.0
reachy.head.r_antenna.speed_limit = 50.0

reachy.head.l_antenna.goal_position = 0.0
reachy.head.r_antenna.goal_position = 0.0
```

## 4. Enregistrer une trajectoire et la reproduire

Jusqu'à présent, nous vous avons commander le robot pour qu'il effectue des mouvements simples. Pour affectuer des mouvements complexes, nous allons utiliser un bout de code qui permet d'enregistrer une trajectoire faite par Reachy. 
Avec cette approche, tu vas effectuer des trajectoires entières avec Reachy en le déplaçant à la main (en utilisant le mode libre) et enregistrer les positions des différents moteurs. Selon ce que tu veux, tu peux enregistrer un seul moteur ou plusieurs à la fois.


Pour enregistrer un mouvement sur le bras droit on précise les jointures sur lesquelles ont enregistre les positions :
```python
recorded_joints = [
    reachy.r_arm.r_shoulder_pitch,
    reachy.r_arm.r_shoulder_roll,
    reachy.r_arm.r_arm_yaw,
    reachy.r_arm.r_elbow_pitch,
    reachy.r_arm.r_forearm_yaw,
    reachy.r_arm.r_wrist_pitch,
    reachy.r_arm.r_wrist_roll,
]

sampling_frequency = 100  #en hertz
record_duration = 10  #en seconde
```

On créer une boucle While qui permet d'enregistrer toute les positions actuelles lors du mouvement :
```python 
reachy.turn_off('r_arm')
trajectories = [] #on créer une nouvelle liste trajectoire

start = time.time() #seconde passées depuis epoch
while (time.time() - start) < record_duration:
    #on optient les positions actuelles de toutes les jointures 
    current_point = [joint.present_position for joint in recorded_joints]
    #on ajoute les positions à la liste trajectoire 
    trajectories.append(current_point)

    time.sleep(1 / sampling_frequency)
```

On lance  :
```python
#on rend rigide toutes jointures utiliser pour effectuer la trajectoire
for joint in recorded_joints:
    joint.compliant = False

#on créer un dictionnaire associant chaque jointure à sa première position 
first_point = dict(zip(recorded_joints, trajectories[0]))

#le robot se positionne sur les premieres positions de chaque jointures 
goto(first_point, duration=3.0)
```

Ensuite pour rejouer la trajectoire :
```python
for joints_positions in trajectories:
    for joint, pos in zip(recorded_joints, joints_positions):
        joint.goal_position = pos

    time.sleep(1 / sampling_frequency)
```

## 5. Création de trajectoire 

Pour effectuer une trajectoire 3 options : 
* trajectoire point par point 
* trajectoire aléatoire 
* trajectoire qui suit une courbe 


