---
title: "Connexion au robot"
menu:
  main:
    name: "Connexion au robot Reachy"
    weight: 2
    parent: "capsules"
---

| Classe de capsule  | &emsp; durée recommandée |
|:-------------------|:------------------|
| Setup  &emsp;  🛠️  |&emsp; 10 min      |


## 📗 Ressources

Plus d'informations sur le robot et sa mise en route avec ces liens :  
- [Doc Pollen Robotics](https://pollen-robotics.github.io/reachy-2019-docs/docs/getting-started/)  (en anglais)
- [Prise en main Reachy](https://github.com/ta18/Reachy_Nautilus/blob/main/Prise%20en%20main.md)

HUB USB 

Configuration écran/HDMI directement sur le robot. 

## 1. Mise en route 

![Dos du robot](img/back2021.png)   

1. Branche l'alimentation fournie sur la prise ronde au dos du robot.
2. Branche l'écran en HDMI sur le robot
3. Branche le Hub USB au robot pour connecter le clavier et l'écran.
4. Appuie sur le bouton poussoir à gauche pour mettre sous tension le NUC et sur le bouton ON/OFF à droite pour mettre sous tension les moteurs.
5. Allume l'écran du Reachy. 

## 2. Travailler sur le robot

Le robot Reachy est livré avec une carte NUC (mini ordinateur) qui permet de contrôler les moteurs et les périphériques qui l'équipent.   
Tu va travailler directement sur le robot, c'est-à-dire sur l'ordinateur du robot (le NUC)     
Lance Jupyter Notebook avec les commandes :    
```bash
cd ~/Reachy_Nautilus
jupyter notebook 
```

Et voilà tu es prêt à commancer avec le robot Reachy ! 🎉


