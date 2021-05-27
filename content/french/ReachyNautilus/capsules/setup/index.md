---
title: "Configuration du robot"
menu:
  main:
    name: "Configuration du robot"
    weight: 2
    parent: "capsules"
---

| Classe de capsule  | &emsp;durée recommandée |
|:-------------------|:------------------|
| Setup  &emsp;  🛠️  |&emsp; 10 min      |
 

## 📗 Documentation

Plus informations sur le robot et sa mise en route avec ces liens :  
- [Doc Pollen Robotics](https://pollen-robotics.github.io/reachy-2019-docs/docs/getting-started/)  
- [Prise en main Reachy](https://github.com/ta18/Reachy_Nautilus/blob/main/Prise%20en%20main.md)

  
### **Infos robot 🤖** : 
Nom du robot: **Nemo**  
Adresse IP : `192.168.4.1` 

## 1. Matériel et branchement

La première étape est de brancher l'alimentation (fournie) au dos du robot (prise ronde) :

![Dos du robot](img/back.png)

Une fois l'alimentation branchée, appuie sur les 2 boutons ON/OFF à droite des prises pour mettre sous tension les moteurs et la Raspberry Pi du robot.

## 2. Connexion au robot

### 2.1 Logiciel et installation

Le robot est livré avec une carte Rasberry Pi qui permet de le contrôler.
Pour programmer les mouvements de robot il suffit d'utiliser un navigateur WEB sur ton ordinateur (Linux, Mac ou Windows); tu n'as aucun logiciel particulier à installer. 

### 2.2 Se connecter au serveur en wifi 📶

Pour te connecter au robot il faut suivre la procédure suivante :

1. Se connecter au hotspot WiFi du robot (dans notre cas c'est "Nemo").
2. Entrer l'adresse http://192.168.4.1:8888/ dans un navigateur web
3. Appuyer sur *se connecter* sans rentrer de mot de passe 

**Si cela ne fonctionne pas...** 

✅ Pour vérifier que tu es bien connecté au hotspot Wifi du robot, tu peux vérifier avec `ping` que tu peux utiliser son adresse IP :
`ping 192.168.4.1` sur un terminal de commande. S'il n'y a pas d'erreur c'est que tu es bien connecté au WiFi du robot.
Fait attention a toujours bien rester connecté au hotspot WiFi "Nemo". 

Et voilà tu es connecté au Reachy, bravo ! 🎉
