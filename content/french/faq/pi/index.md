---
title: "FAQ robots basés sur Raspberry Pi"
menu:
  main:
    name: "FAQ robots basés sur Raspberry Pi"
    weight: 1
    parent: "faq"
---

### 🔔 Mon Turtlebot bipe

🔋 **Il s'agit du signal de batterie faible et il ne doit pas être ignoré.**

Turtlebot est alimenté par une batterie puissante de type Li-Po. Ce type de batterie rend dangereux leur utilisation lorsque la charge est très faible. Dans un cas extrême elle pourrait chauffer et prendre feu. **Mettre en charge rapidement la batterie lorsque Turtlebot bipe.**

## 📥 Flasher la carte SD

Note préliminaire : la carte SD du robot ne se comporte pas tout-à-fait comme une carte SD "normale". Elle ne permet pas de stocker des fichiers dessus ; il est également normal qu'une carte SD insérée dans votre ordinateur n'apparaisse pas dans le Poste de Travail avant de l'avoir flashée.

Téléchargez ces images ROS en vue de remettre à zéro les cartes SD des robots pour ROS4PRO (⚠️⏳ Optimisez votre temps, le téléchargement peut prendre 1 heure) :

* [Image du Turtlebot pour ROS4PRO](https://github.com/ros4pro/turtlebot3_ros4pro/releases/download/v1.0/2021-05-01-turtlebot.7z)
* [Image de Poppy Ergo Jr pour ROS4PRO](https://github.com/poppy-project/poppy_controllers/releases/download/v1.0/poppy-ergo-jr-ros-melodic.img.zip)

Pour flasher l'une de ces images sur une carte SD :

* extrayez le fichier compressé **.zip** ou **.7z** (généralement clic droit > Extraire) dans un dossier de votre ordinateur (pas sur la carte SD) : vous obtenez un fichier d'extension **.img**
* ⚠️ **ne faîtes pas** de glisser-déposer ni de copier-coller de cette image vers la carte SD comme s'il s'agissait d'une clé USB : Il est nécessaire d'utiliser un logiciel de flash comme Etcher ou dd
* 📀 Tapez la commande `etcher` dans le terminal Ubuntu pour ouvrir l'utilitaire de flash préinstallé (ou bien [téléchargez Etcher](https://www.balena.io/etcher/) s'il n'existe pas encore)
* Dans Etcher, "Flash from file", sélectionnez le fichier image ainsi que la destination (la carte SD) et validez
* Le flash de la carte SD est en cours ... ⚠️⏳ Optimisez votre temps, la copie dure environ 15 minutes. Dès qu'Etcher a terminé, votre carte SD est prête à être configurée pour le Wifi et/ou insérée dans le robot

Optionnellement, en cas de besoin de restaurer les robots avec les images d'usine, voici les liens (mais il y a un risque de collision de nom entre les deux robots, si la configuration n'a pas été faite) :

* [Lien vers la documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#sbc-setup) (pas de namespace complet, n'inclut pas la possibilité d'intégrer plusieurs robots)
* [Image d'usine de Poppy Ergo Jr](https://github.com/poppy-project/poppy-ergo-jr/releases/download/2.0.0/2017-04-06-poppy-ergo-jr.img.zip) (avec l'interface graphique `http://poppy.local` mais sans ROS)

⚠️ Si votre ordinateur n'arrive pas à lire la carte SD, vous pouvez essayer la procédure suivante :

1. Fermer tous les terminaux ouverts et réessayer
2. Ouvrir un terminal et exécuter les 2 commandes suivantes. Redémarrer l'ordinateur et réessayer
```bash
sudo apt-get install --reinstall udisks2
sudo apt-get install exfat-fuse exfat-utils
```
## 📡 Connecter le robot en Wifi

⚠️ La mise en place de la connexion du robot en Wifi ne nécessite pas de démarrer les robots.
⚠️ Les robots ont une procédure différente.

### Ergo JR

1. Insérer la carde SD du robot en question dans votre poste de travail (pas dans votre robot) et ouvrir la partition nommée `boot`

2. Télécharger le fichier [wpa_supplicant.conf](https://files.ros4.pro/wpa_supplicant.conf) dans `boot` et modifiez-le pour renseigner le bon mot de passe wifi à l'intérieur (sans changer le nom de fichier). Respectez la casse : majuscules/minuscules.

3. Créer un fichier vide nommé `ssh` au même endroit dans `boot` (par exemple avec la commande `touch ssh` dans le dossier courant)

4. Taper la commande `sync` puis éjectez proprement la carte SD dans le navigateur de fichier pour éviter toute perte de données avant de la retirer

Ces 2 fichiers `wpa_supplicant.conf` et `ssh` seront supprimés au prochain démarrage du robot, signalant que la demande de connexion Wifi a bien été prise en compte. C'est donc normal que vous ne les trouviez plus en regardant à nouveau le contenu de `boot` après un premier démarrage du robot.

En cas de problème, il est possible de connecter un écran HDMI à la Raspberry Pi, le gestionnaire réseau se trouve en haut à droite.

La connexion Wifi fonctionne aussi avec les points d'accès mobiles d'Android et iOS.

### Turtlebot 3

1. Télécharger le fichier [50-cloud-init.yaml](https://files.ros4.pro/50-cloud-init.yaml) et modifiez-le pour renseigner le bon login et mot de passe wifi dans les `WIFI_SSID` et `password`. **Respectez scrupuleusement l'indentation ! Ne rajoutez pas d'espaces ou de sauts de lignes et n'utilisez pas des tabulations (l'identation se fait avec 4 espaces)** Il est facile de faire une erreur sur ce fichier et il n'y aura aucun message d'erreur puisque c'est lui qui détermine si le robot va réussir à s'appairer au réseau. 

2. Insérer la carde SD du robot en question dans votre poste de travail (pas dans votre robot)

3. Ouvrir un terminal et copier le fichier avec cette commande :

```bash
sudo cp ~/Téléchargements/50-cloud-init.yaml /media/$(whoami)/writable/etc/netplan/
```

5. Vérifier dans `/media/$(whoami)/writable/etc/netplan` si le fichier a correctement été copié.

[Aide dans la documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/sbc_setup/#configure-the-raspberry-pi-2) (en cas de problème)

## 🖧 Se connecter via SSH à un robot

SSH (Secure SHell) permet d'ouvrir un terminal à distance sur une autre machine que celle sur laquelle on tape les commandes (par exemple le robot, qui n'a ni clavier ni écran pour interagir avec un terminal). Il est nécessaire de connaître :

* Le nom de la machine distante (par ex `poppy.local` ou `turtlebot.local`)
* Le nom d'utilisateur propriétaire de la session sur laquelle ouvrir un terminal (`pi` pour Poppy ou `ubuntu` pour Turtlebot)
* Le mot de passe de cette session (cf mots ce passe par défaut ci-dessous)

La commande est l'une des suivantes, à taper dans un terminal sur Ubuntu :

```bash
ssh pi@poppy.local
ssh ubuntu@turtlebot.local
```

Taper `yes` pour confirmer la connexion puis taper le mot de passe (`turtlebot` pour Turtlebot ou `raspberry` pour Poppy). Votre invite de commande devrait désormais indiquer `pi@poppy.local~$` ou `ubuntu@turtlebot.local~$` : toute commande tapée dans ce terminal sera exécutée par le robot. En cas d'erreur, consultez la procédure de diagnostic ci-dessous.

### 🔑 Mots de passe par défaut

#### Turtlebot

* Nom d'utilisateur `ubuntu`
* Nom de machine `turtlebot` (ajouter `.local` dans les commandes : `turtlebot.local`)
* Mot de passe `turtlebot`

#### Poppy

* Nom d'utilisateur `pi`
* Nom de machine `poppy` (ajouter `.local` dans les commandes : `poppy.local`)
* Mot de passe `raspberry`

### 🌈 Personnaliser les noms de robots et ordinateurs

Au démarrage du TP, tous les robots et les ordinateurs possèdent le même nom à savoir `ubuntu` (votre ordinateur), `poppy` (le robot manipulateur), `turtlebot` (le robot roulant), ce qui posera problème lorsqu'on les fera communiquer ensemble. Pour ces 3 machines, nous allons donc changer leur nom, en ajoutant juste votre numéro de groupe à la fin, par exemple `poppy5`.

💻🤖 Pour personnaliser votre nom, il faut ouvrir un terminal sur la machine à renommer (via SSH pour les robots) puis :

```bash
sudo hostnamectl set-hostname <NOUVEAU_NOM>
sudo reboot
```

Veillez bien à utiliser ensuite ce nouveau nom dans vos futures commandes (SSH ou ROS_MASTER_URI, ...). Si vous avez nommé votre robot `poppy5` par exemple, il faudra donc utiliser `poppy5.local`.

### 🔧 Procédure de diagnostic

💻 Dans un terminal taper `ping poppy.local` (pour Poppy) ou `ping raspberrypi.local` (pour Turtlebot) :

* **Si 1 ligne s'affiche chaque seconde** avec des statistiques de temps en millisecondes ➡️ Test réseau réussi. Vous avez peut-être oublié de démarrer le roscore ou bien `ROS_MASTER_URI` dans le fichier `~/.bashrc` pointe vers le mauvais robot
* **Si une erreur survient** et la commande s'arrête ➡️ Test réseau échoué. Vérifiez que la LED verte ACT de la Raspberry Pi vacille pendant environ 45 secondes lorsque vous venez de brancher l'alimentation :
  * **Si `ACT` vacille** en 🟢 ➡️ Votre Raspberry Pi démarre correctement mais la configuration réseau est incorrecte. Vérifiez que vous n'avez pas fait d'erreur  dans le fichier de configuration Wifi (`wpa_supplicant.conf` ou `50-cloud-init.yaml`) ou réessayez ; ou bien connectez-vous avec un câble RJ45 sur un routeur
  * **Si `ACT` ne vacille pas** ➡️ Votre Raspberry Pi ne démarre pas correctement. La LED rouge `PWR` s'allume-t-elle ?
    * **Si `PWR` s'allume** en 🔴 ➡️ Votre Raspberry Pi est fonctionnelle mais la carte SD ne possède pas une image valable. Recommencez la procédure de flash ci-dessus.
    * **Si `PWR` ne s'allume pas** ➡️ Votre Raspberry Pi  n'est pas fonctionnelle. Vous avez peut-être mal branché la Pixl (Poppy) ou bien le câble rouge-noir (Turtlebot)

### Mettre à jour l'openCR du Turtlebot

Si vous avez une erreur à propos d'une openCR incompatible, voici la méthode pour mettre à jour le firmware.

Tapez les commandes suivantes :

```bash
cd ./opencr_update
./update.sh /dev/ttyACM0 burger_noetic.opencr
```

Puis testez la mise à jour :

1. Placer le robot sur un sol plat dans un espace libre

2. Appuyer longuement sur le bouton `PUSH SW 1` pendant quelque secondes, le robot devrait aller tout droit pendant 30 centimètres

3. Appuyer longuement sur le bouton `PUSH SW 2` pendant quelque secondes, le robot devrait tourner de 180 degrées sur place

![OpenCR](./img/opencr_models.png)

## 📡 Comment effectuer un scan pour trouver l'adresse IP de la raspberry pi ?
Normalement vous n'avez pas besoin d'utiliser les adresses IP en dur, à la place on utile avahi-daemon (déjà installé) pour effectuer la résolution des noms (c'est ce qui permet de faire `ping raspberrypi.local` sans connaître son adresse). Mais si pour une obscure raison vous voulez quand même le faire, voici comment précéder. Ouvrir un terminal et exécuter les commandes suivantes :
```bash
sudo apt install net-tools
```
```bash
ifconfig
```
Rechercher votre adresse IP IPV4, par exemple "inet 192.168.1.30". Puis lancer un scan nmap sur cette adresse. Les devices connectés à votre réseau devraient apparaître avec un nom qui permet de les discriminer.
```bash
sudo nmap -sP 192.168.1.30/24
```

