---
title: "Python Orienté Objet"
menu:
  main:
    name: "Python Orienté Objet"
    weight: 7
    parent: "capsules"
---
| Classe de capsule  | &emsp;durée recommandée |
|:-------------------|:------------------|
| Info  &emsp;  ℹ️  |&emsp; 10 min      |

### **<p style="text-align: right;">Capsule info ℹ️</p>**
### <p style="text-align: right;">Durée : 10min</p>

## 🎒 Prérequis

- Lycée et +

## Acquis d'apprentissage
A l'issue de cette activité, l'apprenant saura : 
- Créer une classes 
- Définir un constructeur
- définir des attributs
- créer des méthodes  

## 📗 Documentation

Les informations de cette capsule sont tirées des liens suivants :
[Cours sur Python](https://courspython.com/bases-python.html)


## 1. Classes, attribus et méthodes 

La programmation orientée objet (POO) est un concept de programmation très puissant qui permet de structurer ses programmes d'une manière nouvelle. En POO, on définit un « objet » qui peut contenir des « attributs » ainsi que des « méthodes » qui agissent sur lui-même.

**Classe** : Une classe définit des attributs et des méthodes. Par exemple, imaginons une classe Voiture qui servira à créer des instances qui seront des voitures. 
`class Voiture:()` créer la classe voiture. 
`tutut = Voiture()` ici on créer un nouvel objet de type voiture, tutut fait référence a cette objet. 

**Attribut** : Cette classe va pouvoir définir un attribut couleur, un attribut proprietaire, un attribut vitesse, etc. Ces attributs correspondent à des propriétés qui peuvent exister pour une voiture. 

**Méthode** : La classe Voiture pourra également définir une méthode fctFerrari(). Une méthode correspond en quelque sorte à une action, ici l’action c'est de déterminer si l'instance de la classe Voiture est une Ferrari ou pas. 

**Constructeur** :  

### Constructeur 

Un constructeur n’est rien d’autre qu’une méthode, sans valeur de retour. Le constructeur se définit dans une classe comme une fonction avec deux particularités :
* le nom de la fonction doit être __init__ ;
* la fonction doit accepter au moins un paramètre, dont le nom doit être self, et qui doit être le premier paramètre.
Le paramètre self représente en fait l'objet cible, c'est-à-dire que c'est une variable qui contient une référence vers l'objet qui est en cours de création. Grâce à ce dernier, on va pouvoir accéder aux attributs et fonctionnalités de l'objet cible.

```
class Voiture:
    def __init__(self, couleur, proprietaire):
        self.couleur = couleur
        self.proprietaire = proprietaire
```

### Définition des attribus 

```
class Voiture:
    def __init__(self, couleur, proprietaire):
        self.couleur = couleur
        self.proprietaire = proprietaire

tutut = Voiture('rouge', 'thea')
tutut.proprietaire = 'thea'
tutut.couleur = 'rouge'
print("tutut : proprietaire =", tutut.proprietaire, "couleur =", tutut.couleur)
>tutut : proprietaire = thea couleur = rouge
```
Ici les attributs de la classe Voiture sont : 
- le proprietaire  
- la couleur    

La syntaxe pour accéder à un attribut est la suivante : objet.attribut

Pour lire l'attribu d'une instance d'objet :   
`x = objet.attribu`  

Ecrire l'attribu d'une instance d'objet :   
`objet.attribu = valeur`   


### Définition des méthodes 

Une méthode se définit dans une classe comme une fonction, avec comme particularité qu'elle doit accepter au moins un paramètre, dont le nom doit être self, et qui doit être le premier paramètre. Ce paramètre représente l'objet cible sur lequel la méthode est appelée. Il permet notamment d'avoir accès aux variables d'instance de l'objet.

La syntaxe pour appeler une méthode est la suivante : objet.méthode(). 

Ici on va créer une méthode fctFerrari() pour savoir si notre voiture est une Ferrari (si tu as une voiture rouge, c'est une Ferrari) :  
```
class Voiture:
    def __init__(self, couleur, proprietaire):
        self.couleur = couleur
        self.proprietaire = proprietaire
        
    def fctFerrari(self): 
        if(self.couleur=='rouge'):
           print("c'est une ferrari'")
        else : 
            print("ce n'est pas une ferrari désolé")

tutut = Voiture('rouge', 'thea')
tutut.fctFerrari()
>c'est une ferrari
```
### Encapsulation 

Le concept d'encapsulation est un concept très utile de la POO. Il permet en particulier d’éviter une modification par erreur des données d’un objet. En effet, il n’est alors pas possible d’agir directement sur les données d’un objet ; il est nécessaire de passer par ses méthodes qui jouent le rôle d’interface obligatoire.
```
class Voiture:
    def __init__(self, couleur, proprietaire):
        self._couleur = couleur
        self._proprietaire = proprietaire
```
Il n’est alors plus possible de faire appel aux attributs __couleur et __proprietaire depuis l’extérieur de la classe Voiture.

Résultats : 
```
tutut('rouge', 'thea')
tutut.couleur

---------------------------------------------------------------------------
AttributeError                            Traceback (most recent call last)
<ipython-input-2-9d15053ddff5> in <module>
      1 tutut = Voiture('rouge', 'thea')
----> 2 tutut.couleur

AttributeError: 'Voiture' object has no attribute 'couleur'
```

Il faut donc disposer de méthodes qui vont permettre par exemple de modifier ou d’afficher les informations associées à ces variables. 
```
class Voiture:
    def __init__(self, couleur, proprietaire):
        self.__couleur = couleur
        self.__proprietaire = proprietaire
    
    def modifProprio(self, proprietaire):
        self.__couleur=couleur
    
    def affiche(self):
        print("couleur =", self.__couleur, "proprietaire =", self.__proprietaire)
    
```