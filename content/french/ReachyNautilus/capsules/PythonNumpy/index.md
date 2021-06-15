---
title: "Python : les tableaux ndarray de numpy"
menu:
  main:
    name: "Python : les tableaux ndarray de numpy"
    weight: 6
    parent: "capsules"
---

| Classe de capsule  | &emsp;durée recommandée |
|:-------------------|:------------------|
| Info  &emsp;  ℹ️  |&emsp; 10 min      |


## 🎒 Prérequis

- Lycée et +

## Acquis d'apprentissage
A l'issue de cette activité, l'apprenant saura : 
- Manipuler les tableaux 
- Créer un tableau 1 à plusieurs dimensions 
- Effectuer des opérations de matrices
- Afficher des informations sur le tableau 

## 📗 Documentation

Les informations de cette capsule sont tirées des liens suivants :
[Cours sur Python](https://courspython.com/bases-python.html)

Memento Python à toujours avoir à coter de soi : 
{{<pdf src="https://perso.limsi.fr/pointal/_media/python:cours:mementopython3.pdf" >}}
 
📥 Si tu n'as pas installé **Anaconda** et que tu dispose juste de **MiniConda**, il te fait installer le package Numpy [ici](https://numpy.org/install/).  

**Vocabulaire : array = tableau = matrice**  

Pour commencer a utiliser Numpy il faut l'importer : 
```python 
import numpy as np
```


## 1. Création d'un array simple   

* À partir d'une liste python (python détermine lui-même le type de l'array créée) :  
```python 
a = np.array([1, 2, 3.5])
``` 
* À partir d'une liste python, mais en imposant un type (pareil avec float_ et bool_) : 
```python 
a = np.int_([1, 2, 3.5])
```
* À partir d'un tuple :  
```python 
a = np.array((1, 2, 3.5))
```  

Méthodes utiles :  
* pour connaître le type d'un array : 
```python 
a.dtype
```  
* accès à un élément : a[0]. Donne un scalaire du même type que le type du tableau, donc souvent un type numpy. Attention, si on veut un type python, il faut le convertir : int(a[0]) par exemple.  


⚠️ **Attention :**   
Si on fait `b = np.array(a)` b est une copie de a (si a changé, b ne l'est pas).  
Si on fait `b = np.asarray(a)` b pointe vers la même tableau que a (si a modifiée, b l'est aussi).  
`np.copy(a)` : renvoie une copie du tableau (indépendante du tableau de départ).   

## 2. Création d'un array à plusieurs dimensions :
On peut aussi créer un tableau à deux dimensions à partir d'une liste de listes :   
```python 
numpy.array([[1, 2, 3], [4, 5, 6]])
>array([[1, 2, 3],
       [4, 5, 6]])
```

Méthodes utiles :  
* `a[0, 1]` permet d'acceder à l'éléments ligne 1 colonne 2 (index de ligne, puis index de colonne).    
* **a.shape** : renvoie les dimensions du tableau, d'abord le nombre de lignes, puis le nombre de colonnes.  
* **s.size** : donne la taille totale d'un tableau numpy (le produit des tailles des differentes dimensions).  

## 3. Opération sur les array

### Produit matriciel 

Un tableau peut jouer le rôle d’une matrice si on lui applique une opération de calcul matriciel. Par exemple, la fonction numpy.dot() permet de réaliser le produit matriciel.

```python 
>>> a = np.array([[1, 2, 3],
                  [4, 5, 6]])
>>> b = np.array([[4],
                  [2],
                  [1]])
>>> np.dot(a,b) # méthode pour obtenir le produit entre 2 matrice 

array([[11],
       [32]])
```
### Transposé 

```python 
>>> a.T
array([[1, 4],
       [2, 5],
       [3, 6]])
```

### Déterminant 

```python 
>>> from numpy.linalg import det
>>> a = np.array([[1, 2],
                 [3, 4]])
>>> det(a) # méthode pour obtenir le déterminant d'une matrice 
-2.0
```

### Inverse 

```python 
>>> from numpy.linalg import inv
>>> a = np.array([[1, 3, 3],
                  [1, 4, 3],
                  [1, 3, 4]])
>>> inv(a) # méthode pour obtenir l'inverse d'une matrice 
array([[ 7., -3., -3.],
       [-1.,  1.,  0.],
       [-1.,  0.,  1.]])
```

Autres méthodes utiles :  
* **numpy.zeros()**: renvoie un tableau de zéros (marche aussi pour les tableaux 2D)
* **numpy.ones()**: renvoie un tableau de un (marche aussi pour les tableaux 2D)
* **numpy.eye()**: renvoie une matrice identité (tableau 2D avec des uns sur la diagonale)