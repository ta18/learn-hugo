---
title: "Python : tracé de courbes avec matplotlib"
menu:
  main:
    name: "Python : tracé de courbes avec matplotlib"
    weight: 5
    parent: "capsules"
---

| Classe de capsule  | &emsp;durée recommandée |
|:-------------------|:------------------|
| Info  &emsp;  ℹ️  |&emsp; 10 min      |

## 🎒 Prérequis

- Lycée et +

## Acquis d'apprentissage
A l'issue de cette activité, l'apprenant saura : 
- Créer un graphique
- Modifier le visuel des graphiques 
- Mettre en forme le graphique
- Créer plusieurs courbes sur un graphique
- Afficher plusieurs graphiques sur la même image 
- Sauvegarder un graphique sous la forme souhaité 

## 📗 Documentation

Les informations de cette capsule sont tirées des liens suivants :
[Matplotlib](https://www.w3schools.com/python/matplotlib_intro.asp)

Memento Python à toujours avoir à coter de soi : 
{{<pdf src="https://perso.limsi.fr/pointal/_media/python:cours:mementopython3.pdf" >}}
 
📥 Si tu n'as pas installé **Anaconda** et que tu dispose juste de **MiniConda**, il te fait installer le package Matplotlib [ici](https://matplotlib.org/stable/users/installing.html).  

**Vocabulaire : array = tableau = matrice**  

Pour commencer a utiliser pyplot de matplotlib importer pyplot : 
`from matplotlib import pyplot as plt`

## 1. Tracer de graphe avec des points 

Pour tracer un graphe avec les coordonnées de plusieurs points dans l'ordre de la liste : 
``` python 
plt.plot([1, 2, 3, 6], [1, 4, 9, 36])
```  
![graphe](img/graphe.png)


### Tracer de droites horizontales ou verticales :  
* droite horizontale :
```python 
plt.axhline(y = 3)
``` 
* droite verticale : 
```python 
plt.axvline(x = 4)
```

### Tracer deux courbes sur un schéma :   
```python 
y1 = np.array([3, 8, 1, 10])
y2 = np.array([6, 2, 7, 11])

plt.plot(y1)
plt.plot(y2)

plt.show()
```
![graphe](img/graphe2.png)

Tracer deux courbes en précisant la valeur x et y des points de chaque courbes : 
```python 
x1 = np.array([0, 1, 2, 3])
y1 = np.array([3, 8, 1, 10])
x2 = np.array([0, 1, 2, 3])
y2 = np.array([6, 2, 7, 11])

plt.plot(x1, y1, x2, y2)
plt.show()
```

### Afficher plusieurs graphiques sur la même image : 

La fonction subplots() prend trois arguments qui décrivent la disposition de la figure : subplot(nbr de ligne, nbr de colonne, place du graphe)

```python
#plot 1:
x = np.array([0, 1, 2, 3])
y = np.array([3, 8, 1, 10])

plt.subplot(2, 1, 1)
plt.plot(x,y)

#plot 2:
x = np.array([0, 1, 2, 3])
y = np.array([10, 20, 30, 40])

plt.subplot(2, 1, 2)
plt.plot(x,y)

plt.show()
```
![graphe](img/graphex.png)

**Pour tracer d'autres types de graphiques :**
* nuage de points : 
```python 
plt.scatter()
```
* diagramme barres : 
```python 
plt.bar()
```
* histogramme : 
```python 
plt.hist()
```
* graphique camenbert  : 
```python 
plt.pie()
```

## 2. Mise en forme 

On peut mettre en forme les graphiques : 
* changer la couleur des courbes
* changer le style des courbes (en pointillés, tirés ...etc)
* changer la forme des marqueurs (étoile, rond ...etc)
* changer la taille des marqueurs
* mettre des titres 
* changer les axes
* mettre des titres aux axes 

Un exemple :  
```python 
plt.plot([1, 2, 4, 4, 2, 1], color = 'red', linestyle = 'dashed', linewidth = 2,
markerfacecolor = 'blue', markersize = 5)
plt.ylim(0, 5)
plt.title('Un exemple')
```

Différentes forme de marqueur [ici.](https://matplotlib.org/stable/api/markers_api.html)   
Syntaxe pour le changement de forme et de couleur [ici.](http://matplotlib.free.fr/subplot.html)


### Mettre en forme les axes :  
* étiquette des axes : 
```python 
plt.xlabel()
```
* titre d'un graphe : 
```python 
plt.title()
```
* fixer les limites des axes par : 
```python 
ax.set_xlim(), ax.set_ylim()
```
* fixer le titre d'un axe par : 
```python 
ax.set_title()
```
* fixer un entre 2 valeurs : 
```python 
plt.axis([0, 5, 0, 20])
```
* ajouter une grille : 
```python 
plt.grid()
```

### Sauvegarde de l'image dans un fichier :  
* Enregistrer une image : 
```python 
plt.savefig('image.png'); plt.close()
```
* Fixer la résolution : 
```python 
plt.savefig('image.png', dpi = 600)
```
* Fond transparent
 ```python 
plt.savefig('image.png', transparent = True)
```
* Indiquer le format (formats supportés : png, pdf, eps et svg) : 
```python 
plt.savefig('image, format = 'pdf')
``` 