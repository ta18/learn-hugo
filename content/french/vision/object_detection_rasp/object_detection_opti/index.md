---
title: "Quantification post-entraînement d'un réseau TFLite"
menu:
  main:
    name: "Quantification post-entraînement d'un réseau TFLite"
    weight: 3
    parent: "capsulesRSP"
---


| Classe de capsule  | &emsp;Durée recommandée |
|:-------------------|:------------------|
| Task  &emsp;  ⚙️  |&emsp; 10 min      |

## 🎒 Prérequis

* BAC+2 et +
* Bonne compréhension de Python et numpy
* Une première expérience des réseaux de neurones est souhaitable
* Une raspberry Pi avec caméra fonctionnelle
* Capsule sur la **Mise en place des modules sur la Raspberry Pi**
* Capsule sur la **Détection d'objet sur la Raspberry Pi**
* Capsule sur la **Conversion d'un réseau Tensorflow au format TFLite**

## 🎓 Acquis d'apprentissage

* savoir quantifier un réseau Tensorflow Lite.


## 📗 Documentation

Credit : 
* https://www.tensorflow.org/lite/performance/post_training_integer_quant
* https://www.tensorflow.org/lite/performance/post_training_quantization
* https://www.tensorflow.org/lite/performance/model_optimization


## Introduction

Certaines architectures embarquées sont assez limitées 
en terme de mémoire ou de puissance de calcul. 
Après avoir mis au format TFLite notre réseau entraîné, 
on souhaite maintenant l'optimiser. 
Parmi les optimisations possibles, il existe la quantification. 
D'après la documentation du Tensorflow : 


> La quantification fonctionne en réduisant la précision des nombres utilisés 
> pour représenter les paramètres d'un modèle, qui sont par défaut des nombres 
> à virgule flottante 32 bits. Cela se traduit par une taille de modèle plus 
> petite et un calcul plus rapide.

Il existe plusieures manières de quantifier un modèle selon l'utilisation 
qui va en être faite. Ici, l'objectif est d'exporter ce modèle sur un 
microcontrôleur à savoir une Raspberry Pi 4. Dans ce cas là, c'est la
quantification entière des nombres entiers qui est adapté. 
Cette technique de quantification consiste à convertir tous les poids et
sorties d'activation en données entières 8 bits.

## Etalonnage des tenseurs variables

Dans un premier temps, il faut calculer les valeurs minimum et maximum des tenseurs 
à virgule flottante (entrée du modèle, entrée du modèle, sortie du modèle) dans le modèle.
Pour ce faire, on définit un ensemble de données pour les étalonner. 

__Attention !__ L'ensemble de données doit être composé d'au moins 100 images. 
Il faut donc faire attention à entraîner son réseau avec plus d'une centaine d'images.

Le code ci-dessous définit la fonction pour étalonner à partir d'un **.pb** : 

```python 
def representative_dataset():
    folder = "<path_to_train_or_test_images>"
    image_size = 640
    raw_test_data = []

    files = glob.glob(folder+'/*.png')
    for file in files:
        image = Image.open(file)
        image = image.convert("RGB")
        image = image.resize((image_size, image_size))
        #Quantizing the image between -1,1;
        image = (2.0 / 255.0) * np.float32(image) - 1.0
        image = np.asarray(image).astype(np.float32)
        image = image[np.newaxis,:,:,:]
        raw_test_data.append(image)

    for data in raw_test_data:
        yield [data]
```

À des fins de test, on peut utiliser le code ci-dessous : 


```python 
def representative_dataset():
    for _ in range(100):
      data = np.random.rand(1, 640, 640, 3)
      yield [data.astype(np.float32)]
```

## Quantification

Le code suivant __convertit au format TFLite__ et quantifie un **.pb** :

```python 
import tensorflow as tf
converter = tf.lite.TFLiteConverter.from_saved_model(saved_model_dir)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
converter.representative_dataset = representative_dataset
tflite_quant_model = converter.convert()
```

Si le microcontrôleur est un TPU Coral Edge ou s'il ne supporte que les 
opérations d'entier, le modèle ne sera pas compatible car l'entrée et la sortie 
restent flottantes (float32).

Pour tester si la quantification a bien fonctionné, on peut afficher le type
des tenseurs d'entrée et de sortie  :  

```python 
interpreter = tf.lite.Interpreter(model_content=tflite_quant_model)
input_type = interpreter.get_input_details()[0]['dtype']
print('input: ', input_type)
output_type = interpreter.get_output_details()[0]['dtype']
print('output: ', output_type)
```

Pour la première quantification où les entrées et sorties restent en flottants, 
le résultat attendu est le suivant :

```python
input:  <class 'numpy.float32'>
output:  <class 'numpy.float32'>
```

## Ecriture dans un fichier

Pour exporter son réseau sur une architecture embarquée, 
le code suivant écrit le réseau dans un fichier :

```python
with open(<path_for_saving_model/model.tflite>.format(saved_model_dir), 'wb') as w:
    w.write(tflite_quant_model)
print("tflite convert complete!)
```