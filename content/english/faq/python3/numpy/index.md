---
title: "🔨 numpy ndarrays tips & tricks "
menu:
  main:
    name: "numpy"
    weight: 2
    parent: "python3"
---

Within this page `numpy` is supposed to be imported as:
```python
>>> import numpy as np
```

## keywords
[argmax](#argmax) [shape](#shape)  [slicing](#slicing)

### shape

the attribute `shape` of an ndarray object return its shape as a tuple:

```python
>>> V = np.array([1,2,3])
>>> V.shape
(3,)                                       # (3) is a scalar but (3,) is a tuple!
>>> V = np.array([[1,2,3],[4,5,6]])
>>> V.shape
(2, 3)
``` 
[top](#keywords)

## argmax

the method `argmax` returns the rank of the max value in the ndarray:

```python
>>> V = np.array([1,1,1,100,1])
>>> V.max()
100
>>> V.argmax()
3
>>> V[V.argmax()]
100
```
[top](#keywords)

### slicing

You can extract a sub-ndarray with the slicing syntaxe `ndarray[start:stop:setp]`:

```python
>>>
```
[top](#keywords) 
