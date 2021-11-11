---
title: "IA applications: overview"
menu:
  main:
    name: "IA applications: overview"
    weight: 1
    parent: "vision"
---

Artificial Intelligence has developed very strongly for more than 10 years with a
acceleration in the last 5 years. The main fields adressed by AI are:

## A - Visual perception: classification of images, description of scenes, recognition of objects, gestures...

### Classification of images

<table>
  <tr>
    <td> <img src="img/image_classification_sunflowers.png" alt="sunflower" style="width:250px;"></td>
    <td><b>Image Classification</b> attempts to analyze an **entire image as a whole** : the goal is to classify the image by assigning it to a specific label.<br><br>
Image Classification refers to images in which only one object appears and is analyzed.<br>
In contrast, object detection involves both classification and localization tasks, and is used to analyze more realistic cases in which many objects may appear in an image.<br>
A large choice of image databases freely available on the Internet makes it possible to train and test neural networks for image classification (see for example <a href="https://paperswithcode.com/datasets">paperswithcode.com</a></td>
  </tr> 
  <tr>
  <td>Applications</td>
</table> 


### Detection of objects in images

<table>
  <tr>
    <td>
<img src="img/object_detection_aple-banana.png" alt="ObjetcDetection" style="width:250px;"><br>
<font size="2">Image credit: <a href="https://www.tensorflow.org/lite/examples/object_detection/overview">Tensorflow</a></font>
    </td>
    <td>
<b>Object Detection</b> attempts to detect objects of a certain class within an image.<br>
Each detected object has at least 3 attributes:<br>
- a label which identifies the class of the object<br>- a bounding box which allow the localization of the object in the image<br>
- a percentage of confidence which gives the quality of the detection.<br><br>
    
Many algorithms exist with different strengths / weaknesses:<br>
- one-stage algorithms: prioritize inference speed (YOLO, SSD, RetinaNet ...)<br>
- Two-stage algorithms: prioritize detection accuracy, like <em>Faster R-CNN</em>, <i>Mask R-CNN</i> and <i>Cascade R-CNN</i>
    </td>
  </tr> 
  <tr>
  <td>Applications</td>
  <td>
   - Robotic vision<br>
   - Face Recognition on Social Networks<br> 
   - <a href="https://www.pyimagesearch.com/2020/09/21/opencv-automatic-license-number-plate-recognition-anpr-with-python">
    Automatic License Plate Recognition <:a>

  </td>
</table> 

<br>

### Image Segmentation

<table>
  <tr>
    <td>
<img src="img/image_segmentation_Detectron.png" alt="segmentation" style="width:250px;"><br>
<font size="2">Image credit: <a href="https://github.com/facebookresearch/detectron">Detectron</a></font>
    </td>
    <td>
<b>Image Segmentation</b> 
consist in clustering parts of an image together which belong to the same object class.

It is a form of pixel-level prediction because each pixel in an image is classified according to a category: the task of image segmentation is to train a neural network to output a pixel-wise mask of the image.

A large choice of image databases freely available on the Internet makes it possible to train and test neural networks for image segmentation (see for example
<a href="https://paperswithcode.com/datasets?task=semantic-segmentation">paperswithcodes.com</a>).
    </td>
  </tr> 
  <tr>
  <td>Applications</td>
  <td>
  - tumor segmentation  (example: <a href="https://www.nature.com/articles/s41598-021-90428-8">www.nature.com report</a>)<br>
  - 3D Semantic Segmentation (example: <a href="https://paperswithcode.com/paper/pointnet-deep-learning-on-point-sets-for-3d">recognition of common objects in 3D point cloud</a>)<br>
  - Panoptic Segmentation (example: <a href="https://kharshit.github.io/blog/2019/10/18/introduction-to-panoptic-segmentation-tutorial">tutorial</a>)<br>
  - Elimination of background in image or video  (example: <a href="https://www.louisbouchard.ai/remove-background/">Remove any Background Without Green Screens</a>, paper: <a href="https://arxiv.org/pdf/2011.11961.pdf">Is a Green Screen Really Necessary for Real-Time Human Matting?</a>)
  </td>
  </tr>
</table> 

### Pose estimation

<table>
  <tr>
    <td>
<img src="img/pose_estimation_TF.png" alt="poseEstimation" style="width:250px;"><br>
<font size="2">Image credit: <a href="https://www.tensorflow.org/lite/examples/pose_estimation/overview">TesnsorFlow</a></font>
    </td>
    <td>
<b>Pose estimation</b> attempts to analyze an **entire image as a whole** : the goal is to classify the image by assigning it to a specific label.<br><br>
Pose estimation is the task of using an neural network to estimate the pose of a person from an image or a video by estimating the spatial locations of key body joints (keypoints)
    </td>
  </tr> 
  <tr>
  <td>Applications</td>
</table> 

### Image Generation


###|Image Denoising


## B - Dealing with acoustic signal

Recurrent neural networks are designed to learn form temporal data like sound signal. 
Many acoustics application can be dsigned thanks to these recurrent networks. 

### Isolate Voice, Music, and Sound Effects

<table>
  <tr>
    <td>
<img src="img/the_coktail_fork_problem.png" alt="poseEstimation" style="width:'50px;"><br>
<font size="2">Image credit: <a href="https://cocktail-fork.github.io">cocktail-fork.github.io</a></font>
    </td>
    <td>
In this application a sound track is splitted in three chanels (music, speed and sound effects) thanks to a succcession of processing including recurrent neral networks specifically trained.<br><br>
<em>The cocktail party problem aims at isolating any source of interest within a complex acoustic scene, and has long inspired audio source separation research. Recent efforts have mainly focused on separating speech from noise, speech from speech, musical instruments from each other, or sound events from each other...</em><br>
more in the paper <a ref="https://arxiv.org/abs/2110.09958">The Cocktail Fork Problem...</a>
    </td>
  </tr> 
  <tr>
  <td>Applications</td>
</table> 

](/))</font>| 



# B - Understanding of written or spoken natural language: machine translation, production
automatic press articles, sentiment analysis.

# C - Automatic analysis by "understanding" a query and returning relevant results
even if this result does not contain the words of the query.

# D - Autonomous decision-making to beat humans at chess or Go. 

Apart from technological aspects, AI also poses new questions in terms of __ethics__ and
__risk__: dependence on automation, misuse or error linked to "contaminated" data, impact on
privacy, etc. 


# Current trends in learning techniques 

- Generative adversarial networks (GAN)

- Lean and augmented data learning

- Probabilistic programming

- Automated machine learning (AutoML)

- Digital Twin


# Useful links

https://deeplearning.quora.com/
