---
title: "AI applications for artificial vision: overview"
menu:
  main:
    name: "Computer vision: overview"
    weight: 1
    parent: "vision"
---

Artificial Intelligence has developed very strongly for more than 10 years with a acceleration in the last 5 years.<br>
This overview addresse the main applications of IA in the field of computer vision.

## Main applications of AI in the field of computer vision

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

AI image generators are complex tool allow you to generate images from text descriptions. 

https://aiartists.org/ai-generated-art-tools

### Image Processing

AI is used today to make tratments on images which were until carried out thanks to _conventional image processing_ based on mathematical algorithms involing 2D discrete functions. A lot of treatments are 

image generator based on Natural Language Processing (NLP) networks used to process a textual description of the content of the image and then genertae the image. The GP-3 algorithm is often used (refernce : https://arxiv.org/abs/2005.14165).
https://openai.com/blog/dall-e/

image upscaling
https://topten.ai/image-upscalers-review/



# Useful links

https://deeplearning.quora.com/
