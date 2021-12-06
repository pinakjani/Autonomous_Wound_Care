#!/usr/bin/env python3
import tensorflow as tf
import cv2
print(tf.__version__)
import os
import numpy as np
from tensorflow import keras
from tensorflow.keras import layers
from tensorflow.keras.models import Sequential, Model
new_model = tf.keras.models.load_model('best_model.h5')

image = cv2.imread('/home/kartik/catkin_ws/src/ug_project/autodoc/scripts/woundimage.png',cv2.COLOR_BGR2RGB)
image=cv2.resize(image, (150, 150),interpolation = cv2.INTER_AREA)
image1=np.array(image)
image1 = image1.astype('float32')
image1 /= 255 
cv2.imshow('window',image)
cv2.waitKey()
img = []
img.append(image1)
img = np.array(img)
# print(img.shape)
print("Running model...............................")
prediction = new_model.predict(img)
# print(prediction[0])
print("Predicting model...............................")
pred = prediction[0]
print(pred)
maxi = pred.max()
for i in range(len(pred)):
  if pred[i]==maxi:
    out = i
    break
if out == 0:
  print("Predicted: Abrasion Wound")
elif out ==1:
  print("Predicted: Bruise Wound")
elif out == 2:
  print("Predicted: Burn Wound")
else:
  print("Predicted: Cut Wound")
