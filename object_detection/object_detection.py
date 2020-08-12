# -*- coding: utf-8 -*-

from google.colab import drive
drive.mount('/content/drive')

"""Importing required packages"""

import tensorflow as tf
import math
import numpy as np
import matplotlib.pyplot as plt
import os
import pandas as pd
import cv2
import random
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout, Activation, Flatten
from tensorflow.keras.layers import Conv2D, MaxPooling2D
from sklearn.model_selection import train_test_split

"""Loading dataset from drive"""

images = []
labels = []
#X_img = []
img_path = '/content/drive/My Drive/dataset'
for file in os.listdir('/content/drive/My Drive/dataset'):
  thresh = cv2.imread(os.path.join(img_path, file), 0)
#  thresh = cv2.cvtColor(thresh, cv2.COLOR_BGR2HSV)
#  _, thresh = cv2.threshold(img, 155, 255, cv2.THRESH_BINARY)
#  canny = cv2.Canny(thresh, 155, 255)
#  cv2_imshow(canny)
  images.append(thresh)
  label.append(file[0])


"""Shuffling the dataset"""

random.shuffle(data_set)
# print(len(data_set))

X = []
y = []

for feature, label in data_set:
    X.append(feature)
    y.append(label)

X = np.array(X) 
y = np.array(y)

X_train,X_test,Y_train,Y_test = train_test_split(X,y,test_size = 0.15, random_state = 0)

"""Defining the model"""

model = Sequential()

model.add(Conv2D(256, (3, 3)))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Conv2D(256, (3, 3)))
model.add(Activation('relu'))
model.add(MaxPooling2D(pool_size=(2, 2)))

model.add(Flatten())  

model.add(Dense(64))

model.add(Dense(1))

model.add(Activation('sigmoid'))

model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])

"""Training model"""

model.fit(X, y, batch_size= 10, epochs=10, validation_split=0.3)

"""Testing"""

result = model.evaluate(X_test, Y_test, batch_size= 10)
print("accuracy:", result[1])

