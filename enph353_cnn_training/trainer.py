import math
import numpy
import numpy as np
import re
import cv2
import os
import copy
import gc

from collections import Counter
from matplotlib import pyplot as plt
from PIL import Image

from sklearn.model_selection import train_test_split

from keras import layers
from keras import models
from keras import optimizers
from keras import backend
from keras.preprocessing.image import ImageDataGenerator
from keras.preprocessing.image import img_to_array, load_img

from keras.utils import plot_model



PATH = "/home/rehan/enph353_cnn_training/pictures"
img_names = os.listdir(PATH)
num_of_letters = len(img_names)*4

# List of AlphaNumerics in order of pictures
plt_string = ""
for names in img_names:
    for x in range(0, 4):
        plt_string = plt_string + names[x] + " "
plt_letters = plt_string.split(" ")
plt_letters.pop()
letters = numpy.asarray(plt_letters)


def dictionary_maker():
    keys = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9"]
    dict = {
      "A": [],
      "B": [],
      "C": [],
      "D": [],
      "E": [],
      "F": [],
      "G": [],
      "H": [],
      "I": [],
      "J": [],
      "K": [],
      "L": [],
      "M": [],
      "N": [],
      "O": [],
      "P": [],
      "Q": [],
      "R": [],
      "S": [],
      "T": [],
      "U": [],
      "V": [],
      "W": [],
      "X": [],
      "Y": [],
      "Z": [],
      "0": [],
      "1": [],
      "2": [],
      "3": [],
      "4": [],
      "5": [],
      "6": [],
      "7": [],
      "8": [],
      "9": [],
    }

    for i in range(0, len(dict)):
        blank_arr = np.zeros(len(dict))
        blank_arr[i] = 1
        dict[keys[i]] = blank_arr

    return dict


alpha_dict = dictionary_maker()

letters_norm = []
for l in letters:
  letters_norm.append(alpha_dict.get(l))
letters_norm = np.array(letters_norm)
# print(letters_norm)

# #Array of Liscence Plate images
lic_plt = []
for files in img_names:
  s = PATH + "/" + files
  image = cv2.imread(s)
  lic_plt.append(image)
license_plate = numpy.array(lic_plt)


# Cropper
def lp_crop(lp_img):
  ## imgs are 600x298
  ## crop letters with 100x134 rectangle
  ## generate 4x1 array with cropped values


  ##first letter (l1) coordinates are 48 -> 148, 88 -> 222
  ##              l2                  148 -> 248
  ##              l3                  347 -> 447
  ##              l4                  447 -> 547
  crop0_img = lp_img[88:222, 48:148]
  crop1_img = lp_img[88:222, 148:248]
  crop2_img = lp_img[88:222, 347:447]
  crop3_img = lp_img[88:222, 447:547]

  crop_array = [crop0_img, crop1_img, crop2_img, crop3_img]
  croppy = numpy.array(crop_array)

  return(croppy)


alphaNum_list = []
for lp in license_plate:
  lp_single = lp_crop(lp)
  for alphaNum in lp_single:
    norm_alphaNum = copy.deepcopy(alphaNum)/255.0
    resize_alphaNum = cv2.resize(norm_alphaNum, (128, 128))
    alphaNum_list.append(resize_alphaNum)
alphaNum_list = np.array(alphaNum_list)

del license_plate
del lp_single
gc.collect()
# row, col = num_of_letters, 2
# combo_mat = [[0 for x in range(col)] for y in range(row)]
# for i in range(0, num_of_letters):
#   combo_mat[i][0] = alphaNum_list[i]
#   combo_mat[i][1] = letters_norm[i]
# # for i in range(0, 400):
# #   print(combo_mat[i][1])
# #   cv2_imshow(combo_mat[i][0])

LP_train, LP_val, truth_train, truth_val = train_test_split(alphaNum_list, letters_norm, test_size=0.20, random_state=2)

# print("shape of training images is:", LP_train.shape)
# print("shape of validation images is:", LP_val.shape)
# print("shape of training labels is:", truth_train.shape)
# print("shape of validation labels is:", truth_val.shape)

del alphaNum_list
del letters_norm
gc.collect()

ntrain = len(LP_train)
nval = len(LP_val)

batch_size = 128


def reset_weights(model):
    session = backend.get_session()
    for layer in model.layers:
        if hasattr(layer, 'kernel_initializer'):
            layer.kernel.initializer.run(session=session)


conv_model = models.Sequential()
conv_model.add(layers.Conv2D(32, (3, 3), activation='relu',
                             input_shape=(128, 128, 3)))
conv_model.add(layers.MaxPooling2D((2, 2)))
conv_model.add(layers.Conv2D(64, (3, 3), activation='relu'))
conv_model.add(layers.MaxPooling2D((2, 2)))
conv_model.add(layers.Conv2D(128, (3, 3), activation='relu'))
conv_model.add(layers.MaxPooling2D((2, 2)))
conv_model.add(layers.Conv2D(128, (3, 3), activation='relu'))
conv_model.add(layers.MaxPooling2D((2, 2)))
conv_model.add(layers.Flatten())
conv_model.add(layers.Dropout(0.5))
conv_model.add(layers.Dense(512, activation='relu'))
conv_model.add(layers.Dense(36, activation='softmax'))

LEARNING_RATE = 1e-4
conv_model.compile(loss='categorical_crossentropy',
                   optimizer=optimizers.RMSprop(lr=LEARNING_RATE),
                   metrics=['acc'])

reset_weights(conv_model)

train_datagen = ImageDataGenerator(rescale=1./255,
                                   rotation_range=40,
                                   width_shift_range=0.2,
                                   height_shift_range=0.2,
                                   shear_range=0.2,
                                   zoom_range=0.2,
                                   horizontal_flip=True,)

val_datagen = ImageDataGenerator(rescale=1./255)

train_generator = train_datagen.flow(LP_train, truth_train, batch_size=batch_size)
val_generator = val_datagen.flow(LP_val, truth_val, batch_size=batch_size)

history = conv_model.fit_generator(train_generator,
                                   steps_per_epoch=ntrain // batch_size,
                                   epochs=80,
                                   validation_data=val_generator,
                                   validation_steps=nval // batch_size)

conv_model.save_weights('trial1_weights.h5')
conv_model.save('trial1_keras.h5')

acc = history.history['acc']
val_acc = history.history['val_acc']
loss = history.history['loss']
val_loss = history.history['val_loss']

epochs = range(1, len(acc)+1)

plt.plot(epochs, acc, 'b', label='Training Accuracy')
plt.plot(epochs, val_acc, 'r', label='Validation Accuracy')
plt.title('Training and Validation Accuracy')
plt.legend()

plt.show()
