#!/usr/bin/env python

import numpy as np
from sklearn import neighbors
from sklearn import preprocessing
import math
#import csv
#from sklearn.model_selection import train_test_split
#import pandas as pd
#from sklearn import preprocessing
#from sklearn.metrics import accuracy_score, confusion_matrix
#import seaborn as sn

class thirdarm_task_prediction:

    def __init__(self):
        self.time_threshold = 1.0
        self.data_buffer = np.array([], ndmin=2)
        self.command_buffer = np.array([], ndmin=2)
        self.N = 20
        self.x_train = np.array([], ndmin=2)
        self.y_train = np.array([], ndmin=2)
        self.relevant_commands = []
        self.do_nothing_label = []

        self.knn_model = neighbors.KNeighborsClassifier(n_neighbors=self.N, weights='distance')
        self.label_encoder = preprocessing.LabelEncoder()

    def add_to_data_buffer(self, xdata):
        if self.data_buffer.size == 0:
            self.data_buffer = xdata
        else:
            self.data_buffer = np.append(self.data_buffer, xdata, axis=0)

    def add_to_command_buffer(self, ydata):
        if self.command_buffer.size == 0:
            self.command_buffer = ydata
        else:
            self.command_buffer = np.append(self.command_buffer, ydata, axis=0)

    def set_relevant_commands(self, command_labels, dn_label):
        self.relevant_commands = command_labels
        self.do_nothing_label = dn_label
        self.label_encoder.fit_transform(self.relevant_commands)

    def process_data(self):
        T_y = self.command_buffer[:, -1].astype(float)
        T_x = self.data_buffer[:, -1]
        self.x_train = np.array([], ndmin=2)
        self.y_train = np.array([])
        ind_set = np.array([])

        for i in range(len(T_y)):
            y_curr = self.command_buffer[i, 0]
            if y_curr in self.relevant_commands:
                i2 = np.where(T_x < T_y[i])[0]
                ind = np.where(T_x[i2] >= T_y[i] - self.time_threshold)[0]
                if i == 0:
                    ind_set = ind
                    self.x_train = self.data_buffer[ind, :-1]
                    self.y_train = np.repeat(self.label_encoder.transform([y_curr]), len(ind))
                else:
                    ind_set = np.append(ind_set, ind)
                    self.x_train = np.append(self.x_train, self.data_buffer[ind, :-1], axis=0)
                    self.y_train = np.append(self.y_train, np.repeat(self.label_encoder.transform([y_curr]), len(ind)))

        resid = np.delete(self.data_buffer[:, :-1], ind_set, axis=0)
        self.x_train = np.append(self.x_train, resid, axis=0)
        self.y_train = np.append(self.y_train, np.repeat(self.label_encoder.transform(self.do_nothing_label), len(resid)))

    def train(self):
        self.knn_model.fit(self.x_train, self.y_train)

    def predict(self, x_test):
        y_pred = self.knn_model.predict(x_test)
        return self.label_encoder.inverse_transform(y_pred)

    def predict_probabilites(self, x_test):
        y_pred_probs = self.knn_model.predict_proba(x_test)
        return y_pred_probs

