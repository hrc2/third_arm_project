#!/usr/bin/env python

import numpy as np
from sklearn.linear_model import LogisticRegression
import math
from sklearn.cluster import KMeans
from jumpsmethod import JumpsMethod
#import csv
#from sklearn.model_selection import train_test_split
#import pandas as pd
#from sklearn import preprocessing
#from sklearn.metrics import accuracy_score, confusion_matrix
#import seaborn as sn

#PROC_FILE = 'Data/XktProc.npy'

class thirdarm_logit:

    def __init__(self, jump_init_number=5):
        self.logreg = LogisticRegression()
        self.data_buffer = np.array([], ndmin=2)
        self.label_buffer = np.array([], ndmin=2)
        self.Xkt = np.array([])
        self.Xnt = np.array([])
        self.Ykt = np.array([])
        self.Ynt = np.array([])
        self.num_targets = 0
        self.trial_number = 0
        self.target_positions = np.array([])
        self.jump_init_number = jump_init_number
        self.target_labels = np.array([])
        self.target_means = np.array([])

    def add_to_data_buffer(self, X_current, Y_current):
        if(self.Xkt.size == 0):
            self.Xkt = np.array(X_current, ndmin=2)
            self.Ykt = np.array(Y_current, ndmin=2)
            #self.Tnt = np.array(tk, ndmin=2)
        else:
            self.Xkt = np.append(self.Xkt, np.array(X_current, ndmin=2), axis=0)
            self.Ykt = np.append(self.Ykt, np.array(Y_current, ndmin=2), axis=0)
            #self.Tnt = np.append(self.Tnt, np.array(tk, ndmin=2), axis=0)

    def update_train_buffer(self):
        if (self.Xnt.size == 0):
            self.Xnt = self.Xkt
            self.Ynt = self.Ykt
        else:
            self.Xnt = np.append(self.Xnt, self.Xkt, axis=0)
            self.Ynt = np.append(self.Ynt, self.Ykt, axis=0)

    def assign_data(self):
        self.Xtrain = self.Xnt
        y = np.ravel(self.Ynt[:, 0]).tolist()
        y = np.array([int(num) for num in y], dtype=int)
        self.Ytrain = y

    def process_data(self, trial_number):
        self.trial_number = trial_number
        jm = JumpsMethod(self.target_positions)
        jm.Distortions(cluster_range=range(1, self.target_positions.shape[0]), random_state=0)
        jm.Jumps(Y=0.1)
        self.num_targets = jm.recommended_cluster_number
        if self.num_targets != 2:
            self.num_targets = 2
        km = KMeans(n_clusters=self.num_targets)
        self.target_labels = km.fit_predict(self.target_positions) + 1
        self.reprocess_Yn()
        self.compute_target_means()

    def compute_target_means(self):
        labels = np.unique(self.target_labels)
        self.target_means = np.zeros([len(labels), 2])
        for i in range(len(labels)):
            ind = np.where(self.target_labels == labels[i])[0]
            self.target_means[i, :] = np.mean(self.target_positions[ind, :], axis=0)

    def reprocess_Yn(self):
        for i in range(0, int(self.trial_number)):
            ind = np.where(self.Ynt[:, 1].astype(int) == i+1)[0]
            self.Ynt[ind, 0] = self.target_labels[i]

    def update_target_data(self, target_data):
        if (self.target_positions.size == 0):
            self.target_positions = np.array(target_data, ndmin=2)
        else:
            self.target_positions = np.append(self.target_positions, target_data, axis=0)

    def get_target_means(self):
        return self.target_means

    def train(self):
        self.assign_data()
        self.logreg.fit(self.Xtrain, self.Ytrain)

    def predict_probabilites(self, Xtest):
        y_pred_probs = self.logreg.predict_proba(Xtest)
        return y_pred_probs

