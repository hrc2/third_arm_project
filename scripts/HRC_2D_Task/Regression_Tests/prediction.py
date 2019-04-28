#!/usr/bin/env python

import numpy as np
from sklearn.linear_model import LogisticRegression
import math
#import csv
#from sklearn.model_selection import train_test_split
#import pandas as pd
#from sklearn import preprocessing
#from sklearn.metrics import accuracy_score, confusion_matrix
#import seaborn as sn

#PROC_FILE = 'Data/XktProc.npy'

class thirdarm_logit:

    def __init__(self):
        self.logreg = LogisticRegression()

    def assign_data(self, training_data, target_labels):
        self.Xtrain = training_data
        y = target_labels.tolist()
        y = np.array([int(num) for num in y], dtype=int)
        self.Ytrain = y

    def train(self):
        self.logreg.fit(self.Xtrain, self.Ytrain)

    def predict_probabilites(self, Xtest):
        y_pred_probs = self.logreg.predict_proba(Xtest)
        return y_pred_probs

