#!/usr/bin/env python3
import os
import datetime

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
#import pandas as pd
#import seaborn as sns
import tensorflow as tf
import time


from six.moves import xrange # pylint: disable=redefined-builtin
import data_utils
import seq2seq_model
import translate as tl

import zmq
import rospy
from std_msgs.msg import ColorRGBA, Float32, Bool, Float64, Int32, String, Float32MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

def receive_mocap(msg, d1=50, d2=9, k=10):
    # Receive d1+k from first message to init the decoder
    
    data = np.fromstring(msg, dtype=float)

    if data.size == d1*d2:
        mocap_arr = data.reshape(d1, d2)
        return mocap_arr, [], []
    elif data.size == (d1+k)*d2:
        dat = data.reshape(d1+k, d2)
        return dat[0:d1, :], dat[d1-1:-1, :], dat[d1:, :]


class rnn_server:

    def __init__(self, sess, mod):
        rospy.init_node('rnn_predictor')
        self.mocap = np.array([], ndmin=2)
        self.dec_in = np.array([])
        self.dec_out = np.array([])
        self.pred_pub = rospy.Publisher( '/rnn_pred', numpy_msg(Floats), queue_size=10)
        self.sess = sess
        self.mod = mod
        self.k = 10
        self.pred = np.zeros((self.k, self.mod.input_size), dtype=float)

        data = rospy.wait_for_message('/rnn_sample', numpy_msg(Floats))
        arr = data.data
        d1 = 50
        d2 = 9
        k = self.k        
        dat = arr.reshape(d1+k, d2)
        self.mocap = dat[0:d1, :]
        self.dec_in = dat[d1-1:-1, :]
        self.dec_out = dat[d1:, :]
        print('Initial Mocap: ', self.mocap)

        rospy.Subscriber('/rnn_sample',  numpy_msg(Floats), self.receive_rnn_sample)

    def receive_rnn_sample(self, data):
        d1=50
        d2=9
        k = self.k
        arr = data.data
        if arr.size == d1*d2:
            self.mocap = arr.reshape(d1, d2)
        elif arr.size == (d1+k)*d2:
            dat = arr.reshape(d1+k, d2)
            self.mocap = dat[0:d1, :]
            self.dec_in = dat[d1-1:-1, :]
            self.dec_out = dat[d1:, :]
        else:
            self.mocap = self.mocap[-d1:, :]                   
        


    def prepare_step(self, data, dec_in, dec_out):
        batch_size = 1
        encoder_inputs  = np.zeros((batch_size, mod.source_seq_len-1, mod.input_size), dtype=float)
        decoder_inputs  = np.zeros((batch_size, mod.target_seq_len, mod.input_size), dtype=float)
        decoder_outputs = np.zeros((batch_size, mod.target_seq_len, mod.input_size), dtype=float)  

        encoder_inputs[0, :, 0:mod.input_size] = data[0:-1, :]
        decoder_inputs[0, :, 0:mod.input_size] = dec_in
        decoder_outputs[0, :, 0:mod.input_size] = dec_out

        return encoder_inputs, decoder_inputs, decoder_outputs


    def predict(self, sess, mod, encoder_inputs, decoder_inputs, decoder_outputs):
        # Prediction
        output_feed = [mod.loss, # Loss for this batch.
                       mod.outputs,
                       mod.loss_summary]

        input_feed = {mod.encoder_inputs: encoder_inputs,
                      mod.decoder_inputs: decoder_inputs,
                      mod.decoder_outputs: decoder_outputs}

        outputs = sess.run(output_feed, input_feed)

        pred_list = outputs[1]
        pred = np.ndarray(shape=(len(pred_list), mod.input_size))

        for j in range(len(pred_list)):
            dd = pred_list[j]
            pred[j, :] = dd[0, :]

        #pred = np.array(pred[0, :].tolist(), ndmin=2)
        return pred[0, :]

    def run(self):      

        for j in range(self.k):
            enc_in_proc, dec_in_proc, dec_out_proc = self.prepare_step(self.mocap, self.dec_in, self.dec_out)
            self.pred[j, :] = self.predict(self.sess, self.mod, enc_in_proc, dec_in_proc, dec_out_proc)            
            
            re = np.roll(self.dec_in, 1, axis=0)
            self.dec_in = re
            self.dec_in[0, :] = self.mocap[-1, :]
            self.dec_in[-1, :] = self.dec_out[0, :]
            rd = np.roll(self.dec_out, -1, axis=0)
            self.dec_out = rd
            self.dec_out[-1, :] = self.pred[j, :]

        print('Prediction: ', self.pred)  
        outvec = np.ravel(self.pred)
        nans = np.sum(np.isnan(outvec))
        if nans == 0:
            self.pred_pub.publish(outvec)


if __name__ == '__main__':

    t_start = time.time()
    iter = 10
    tl.FLAGS.load = 0
    tl.FLAGS.action = "wrf"
    tl.FLAGS.iterations = iter
    tl.FLAGS.save_every = iter
    tl.FLAGS.test_every = iter
    tl.FLAGS.batch_size = 20
    sess, mod = tl.train()
    print('Trained model in {0} seconds'.format(time.time()-t_start))
    
    # i = 0    

    #context = zmq.Context()
    #sock = context.socket(zmq.REP)
    #sock.bind("tcp://*:5555")
    #rec_sock = context.socket(zmq.REQ)
    #rec_sock.connect("tcp://localhost:5555")

    node = rnn_server(sess, mod)        

    #while True:
    while not rospy.is_shutdown():        
        node.run()                
        # pred_msg =  pred.tostring()
        # sock.send(pred_msg)
    rospy.spin()

    