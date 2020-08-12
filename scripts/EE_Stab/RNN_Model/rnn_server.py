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




def receive_mocap(msg, d1=50, d2=9, k=10):
    # Receive d1+k from first message to init the decoder
    
    data = np.fromstring(msg, dtype=float)

    if data.size == d1*d2:
        mocap_arr = data.reshape(d1, d2)
        return mocap_arr, [], []
    elif data.size == (d1+k)*d2:
        dat = data.reshape(d1+k, d2)
        return dat[0:d1, :], dat[d1-1:-1, :], dat[d1:, :]

# def send_pred(data):
#     msg =  data.tostring()
#     send_sock.send(msg)

def prepare_step(data, dec_in, dec_out):
    batch_size = 1
    encoder_inputs  = np.zeros((batch_size, mod.source_seq_len-1, mod.input_size), dtype=float)
    decoder_inputs  = np.zeros((batch_size, mod.target_seq_len, mod.input_size), dtype=float)
    decoder_outputs = np.zeros((batch_size, mod.target_seq_len, mod.input_size), dtype=float)  

    encoder_inputs[0, :, 0:mod.input_size] = data[0:-1, :]
    decoder_inputs[0, :, 0:mod.input_size] = dec_in
    decoder_outputs[0, :, 0:mod.input_size] = dec_out

    return encoder_inputs, decoder_inputs, decoder_outputs



def predict(sess, mod, encoder_inputs, decoder_inputs, decoder_outputs):
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
    
    i = 0
    k = 10
    pred = np.zeros((k, mod.input_size), dtype=float)

    context = zmq.Context()
    sock = context.socket(zmq.REP)
    sock.bind("tcp://*:5555")
    #rec_sock = context.socket(zmq.REQ)
    #rec_sock.connect("tcp://localhost:5555")

    while True:
        msg = sock.recv()
        mocap, d_in, d_out = receive_mocap(msg)
        
        if len(d_in) !=0:
            dec_in = d_in
            dec_out = d_out

        for j in range(k):
            enc_in_proc, dec_in_proc, dec_out_proc = prepare_step(mocap, dec_in, dec_out)
            pred[j, :] = predict(sess, mod, enc_in_proc, dec_in_proc, dec_out_proc)            
            
            re = np.roll(dec_in, 1, axis=0)
            dec_in = re
            dec_in[0, :] = mocap[-1, :]
            dec_in[-1, :] = dec_out[0, :]
            rd = np.roll(dec_out, -1, axis=0)
            dec_out = rd
            dec_out[-1, :] = pred[j, :]

        print('Prediction: ', str(pred))    
            
        pred_msg =  pred.tostring()
        sock.send(pred_msg)


    