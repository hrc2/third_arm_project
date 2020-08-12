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


def enc_proc(data, self):
    all_keys    = list(data.keys())
    batch_size = 1
    #chosen_keys = np.random.choice( len(all_keys), self.batch_size )
    chosen_keys = np.random.choice( len(all_keys), batch_size )

    # How many frames in total do we need?
    total_frames = self.source_seq_len + self.target_seq_len

    encoder_inputs  = np.zeros((batch_size, self.source_seq_len-1, self.input_size), dtype=float)
    decoder_inputs  = np.zeros((batch_size, self.target_seq_len, self.input_size), dtype=float)
    decoder_outputs = np.zeros((batch_size, self.target_seq_len, self.input_size), dtype=float)

    for i in xrange(batch_size):

        the_key = all_keys[ chosen_keys[i] ]

        # Get the number of frames
        n, _ = data[ the_key ].shape

        # Sample somewherein the middle
        #idx = np.random.randint( 16, n-total_frames )
        idx = n-total_frames-1

        # Select the data around the sampled points
        data_sel = data[ the_key ][idx:idx+total_frames ,:]

        # Add the data
        encoder_inputs[i,:,0:self.input_size]  = data_sel[0:self.source_seq_len-1, :]
        decoder_inputs[i,:,0:self.input_size]  = data_sel[self.source_seq_len-1:self.source_seq_len+self.target_seq_len-1, :]
        decoder_outputs[i,:,0:self.input_size] = data_sel[self.source_seq_len:, 0:self.input_size]

    return encoder_inputs, decoder_inputs, decoder_outputs


cwd = os.getcwd()
# tr_path = cwd + '\Data\\wip_train.csv'
# va_path = cwd + '\Data\\wip_valid.csv'
# train = np.genfromtxt(tr_path, delimiter=',', dtype=float)
# valid = np.genfromtxt(va_path, delimiter=',', dtype=float)

sav_path = cwd + '/TrainedModel/checkpoint-2.meta'
sav_dir = cwd + '/TrainedModel'
result_dir = cwd + '/Results'

te_path = cwd + '/Data/wip_test.csv'
test = np.genfromtxt(te_path, delimiter=',', dtype=float)

FLAGS = tl.FLAGS

#gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=1, allow_growth=True)
#device_count = {"GPU": 0} if FLAGS.use_cpu else {"GPU": 1}

#sess = tf.Session(config=tf.ConfigProto( gpu_options=gpu_options, device_count = device_count ))

# trs = tf.train.Saver()
# mod = tf.train.import_meta_graph(sav_path)
# ckpt = tf.train.get_checkpoint_state(sav_dir, latest_filename="checkpoint")
# mod.restore( sess, ckpt.model_checkpoint_path )


def run(sess, mod):

    off = 70
    n_start = off
    n_end = test.shape[0] #- off - 2

    t_vec = np.array([], ndmin=2)
    p_vec = np.array([], ndmin=2)

    batch_size = 1
    encoder_inputs  = np.zeros((batch_size, mod.source_seq_len-1, mod.input_size), dtype=float)
    decoder_inputs  = np.zeros((batch_size, mod.target_seq_len, mod.input_size), dtype=float)
    decoder_outputs = np.zeros((batch_size, mod.target_seq_len, mod.input_size), dtype=float)

    for i in range(n_end - n_start):
        # tdat = test[i:i+off, :]
        # test_data = {"wrf": tdat}

        enc_in = test[i:i+mod.source_seq_len-1, :]

        if i == 0:
            dec_in = test[i+mod.source_seq_len-1 : i+mod.source_seq_len-1+mod.target_seq_len, :]
            dec_out = test[i+mod.source_seq_len : i+mod.source_seq_len+mod.target_seq_len, :]
        else:
            re = np.roll(dec_in, 1, axis=0)
            dec_in = re
            dec_in[0, :] = enc_in[-1, :]
            dec_in[-1, :] = dec_out[0, :]

            rd = np.roll(dec_out, -1, axis=0)
            dec_out = rd
            dec_out[-1, :] = pred[0, :]

        encoder_inputs[0, :, 0:mod.input_size] = enc_in
        decoder_inputs[0, :, 0:mod.input_size] = dec_in
        decoder_outputs[0, :, 0:mod.input_size] = dec_out


        #encoder_inputs, decoder_inputs, decoder_outputs = enc_proc(test_data, mod)

        # Prediction
        output_feed = [mod.loss, # Loss for this batch.
                       mod.outputs,
                       mod.loss_summary]

        input_feed = {mod.encoder_inputs: encoder_inputs,
                      mod.decoder_inputs: decoder_inputs,
                      mod.decoder_outputs: decoder_outputs}

        outputs = sess.run(output_feed, input_feed)

        pred_list = outputs[1]
        pred = np.ndarray(shape=(len(pred_list), test.shape[1]))

        for j in range(len(pred_list)):
            dd = pred_list[j]
            pred[j, :] = dd[0, :]

        #td = np.array(test[i+off, :].tolist(), ndmin=2)
        td = np.array(test[i+mod.source_seq_len+mod.target_seq_len, :].tolist(), ndmin=2)
        pd = np.array(pred[0, :].tolist(), ndmin=2)

        if i == 0:
            t_vec = td
            p_vec = pd
        else:
            t_vec = np.append(t_vec, td, axis=0)
            p_vec = np.append(p_vec, pd, axis=0)

        #print('Prediction: ', pred[0, :])
        #print('True: ', test[i+off, :])

    #print('Prediction Done')
    col = 1
    delt = t_vec[0, col] - p_vec[0, col]

    xt = np.arange(test.shape[0])
    xp = mod.source_seq_len + np.arange(p_vec.shape[0])

    ground_truth = test[xp, :]
    gtfile = result_dir + '/ground_truth.csv'
    np.savetxt(gtfile, ground_truth, delimiter=',')
    np.savetxt(result_dir + '/full_test_set.csv', test, delimiter=',')

    predfile = result_dir + '/pred_' + str(tl.FLAGS.iterations) + '.csv'
    np.savetxt(predfile, p_vec, delimiter=',')

    plt.plot(xt, test[:, col], 'b')
    #plt.plot(xp, p_vec[:, col] + delt, 'r')
    plt.plot(xp, p_vec[:, col], 'r')
    plt.show()

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

    print('Trained in {0} seconds'.format(time.time()-t_start))
    #mod = tl.create_model(sess, tl.FLAGS.action)

    run(sess, mod)