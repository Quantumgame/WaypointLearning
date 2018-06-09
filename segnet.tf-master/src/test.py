from __future__ import division
import os, pdb
from PIL import Image
import numpy as np
from openravepy import *

from inputs import inputs
from scalar_ops import accuracy, loss

import classifier
import config
import tensorflow as tf
import utils

test_file, test_labels_file = utils.get_test_set(config.working_dataset, include_labels=True)

tf.app.flags.DEFINE_string('ckpt_dir', './ckpts', 'Train checkpoint directory')
tf.app.flags.DEFINE_string('test', test_file, 'Test data')
tf.app.flags.DEFINE_string('test_labels', test_labels_file, 'Test labels data')
tf.app.flags.DEFINE_string('test_logs', './logs/test', 'Log directory')

tf.app.flags.DEFINE_integer('batch', 1, 'Batch size') # was 200

FLAGS = tf.app.flags.FLAGS

bounds = [[-2.41, -2.41, 0], [2.41, 2.41, 0]] # all random envs are built using the same base world, thus they have the same world bounds
ppwu = 224 / (bounds[1][0] - bounds[0][0]) # number of pixels per unit in the robot world, images are all 224x224
pixelwidth = 1 / ppwu

def pixelBounds(pixel, pixelwidth=pixelwidth, bounds=bounds):
  '''
  obtain pixel bounds in terms of world coordinates
  '''
  pixminx = bounds[0][0] + (pixel[0] * pixelwidth)
  pixminy = bounds[1][1] - ((pixel[1] + 1) * pixelwidth)
  pixmaxx = bounds[0][0] + ((pixel[0] + 1) * pixelwidth)
  pixmaxy = bounds[1][1] - (pixel[1] * pixelwidth)
  b = [(pixminx, pixminy), (pixmaxx, pixmaxy)]

  return b
'''
# save random seed
assumes: model trained, test env & label's tfrecords already in correct folder
actions: passes test env through model, records green pixels and their bounds in OR coordinates in samples.txt in or_ompl catkin_ws folder, record ompl trajectory,
     compare traj with normal sampler and traj with mod sampler
'''

def test():
  images, labels = inputs(FLAGS.batch, FLAGS.test, FLAGS.test_labels)
  tf.summary.image('labels', labels)
  one_hot_labels = classifier.one_hot(labels)

  autoencoder = utils.get_autoencoder(config.autoencoder, config.working_dataset, config.strided)
  logits = autoencoder.inference(images)

  accuracy_op = accuracy(logits, one_hot_labels)
  tf.summary.scalar('accuracy', accuracy_op)

  saver = tf.train.Saver(tf.global_variables())
  summary = tf.summary.merge_all()
  summary_writer = tf.summary.FileWriter(FLAGS.test_logs)

  gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=config.gpu_memory_fraction)
  session_config = tf.ConfigProto(allow_soft_placement=True, gpu_options=gpu_options)
  with tf.Session(config=session_config) as sess:
    coord = tf.train.Coordinator()
    threads = tf.train.start_queue_runners(sess=sess, coord=coord)

    ckpt = tf.train.get_checkpoint_state(FLAGS.ckpt_dir)

    if not (ckpt and ckpt.model_checkpoint_path):
      print('No checkpoint file found')
      return

    ckpt_path = ckpt.model_checkpoint_path
    saver.restore(sess, ckpt_path)

    summary_str = sess.run(summary) 
    summary_writer.add_summary(summary_str)
    summary_writer.flush()

    coord.request_stop()
    coord.join(threads)
    
    # extract output label and save green pixel OR coordinate bounds
    summary_pb = tf.summary.Summary()
    summary_pb.ParseFromString(summary_str)
    
    for val in summary_pb.value:
      if 'output/image/' in val.tag:
        summary_img = val.image
        break

    output = sess.run(tf.image.decode_png(summary_img.encoded_image_string))
    '''
    from matplotlib import pyplot as plt
    plt.imshow(output, interpolation='nearest')
    plt.show()
    '''
    img = Image.fromarray(output)
    img.save(os.path.join('results', 'output.png'))

    greenpixelcount = 0
    with open(os.path.join('results', 'samples.txt'), 'w+') as sfile, open(os.path.join('results', 'numgreenpixels.txt'), 'w+') as gpfile:
      for width in xrange(img.size[0]):
        for length in xrange(img.size[1]):
          pixel = img.getpixel((width, length))

          if pixel == (0, 255, 0):
            greenpixelcount += 1
            pbounds = pixelBounds((width, length))
            pbstring = str(pbounds[0][0]) + ' ' + str(pbounds[0][1]) + ' ' + str(pbounds[1][0]) + ' ' + str(pbounds[1][1])
            sfile.write(pbstring + '\n')

      gpfile.write(str(greenpixelcount))

def main(argv=None):
  utils.restore_logs(FLAGS.test_logs)
  test()

if __name__ == '__main__':
  tf.app.run()
