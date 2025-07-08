# dbc_infer.py
import tensorflow.compat.v1 as tf
tf.disable_eager_execution()
import numpy as np
from model import DB_CNN as DB_Net

# class DBCNNPlanner:
#     def __init__(self, model_path, imsize=64):
#         self.imsize = imsize
#         self.X = tf.placeholder(tf.float32, shape=[None, imsize, imsize, 2], name='X')
#         self.S1 = tf.placeholder(tf.int32, shape=[None, 1], name='S1')
#         self.S2 = tf.placeholder(tf.int32, shape=[None, 1], name='S2')
#
#         _, self.prob_actions, _ = DB_Net(self.X, self.S1, self.S2, args=None)
#         self.actions = tf.argmax(self.prob_actions, 1)
#
#         self.sess = tf.Session()
#         saver = tf.train.Saver()
#         saver.restore(self.sess, model_path)
#         self.nA = 8  # 8 actions
class DBCNNPlanner:
    def __init__(self, model_path, imsize=64):
        self.imsize = imsize

        # 1. 模拟训练时的配置参数
        class DummyConfig:
            imsize = 64
            statebatchsize = 1
            ch_i = 2
            ch_h = 150
            ch_q = 8
            batch_size = 1
            k = 80  # 如果 DB_CNN 里用到了 config.k，则也必须提供
        config = DummyConfig()

        # 2. 占位符和网络图
        self.X = tf.placeholder(tf.float32, shape=[None, imsize, imsize, config.ch_i], name='X')
        self.S1 = tf.placeholder(tf.int32, shape=[None, config.statebatchsize], name='S1')
        self.S2 = tf.placeholder(tf.int32, shape=[None, config.statebatchsize], name='S2')

        # 3. 构建模型
        _, self.prob_actions, _ = DB_Net(self.X, self.S1, self.S2, config)
        self.actions = tf.argmax(self.prob_actions, 1)

        # 4. 初始化与模型恢复
        self.sess = tf.Session()
        saver = tf.train.Saver()
        saver.restore(self.sess, model_path)

        self.nA = config.ch_q


    def predict_path(self, input_map, start, goal):
        domain = np.stack([input_map == 1, input_map == 5], axis=-1).astype(np.float32).reshape(1, self.imsize, self.imsize, 2)

        canvas = input_map.copy()
        canvas[goal[0], goal[1]] = 5

        current = [start[0], start[1]]
        path = []
        max_steps = self.imsize * 2
        for _ in range(max_steps):
            path.append(tuple(current))
            s1 = np.array([[current[0]]], dtype=np.int32)
            s2 = np.array([[current[1]]], dtype=np.int32)

            act = self.sess.run(self.actions, feed_dict={
                self.X: domain,
                self.S1: s1,
                self.S2: s2
            })[0]

            dx, dy = self._action2cord(act)
            next_x, next_y = current[0] + dx, current[1] + dy

            if (next_x == goal[0] and next_y == goal[1]) or canvas[next_x, next_y] == 5:
                path.append((next_x, next_y))
                break
            if canvas[next_x, next_y] == 1 or not (0 <= next_x < self.imsize and 0 <= next_y < self.imsize):
                break
            current = [next_x, next_y]

        return path

    def _action2cord(self, a):
        return {'0':[-1,0],'1':[1,0],'2':[0,1],'3':[0,-1],'4':[-1,1],'5':[-1,-1],'6':[1,1],'7':[1,-1]}.get(str(a),[0,0])
