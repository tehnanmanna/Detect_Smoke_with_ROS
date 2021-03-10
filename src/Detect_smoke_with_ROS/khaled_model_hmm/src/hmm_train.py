#!/usr/bin/python

import cPickle
import os
import sys 
from hmmlearn.hmm import GaussianHMM
import numpy as np
from matplotlib import pyplot as plt
import seaborn as sns
import rospy 
from std_msgs.msg import String
from std_msgs.msg import Int16

pub = None
model = None
def plotTimeSeries(Q, hidden_states, ylabel, filename):
 
    sns.set()
    fig = plt.figure()
    ax = fig.add_subplot(111)
 
    xs = np.arange(len(Q))+1
    masks = hidden_states == 0
    ax.scatter(xs[masks], Q[masks], c='g', label='Normal State')
    masks = hidden_states == 1
    ax.scatter(xs[masks], Q[masks], c='r', label='Smoky State')
    ax.plot(xs, Q, c='k')
     
    ax.set_xlabel('Samples')
    ax.set_ylabel(ylabel)
    fig.subplots_adjust(bottom=0.2)
    handles, labels = plt.gca().get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', ncol=2, frameon=True)
    fig.savefig(filename)
    fig.clf()
 
    return None

def fitHMM(Q, nSamples):
	global model

	loaded = False
	if os.path.isfile('/home/khaled/saied_ws/src/khaled_model_hmm/src/train.pkl'):	
		# load it again
		with open('/home/khaled/saied_ws/src/khaled_model_hmm/src/train.pkl', 'rb') as fid:
			model = cPickle.load(fid)
			loaded = True
			print("loaded")

	if not loaded:
		print("training")
		# fit Gaussian HMM to Q
		model = GaussianHMM(n_components=2, n_iter=1000).fit(np.reshape(Q,[len(Q),1]))
		 
		# save the classifier
		with open('/home/khaled/saied_ws/src/khaled_model_hmm/src/train.pkl', 'wb') as fid:
			cPickle.dump(model, fid)

	# classify each observation as state 0 or 1
	hidden_states = model.predict(np.reshape(Q,[len(Q),1]))
 
	# find parameters of Gaussian HMM
	mus = np.array(model.means_)
	sigmas = np.array(np.sqrt(np.array([np.diag(model.covars_[0]),np.diag(model.covars_[1])])))
	P = np.array(model.transmat_)
 
	# find log-likelihood of Gaussian HMM
	Prob = model.score(np.reshape(Q,[len(Q),1]))
 
	# generate nSamples from Gaussian HMM
	samples = model.sample(nSamples)
	print(model.transmat_)
	print('score',Prob)
	print('hidden states',len(hidden_states))
	# re-organize mus, sigmas and P so that first row is lower mean (if not already)
	if mus[0] > mus[1]:
	    mus = np.flipud(mus)
	    sigmas = np.flipud(sigmas)
	    P = np.fliplr(np.flipud(P))
	    hidden_states = 1 - hidden_states

	return hidden_states, mus, sigmas, P, Prob, samples

def Callback(data):
	"""print(data.data)
	cluster=[]
	for i in range(10):
    	    cluster.append(data.data)
	print(cluster)
 	hidden_states, mus, sigmas, P, Prob, samples = fitHMM(cluster,10)
	#print(hidden_states)
    	#plt.switch_backend('agg') # turn off display when running with Cygwin
    	#plotTimeSeries(cluster, hidden_states, 'Number of Clusters', '/home/mohamed/catkin_ws/src/hmm_model/src/StateTseries_Log2.png')
"""
def cluster_data(): 

    rospy.init_node("HMM_model", anonymous=True)
    rospy.Subscriber('/cluster_num', Int16, Callback)
    

    Clusters = np.loadtxt('/home/khaled/saied_ws/src/khaled_model_hmm/src/khaled_log1.txt')
    # log transform the data and fit the HMM
    seq_len=len(Clusters)
    print('length',seq_len)
    hidden_states, mus, sigmas, P, Prob, samples = fitHMM(Clusters,seq_len)

    for i in range(seq_len):
    	if hidden_states[i]==1:
		print('Anamoly detected at sample',i)	    
    plt.switch_backend('agg') # turn off display when running with Cygwin
    plotTimeSeries(Clusters, hidden_states, 'Number of Clusters', '/home/khaled/saied_ws/src/khaled_model_hmm/src/StateTseries_Log22.png')
    rospy.spin()	

if __name__ == '__main__':
    try:
        cluster_data()
	
    except rospy.ROSInterruptException:
        pass
