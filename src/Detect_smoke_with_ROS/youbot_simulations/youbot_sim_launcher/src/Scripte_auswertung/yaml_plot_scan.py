#!/usr/bin/env python
import sys
import matplotlib.pyplot as plt
import numpy as np
import yaml
import os

def load_file(filePath,file_name):
    dict_ = {}
    print "Loading: " + filePath+"/"+file_name
    try:
        print "Loading: "+file_name
        file = open(filePath+file_name,'r')
        dict_ = yaml.load(file)
    except yaml.YAMLError as exc:
        print(exc)
        print 'Failed to load: %s From: %s'
    file.close()
    return dict_


def main():
    path="/home/alex/wentz_catkin_ws/src/automatic_simulation/tests/yaml/"
    for f in os.listdir(path):
        if "images" in f:
            continue
        d = load_file(path, f)
        loss = []
        ids = []
        angels = []
        ref_range = []
        mean_range = []
        std = []
        params = {}
        total_loss = 0
        total_count = 0
        for key in d.keys():
            '''
            ids.append(key)
            loss.append(d[key]['loss'])
            angels.append(d[key]['angle'])
            ref_range.append(d[key]['ref_range'])
            mean_range.append(d[key]['mean_range'])
            std.append(d[key]['stdev'])
            '''
            #print key
            if key == "params":
                params = d[key]
                continue
            if d[key]['loss'] != 1.0:
                total_count += 1
                loss.append(d[key]['loss'])
                angels.append(d[key]['angle'])
                ref_range.append(d[key]['ref_range'])
                mean_range.append(d[key]['mean_range'])
                std.append(d[key]['stdev'])
            else:
                total_count += 1
                total_loss += 1

        absr = params['absorption']
        refl = params['reflectivity']
        trans = params['transmission']
        ang_fac = params['angular_factor']
        total_loss_percent = (float(total_loss)/float(total_count)) * 100
        total_loss_s = str(total_loss)+" / "+str(total_count)
        avg_loss = sum(loss)/len(loss)

        x = np.array(ref_range)
        y1 = np.array(mean_range)
        err_l = [i - j for i,j in zip(mean_range, ref_range)]
        yoff = np.array(err_l)
        fig, ax = plt.subplots()
        ax.set_ylim([-0.3,0.3])
        plt.xlabel("Distance")
        plt.ylabel("Distance Err")
        plt.title(f)
        plt.grid(True)
        plt.errorbar(x, yoff, yerr=std, fmt='r^')

        props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
        textstr = '\n'.join((
            r'transmission=%.2f' % (trans, ),
            r'absorption=%.2f' % (absr, ),
            r'reflection=%.2f' % (refl, ),
            r'angular_factor=%.2f' % (ang_fac, ),
            r'total_loss=%s' % (total_loss_s, ),
            r'avg_loss=%.2f' % (avg_loss, )))
        ax.text(0.65, 0.95, textstr,transform=ax.transAxes,
                fontsize=14, verticalalignment='top',
                bbox=props)

        #plt.show()
        plt.savefig(path+ "images/" + f.replace(".yaml",".png"))
        plt.close(fig)



if __name__=='__main__':
    main()
