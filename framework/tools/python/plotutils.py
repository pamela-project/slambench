#!/usr/bin/python2


# DO not use DISPLAY within matplotlib
#import matplotlib as mpl
#mpl.use('Agg')
import matplotlib.pyplot as plt
# End of BugFix

import matplotlib
import pylab as plot

from slamlog import *


#########################################################################################
# UTILS PLOT
#########################################################################################


matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42


params = {'legend.fontsize': 22,
          'xtick.labelsize' : 20,
          'ytick.labelsize' : 20,
          'figure.titlesize': 20,
          #'legend.linewidth': 2,          
#          'font.size' : 8,
         }



plot.rcParams.update(params)


#font = {
#        'size'   : 18}
#matplotlib.rc('font', **font)




#########################################################################################
# UTILS ALGO
#########################################################################################



labels = {
    'kfusion-cuda'      : "KF-CUDA",
    'kfusion-opencl'    : "KF-OCL",
    'kfusion-openmp'    : "KF-OMP",
    'kfusion-cpp'       : "KF-CPP",
    'kfusion-notoon'    : "KF-NOT",
    
    'kfusion-octree-openmp'    : "KO-OMP",
    'kfusion-octree-cpp'       : "KO-CPP",
    
    'efusion-cuda'      : "EF-CUDA",
    
    'orbslam2-original' : "OS2-CPP",
    
    'infinitam-cpp'     : "IT-CPP",
    'infinitam-openmp'     : "IT-OMP",
    'infinitam-cuda'    : "IT-CUDA",
    
    'lsdslam-cpp'       : "LS-CPP",
    'lsdslam-original_mp'       : "LS-PTH",

    'ptam-original_mp'       : "PT-PTH",
    
    'svo-original'       : "SV-ORI",
    'okvis-original'       : "OK-ORI",
    'monoslam-sequential'       : "MO-CPP",
    
    
}

   



def getlabel(name) :
    for lab in labels.keys() :
        if lab in name :
            return labels[lab]
    printerr("Name '%s' not found in labels '%s'\n" % (name,labels))
    exit(1)
    

short_list = [
    "living_room_traj0_loop.slam",
    "living_room_traj1_loop.slam",
    "living_room_traj2_loop.slam",
    "living_room_traj3_loop.slam",
    "rgbd_dataset_freiburg1_rpy.slam" ,
    "rgbd_dataset_freiburg2_rpy.slam" ,
    "rgbd_dataset_freiburg1_xyz.slam" ,
    "rgbd_dataset_freiburg2_xyz.slam" ,
]



#############################################################################################
########     VISUALIZATION
#############################################################################################




def generate_violins (data) :

    violins = {}

    datasets = []
    for a in data :
        datasets = list ( set (  datasets + data[a].keys()  )  ) 
    algorithms = data.keys()

    
    for algorithm in data :
        for dataset in data[algorithm] :
            for it in data[algorithm][dataset] :
                if not STATISTICS_SECTION in it.keys() :
                    printerr ("Invalid data (%s,%s), no STATISTICS_SECTION.\n" % (algorithm,dataset))
                    exit(1)

    
                if not ATE_COLUMN in it[STATISTICS_SECTION].keys() :
                    printerr ("Invalid data(%s,%s), no ATE_COLUMN.\n" % (algorithm,dataset))
                    exit(1)

    

    for algorithm in data :
        MeanATE = []
        FPS     = []
        Memory  = []
        for dataset in data[algorithm] :
            
            for it in data[algorithm][dataset] :
                MeanATE += [it[STATISTICS_SECTION][ATE_COLUMN][MEAN_FIELD]]
                Memory  += [it[STATISTICS_SECTION][CPU_MEMORY_COLUMN][MAX_FIELD] / 1000000]
                FPS     += [1 / it[STATISTICS_SECTION][DURATION_COLUMN][MEAN_FIELD]]
            
            
        violins[algorithm] = { ATE_COLUMN : MeanATE , FPS_COLUMN : FPS , CPU_MEMORY_COLUMN : Memory }

    return violins



def plot_violins (data, filename , order) :

    violins = data

    ## PLOT DATA ##


    
    algo_long_name  = [x for x in  violins.keys() if getlabel(x) in order]
    algo_short_name = [getlabel(x) for x in  violins.keys() if getlabel(x) in order]

    algo_position   = [order.index(x) for x in algo_short_name]

    
    algo_ordered    =  [x for _,x in sorted(zip(algo_position,algo_long_name))]
 
    algos = algo_ordered
    pos   = [x for x in xrange(len(algos))]

    if len(pos) == 0 :
        printerr("Empty data, cannot draw anything.\n")
        return False
    

    fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(18, 5))

    FPS_DATA = [ violins[algo][FPS_COLUMN] for algo in algos ]
    ATE_DATA = [ violins[algo][ATE_COLUMN] for algo in algos ]
    MEM_DATA = [ violins[algo][CPU_MEMORY_COLUMN] for algo in algos ]

    LABELS   = [ getlabel(algo) for algo in algos ]
    
    axes[0].violinplot(FPS_DATA, pos, points=60, widths=0.7, showmeans=True, showextrema=True, showmedians=True, bw_method='silverman')
    axes[0].set_title('Speed (FPS)' , fontsize = 20)

    axes[1].violinplot(ATE_DATA, pos, points=60, widths=0.7, showmeans=True, showextrema=True, showmedians=True, bw_method='silverman')
    axes[1].set_title('Accuracy (ATE in meters)', fontsize = 20)

    axes[2].violinplot(MEM_DATA, pos, points=60, widths=0.7, showmeans=True, showextrema=True, showmedians=True, bw_method='silverman')
    axes[2].set_title('Memory CPU (MB)', fontsize = 14 )



    for ax in axes.flatten():
        ax.set_xticks(pos)
        ax.set_xticklabels(LABELS)

        for label in ax.get_xmajorticklabels():
            label.set_rotation(30)
            label.set_horizontalalignment("right")
    
        
    if filename :
        fig.subplots_adjust(hspace=0.4)
        plt.savefig(filename, bbox_inches='tight')
    else :
        plt.show()

    return True

