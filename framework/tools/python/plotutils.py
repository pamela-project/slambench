#!/usr/bin/python2


# DO not use DISPLAY within matplotlib
#import matplotlib as mpl
#mpl.use('Agg')
import matplotlib.pyplot as plt
# End of BugFix

import matplotlib
import pylab as plot

from matplotlib.legend_handler import HandlerLine2D

from slamlog import *

import operator


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
# UTILS COLORS
#########################################################################################


ccycle = [ (101, 153, 255),   (0,0,0),  (100,100,100), (150,100,150),(150,150,150),  (192, 192, 192), (255,0,0), (255, 153, 0), (199,233,180), (9, 112, 84) ,
          
          (170,163, 57),(255,251,188),(230,224,123),(110,104, 14),( 49, 46,  0),
          (138,162, 54),(234,248,183),(197,220,118),( 84,105, 14),( 37, 47,  0),
          
          (122, 41,106),(213,157,202),(165, 88,150),( 79, 10, 66),( 35,  0, 29),

          (65,182,196),(34,94,168),(12,44,132),
          ( 79, 44,115),(181,156,207),(122, 89,156),( 44, 15, 74),( 18,  2, 33)
          ]



def get_color_cycle (total_step) :
    return ccycle


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

   
#########################################################################################
# Filters
#########################################################################################



def select_all (data_array,i) :
    return True


def select_threshold(data_array,i) :
    ate_value = float( data_array["AbsoluteError"][i]) 
    return ate_value < accuracy_threshold

def select_twicethreshold(data_array,i) :
    ate_value = float( data_array["max_ATE"][i]) 
    return ate_value < 1.25*accuracy_threshold

#########################################################################################
# Naming
#########################################################################################




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




def compute_pareto_lines(data_array,x_select,x_operator,y_select ,y_operator,  filtered_list) :

    data_size  =  len(data_array[next(iter(data_array.keys()))])

    
    paretolines = []
    
    To_test = filtered_list[:]
    
    for i in filtered_list :
        xval = float(data_array[x_select][i])
        yval = float(data_array[y_select][i])
        strong = True
        for j in To_test :
            xvalbis = float(data_array[x_select][j])
            yvalbis = float(data_array[y_select][j])
            if x_operator (xvalbis , xval) and y_operator (yvalbis , yval) :
                strong = False
                break
        if strong :
            paretolines.append(i)
        else :
            To_test.remove(i)
    return paretolines

def find_nearest_below(my_array, target):
    if not isinstance(target,float) and target in my_array : 
        return target
    
    diff = np.array(my_array) - target
    mask = np.ma.greater(diff, 0)

    if np.all(mask):
        return 0
    masked_diff = np.ma.masked_array(diff, mask)
    return my_array[masked_diff.argmax()]
    
def plot_data (data_array, coloring = "version", 
    xelem = "total_duration",
    yelem = "max_ATE",
    ylabel= "Computation duration (sec)",
    xlabel= "Accuracy (m)",
    xoperator = operator.lt,
    yoperator = operator.lt,
    xlog = False,
    ylog = False,
    filename = "paper.png",
    print_legend = True, 
    ylim=None,
    filter_function = select_all
    
    ) : 

    maximum_segmentation = 6

    data_size  =  len(data_array[next(iter(data_array.keys()))])
    
    sys.stderr.write("Preparing %s out of %d points ...\n" % (filename, data_size))        
    handler_map_for_legend = {}
    
    
    segmentation = sorted(set(data_array[coloring]))
    
    if len(segmentation) > maximum_segmentation :
        new_segmentation = []
        for i in range (0,len(segmentation),int(len(segmentation)/maximum_segmentation)) :
            new_segmentation.append(sorted(segmentation)[i])
        segmentation = new_segmentation


    xval12  = {}
    yval1   = {}
    paretoX = {}
    paretoY = {}
    segmentation_groups = {}
    
    for e in segmentation :
        xval12[e] = []
        yval1[e] = []
        paretoX[e] = []
        paretoY[e] = []
        segmentation_groups[e] = []
        
    for i in range(data_size) :
        c = find_nearest_below(segmentation,data_array[coloring][i])
        if filter_function (data_array,i) :
            xval12[c].append ( data_array[xelem][i] )
            yval1[c].append  ( data_array[yelem][i] )
            segmentation_groups[c].append(i)

    pareto_list = {}
    for e in segmentation :
        pareto_list[e] = compute_pareto_lines(data_array,xelem,xoperator,yelem,yoperator,segmentation_groups[e])
        sys.stderr.write("Segment %s : %d pareto points.\n" % (e,len(pareto_list[e])))        
        sys.stderr.write("Segment %s : %s\n" % (e,pareto_list[e]))        
    for e in segmentation :
        for i in pareto_list[e] :
            xval =  float(data_array[xelem][i])
            yval =  float(data_array[yelem][i])
            paretoX[e].append(xval)
            paretoY[e].append(yval)
            
        if len(paretoY[e]) > 0 :
           # paretoX[e].append(1.0)
           # paretoY[e].append(min(paretoY[e]))
            paretoX[e],paretoY[e]  = zip(*sorted(zip((paretoX[e]),paretoY[e]), key=lambda item: item[0]))
    
    # START FIG ACCURACY MATEX ATE / SPEED 
    
    fig = plt.figure()

    ccycle = [ (float(a)/255,float(b)/255,float(c)/255)   for (a,b,c) in get_color_cycle(len(segmentation))]

    
    # FIRST PLOT
    
    
    ax1 = plt.subplot(1, 1, 1)
    
    if xelem == "total_duration" :
        plt.xlim((0,0.5))
        
    if xlog :
        ax1.set_xscale('log')
    if ylog :
        ax1.set_yscale('log')
    
    for seg in (segmentation) :
        plt.plot(xval12[seg],yval1[seg], linestyle='None', marker='o', mew=0.5,markersize=3,fillstyle="none")

    # basepoint, = plt.plot([baseline[xelem]],[baseline[yelem]], linestyle='None', color='black', marker='x',markersize=10, mew=3.5,fillstyle="none",label="Initial configuration")
    # handler_map_for_legend[basepoint] = HandlerLine2D(numpoints=1)       

    for i in range(len(segmentation)) :
        seg = segmentation[i]
        paretoline, = plt.plot(paretoX[seg],paretoY[seg] , color=ccycle[i],label=str(seg).replace("../", "").replace("/data", "").replace(".data",""), linewidth = 2)
        handler_map_for_legend[paretoline] = HandlerLine2D(numpoints=1)
        
    plt.ylabel(yelem)
    plt.xlabel(xelem)
    
    if ylim != None :
        plt.ylim(ylim)
     
    [_, xmax, _, _] = plt.axis()
    
    if (yelem == "max_ATE") :
        ax1.text(xmax, accuracy_threshold,"Accuracy limit = %2.2fcm" % (accuracy_threshold), horizontalalignment='right',      verticalalignment='bottom')
        ax1.arrow(0, accuracy_threshold, xmax, 0, head_width=0, head_length=0 , width=0.0001)
    
    
    # Now add the legend with some customizations.
    if print_legend :
        lgd = ax1.legend(handler_map=handler_map_for_legend,loc='center left', bbox_to_anchor=(1, 0.5) ,  fancybox=True, shadow=True, ncol=1)
        fig.savefig(filename, dpi=120,  bbox_extra_artists=(lgd,), bbox_inches='tight')
    else :

        fig.savefig(filename, dpi=120, bbox_inches='tight')
    

######################################################################################
######################################################  VIOLINS
######################################################################################

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

