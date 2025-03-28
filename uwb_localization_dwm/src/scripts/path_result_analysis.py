#!/usr/bin/env python3

"""
Path results analysis.
"""
from numpy import *
import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.art3d as art3d
from mpl_toolkits.mplot3d.axes3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import sys
import typing
import copy
import seaborn as sns
import pandas as pd
from kabsch_umeyama import kabsch_umeyama as umeyama
import matplotlib.patches as mpatches

if len(sys.argv) < 2 :
    bagname = '2024-10-13-17-13-33_8'
else:
    bagname = sys.argv[1]
bagpath = '/home/liu/workspace/ws/data/jackal_bag/rosbag_restore_xh/'

MatchingIndices = typing.Tuple[typing.List[int], typing.List[int]]
def matching_time_indices(stamps_1: np.ndarray, stamps_2: np.ndarray,
                          max_diff: float = 0.01,
                          offset_2: float = 0.0) -> MatchingIndices:
    """
    Searches for the best matching timestamps of two lists of timestamps
    and returns the list indices of the best matches.
    :param stamps_1: first vector of timestamps (numpy array)
    :param stamps_2: second vector of timestamps (numpy array)
    :param max_diff: max. allowed absolute time difference
    :param offset_2: optional time offset to be applied to stamps_2
    :return: 2 lists of the matching timestamp indices (stamps_1, stamps_2)
    """
    matching_indices_1 = []
    matching_indices_2 = []
    stamps_2 = copy.deepcopy(stamps_2)
    stamps_2 += offset_2
    for index_1, stamp_1 in enumerate(stamps_1):
        diffs = np.abs(stamps_2 - stamp_1)
        index_2 = int(np.argmin(diffs))
        if diffs[index_2] <= max_diff:
            matching_indices_1.append(index_1)
            matching_indices_2.append(index_2)
    return matching_indices_1, matching_indices_2


def getdata(data_path) -> [array]:
    use_data_cols=[0,1,2,3]
    path_data=pd.read_table(data_path, sep=' ',header=None,usecols=use_data_cols)

    return path_data

def path_align(ref_poses:array=None,est_poses:array=None, t_max_diff:float=0.01,
               t_offset:float=0.0, with_scale:bool=False, verbose:bool=False)->[array,array,array]:
    '''
    Find the matching poses in two paths with abs max time difference
    and time offset, and align them using Umeyama's method.
    '''
    stamps_1 = ref_poses[0]
    stamps_2 = est_poses[0]
    t_max_diff=0.1

    index_matches = matching_time_indices(stamps_1, stamps_2, max_diff= t_max_diff, offset_2= t_offset)

    # ref_poses_match = np.array([b for b in np.array(ref_poses.loc[index_matches[0],[1,2,3]])])
    ref_poses_match = np.array(ref_poses.loc[index_matches[0],[1,2,3]]) # pandas.core.frame.DataFrame

    R, t, c = umeyama(np.array(ref_poses.loc[index_matches[0],[1,2,3]]),
                      np.array(est_poses.loc[index_matches[1],[1,2,3]]), with_scale=with_scale)
    est_poses_aligned = np.array([t + c * R @ b for b in np.array(est_poses.loc[index_matches[1],[1,2,3]])])

    if verbose:
        print("R,t,c = :", end='\n')
        print(R)
        print(t)
        print('{}'.format(c,">20"), end='\n\n')

    return ref_poses_match, est_poses_aligned, index_matches

def color_map(data, cmap):
    """mapping value to color"""

    dmin, dmax = np.nanmin(data), np.nanmax(data)
    cmo = plt.cm.get_cmap(cmap)
    cs, k = list(), 256/cmo.N

    for i in range(cmo.N):
        c = cmo(i)
        for j in range(int(i*k), int((i+1)*k)):
            cs.append(c)
    cs = np.array(cs)
    data = np.uint8(255*(data-dmin)/(dmax-dmin))

    return cs[data]

def plot_path(ref:array=None, est:array=None, is_save:bool=False,savepath:str=None,method:str=None):
    plt.figure()
    plt.plot(est)
    plt.plot(ref)
    plt.xlabel('Position number')
    plt.ylabel('Value(m)')
    plt.legend(['x', 'y', 'z', 'ref_x', 'ref_y', 'ref_z'], loc='upper right')
    if method:
        plt.title(method+' positions')
    else:
        plt.title('Positions')
    if is_save:
        plt.savefig(savepath+'positions.png', transparent=True)
    plt.show()

    fig = plt.figure()
    ax = fig.add_axes(Axes3D(fig))
    # subplot_arg='122'
    # ax = fig.add_subplot(subplot_arg, projection="3d")
    step = 1
    alpha = 1
    linestyles = "solid"
    colors = 'b'
    xs = [[x_1, x_2]
        for x_1, x_2 in zip(est[:-1:step, 0], est[1::step, 0])]
    ys = [[x_1, x_2]
        for x_1, x_2 in zip(est[:-1:step, 1], est[1::step, 1])]
    zs = [[x_1, x_2]
        for x_1, x_2 in zip(est[:-1:step, 2], est[1::step, 2])]
    segs_3d = [list(zip(x, y, z)) for x, y, z in zip(xs, ys, zs)]
    # line_collection = art3d.Line3DCollection(segs_3d, colors=colors,
    #                                         alpha=alpha,
    #                                         linestyles=linestyles)

    line_collection = Line3DCollection(segs_3d, cmap=plt.get_cmap('jet'),alpha=alpha,linestyles=linestyles)
    [err,_,_,_] = calculate_error(ref, est)
    line_collection.set_array(err) # color the segments by our parameter
    curve = ax.add_collection(line_collection)
    colorbar = plt.colorbar(curve, ax=ax, cmap='jet', label='Error', shrink=0.7)

    ax.plot3D(ref[:,0],ref[:,1],ref[:,2], color='gray',label='ref')
    ax.set_xlabel('X(m)')
    ax.set_ylabel('Y(m)')
    ax.set_zlabel('Z(m)')
    ax.axis('equal')
    if method:
        ax.set_title(method+'3D line plot')
    else:
        ax.set_title('3D line plot')
    ax.legend(['est','ref'])
    if is_save:
        plt.savefig(savepath+'trajectory.png', transparent=True)
    plt.show()

def get_sse(ref:array=None, est:array=None)->float:
    '''
    Sum of square error
    '''
    if len(ref) == len(est):
        return np.sum([(x - y) ** 2 for x, y in zip(ref, est)])
    else:
        return None

def get_mse(ref:array=None, est:array=None)->float:
    '''
    Mean square error
    '''
    if len(ref) == len(est):
        # Note: if ref is an matrix, first compute diferent, next square of elements of each row, then sum of matrix
        return np.sum([(x - y) ** 2 for x, y in zip(ref, est)]) / len(ref)
    else:
        return None

def get_rmse(ref:array=None, est:array=None)->float:
    '''
    Root mean square error
    '''
    mse = get_mse(ref, est)
    if mse:
        return np.sqrt(mse)
    else:
        return None

def calculate_error(ref:ndarray=None, est:ndarray=None) -> [ndarray,ndarray,ndarray,ndarray]:
    err_x = np.array(est[:,0]-ref[:,0])
    err_y = np.array(est[:,1]-ref[:,1])
    err_z = np.array(est[:,2]-ref[:,2])

    # Distance error
    err = np.sqrt(err_x**2+err_y**2+err_z**2)
    # for i in len(err_x): err[i] = np.linalg.norm([err_x[i], err_y[i], err_z[i]],ord=2)

    return [err, err_x,err_y,err_z]

def plot_error(ref:array=None, est:array=None, is_save:bool=False,savepath:str=None,method:str=None):
    [err,err_x,err_y,err_z] = calculate_error(ref, est)

    # plot error in different axes
    plt.figure()
    plt.plot(err_x)
    plt.plot(err_y)
    plt.plot(err_z)
    plt.xlabel('Position number')
    plt.ylabel('Error(m)')
    plt.title('Localization performance')
    plt.legend(['err_x', 'err_y', 'err_z'], loc='upper right')
    if is_save:
        plt.savefig(savepath+'error.png', transparent=True)
    plt.show()

    # Show error in violinplot
    plt.figure() # dpi = 300
    # plt.rcParams["font.family"] = "SimSun" # For language
    # plt.rcParams["axes.unicode_minus"] = False
    label = ["err_x", "err_y", "err_z"]
    font = {"size": 10}
    sns.violinplot(data = [err_x, err_y, err_z])
    plt.xlabel("Axes", font)
    plt.ylabel("Error(m)", font)
    plt.xticks(ticks = [0, 1, 2], labels = label, fontsize = 10)
    plt.yticks(fontsize = 10)
    if method:
        plt.title(method)
    if is_save:
        plt.savefig(savepath+'errors_violin.png', transparent=True)
    plt.show()


    plt.figure()
    ax = sns.boxplot(data=[err_x,err_y,err_z], orient="v", palette="Set2") # orient="h"
    ax.set_title("Error")
    ax.set_ylabel('Error(m)')
    plt.xticks(ticks = [0, 1, 2], labels = label, fontsize = 10)
    if is_save:
        plt.savefig(savepath+'error_box.png', transparent=True)
    plt.show()

    # Show total error
    plt.figure()
    plt.plot(err,c='dimgray')
    # # Distance error's RMSE
    # rmse = get_rmse(ref, est) # sqrt(sum(err_x**2,err_y**2,err_z**2)/len(err_x))
    rmse = np.sqrt(np.sum(err**2) /len(err))
    err_rmse_l = plt.axhline(rmse, c='b')
    err.sort()
    mid = int(len(err) / 2)
    if len(err) % 2 == 0:
        median = (err[mid-1] + err[mid]) / 2.0
    else:
        median = err[mid]
    err_median_l = plt.axhline(median, c='g')
    mean = np.mean(err)
    err_mean_l = plt.axhline(mean, c='r')
    std = np.std(err)
    lower = mean-std
    upper = mean+std
    # plt.fill_between(lower, upper, color='gray', alpha=0.2)
    fill_x=[0,0,len(err),len(err)]
    fill_y=[upper,lower, lower,upper]
    std_area = plt.fill(fill_x, fill_y,color="cornflowerblue",alpha=0.3)
    plt.legend(['err', 'rmse','median', 'mean','std'], loc='upper right')
    plt.xlim(0,len(err))
    plt.xlabel('Position number')
    plt.ylabel('Error(m)')
    plt.title('Localization performance')
    if is_save:
        plt.savefig(savepath+'error_violin.png', transparent=True)
    plt.show()

    print('Dataset:{}'.format(savepath,">30"), end='\n')
    print('median:{}'.format(median,">20"), end='\n')
    print('mean: {}'.format(mean,">20"), end='\n')
    print('max: {}'.format(np.max(err),">20"), end='\n')
    print('min: {}'.format(np.min(err),">20"), end='\n')
    print('rmse: {}'.format(rmse,">20"), end='\n')
    print('sse: {}'.format(np.sum(err**2),">20"), end='\n') # get_sse(ref,est)
    print('std: {}'.format(std,">20"), end='\n')

def plot_result(ref:ndarray=None, est:ndarray=None, methods:ndarray=None, is_save:bool=False,savepaths:str=None):
    # sns.set_style("whitegrid")
    save_result={}
    save_error={}
    error={}
    # Plot error of each methods seperately
    for i in range(1,len(methods)):
        save_result[methods[i]] = savepaths+methods[i]
        plot_path(ref[methods[i]],est[methods[i]],is_save=is_save,savepath=save_result[methods[i]],
                  method=methods[i])

        save_error[methods[i]] = savepaths+methods[i]
        plot_error(ref[methods[i]],est[methods[i]],is_save=is_save, savepath=save_error[methods[i]],
                   method=methods[i])

        error[methods[i]]=calculate_error(ref[methods[i]],est[methods[i]])

    # Plot error in violin of methods of different axes
    label = ["err_x", "err_y", "err_z"]
    colors =['r','g','b']
    c_palette=sns.color_palette() #"hls",len(label)
    fig,axes=plt.subplots(1,len(methods)-1,sharex=True, sharey=True)
    for i in range(1,len(methods)):
        ax=sns.violinplot(error[methods[i]][1:4],ax=axes[i-1], palette=c_palette,
                          scale="count", frameon=False) # , inner="quartile",scale_hue=False
        if 1 == i:
            ax.set_ylabel("Error(m)")
            # ax.legend(label)
            plt.yticks(fontsize = 10)
            # ax.set_yticks(ticks = [-0.5,0,0.5],labels=[-0.5,0,0.5],fontsize = 10)
            # plt.yticks(ticks = [-0.5,0,0.5],fontsize = 10)
        if 1<i:
            # ax.set_xticks([])
            # ax.set_yticks([])
            # plt.yticks([])
            ax.spines['left'].set_visible(False)
            ax.get_yaxis().set_visible(False)
            # plt.gca().axes.get_yaxis().set_visible(False)
        ax.set_xticks(ticks = [0, 1, 2], labels = label, fontsize = 10)
        # ax.set_xticks(ticks = [0, 1, 2], labels = [], fontsize = 10)
        ax.set_xlabel(methods[i])
        ax.spines['right'].set_visible(False)
        # ax.spines['top'].set_visible(False)
        # ax.legend(label,loc = 'upper right', frameon=False,ncol=3)
        if len(methods)-1==i:
            ax.spines['right'].set_visible(True)
            # ax.legend(labels=label, loc = 'upper right', frameon=False)
            # label_s = ax.get_legend_handles_labels()
            palette=sns.color_palette() # Default color of seaborn
            patches = [mpatches.Patch(color=palette[i], label=label[i]) for i in range(len(label))]
            ax.legend(handles=patches,labels=label, loc = 'upper right', frameon=False)
        # fig.tight_layout()
        plt.subplots_adjust(wspace =0, hspace =0)


    if is_save:
        plt.savefig(savepaths+'error_violin_methods.png', transparent=True)
    plt.show()

    # Plot error in box of methods
    fig,axes=plt.subplots(1,len(methods)-1,sharex=True, sharey=True)
    for i in range(1,len(methods)):
        ax=sns.boxplot(data=error[methods[i]][1:4],ax=axes[i-1], orient="v",showfliers=False)#, palette="Set2"
        if 1 == i:
            ax.set_ylabel("Error(m)")
            plt.yticks(fontsize = 10)
        if 1<i:
            ax.spines['left'].set_visible(False)
            ax.get_yaxis().set_visible(False)
        ax.set_xticks(ticks = [0, 1, 2], labels = label, fontsize = 10)
        ax.set_xlabel(methods[i])
        if len(methods)-1==i:
            ax.spines['right'].set_visible(True)
            palette=sns.color_palette()
            patches = [mpatches.Patch(color=palette[i], label=label[i]) for i in range(len(label))]
            ax.legend(handles=patches,labels=label, loc = 'upper right', frameon=False)
        # fig.tight_layout()
        plt.subplots_adjust(wspace =0, hspace =0)
        ax.set_xlabel(methods[i])
        # ax.set_title("Error")

    if is_save:
        plt.savefig(savepaths+'error_box_methods.png', transparent=True)
    plt.show()

    is_save=False
    # save data to compare with diferent date's data; Or better to save paths
    if is_save:
        err_path=savepaths+'error_methods_e_x_y_z'+'.npy' # dictionary {methods[i]:array[e, e_x, e_y, e_z]}
        np.save(err_path, dict)

def main():
    result_path = bagpath +'uwb_path_results/'

    methods = ['aft_mapped_path','saved_dis_path_ls','saved_dis_path_tsvd','saved_dis_path_ftr','saved_dis_path_hr'] # Using

    ref_path = result_path+bagname+'_'+methods[0]+'.txt'
    ref_poses = getdata(ref_path)
    print('Processing data: ', bagname)

    est_poses ={}
    ref_poses_match={}
    est_poses_aligned={}
    for i in range(1,len(methods)):
        est_path = result_path+bagname+'_'+methods[i]+'.txt'
        est_poses[methods[i]] = getdata(est_path)

        with_scale=False
        if methods[i] == 'pose_graph_path' or methods[i] == 'path':
            with_scale = True
        [ref_poses_match[methods[i]], est_poses_aligned[methods[i]], _]= \
            path_align(ref_poses=ref_poses,est_poses=est_poses[methods[i]], t_max_diff=0.1,
                t_offset=0.0, with_scale=with_scale,verbose=True)

    save_results= result_path+bagname+'_'
    plot_result(ref_poses_match,est_poses_aligned, methods=methods,is_save=False, savepaths=save_results)

if __name__ == '__main__':
    main()

