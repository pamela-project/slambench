#!/usr/bin/env python2
from __future__ import print_function
import sys
import argparse
import math
import os
import numpy as np
import matplotlib.pyplot as plt
from tum_evaluate_tools import associate
from tum_evaluate_tools import evaluate_ate
from tf import transformations

def parse_input(filename, remove_repeat=True, input=None):
    sequences = {}
    info = {}
    inputs = {}
    if input is None:
        seq = 0 # valid seq will start from 1
    else:
        seq = 1
        sequences[1] = {}
        sequences[1]['traj'] = {}
        inputs[1] = input
    with open(filename) as fp:
        last_pose = None
        for line in fp:
            if line.startswith('#') or len(line.strip()) is 0:
                continue
            elif line.startswith('seq:'):
                try:
                    seq = int(line.lstrip('seq:'))
                    sequences[seq] = {}
                    sequences[seq]['traj'] = {}
                    inputs[seq] = info['scene'] + '-%s' % seq
                except ValueError:
                    exit('Invalid seq value:\n' + line)
            elif line.startswith('input:'):
                if input is not None: continue
                seq += 1
                sequences[seq] = {}
                sequences[seq]['traj'] = {}
                inputs[seq] = line.lstrip('input:').strip()
            elif line.startswith('reloc:'):
                aided_reloc = line.lstrip('aided_reloc:')
                sequences[seq]['aided_reloc'] = bool(aided_reloc) \
                    if 'false' not in aided_reloc and 'False' not in aided_reloc else False
            elif ':' in line:
                key = line.split(':')[0].strip()
                value = line.split(':')[1].strip()
                if key not in info: info[key] = value
            else:
                s = line.split()
                if len(s) != 8 and len(s) != 9:
                    exit('Invalid line:\n' + line)
                try:
                    stamp = float(s[0])
                    pose = [float(v) for v in s[-7:]]
                except ValueError:
                    exit('Invalid line:\n' + line)
                else:
                    if np.isnan(sum(pose)): continue
                    if remove_repeat and pose == last_pose: continue
                    sequences[seq]['traj'][stamp] = pose
                    last_pose = pose
    return info, sequences, inputs

def tf_matrix_to_values(matrix):
    t = list(matrix[:3, 3])
    q = list(transformations.quaternion_from_matrix(matrix))
    return t + q

def tf_values_to_matrix(values):
    angles = transformations.euler_from_quaternion(values[3:])
    return transformations.compose_matrix(angles=angles, translate=values[:3])

def transform_target_frame(sequences, scene, tf_target_new):
    tf_target_new = tf_values_to_matrix(tf_target_new)
    for seq in sequences:
        for t in sequences[seq]['traj'].keys():
            base_pose = np.dot(tf_values_to_matrix(sequences[seq]['traj'][t]), tf_target_new)
            sequences[seq]['traj'][t] = tf_matrix_to_values(base_pose)

def transform_world_frame(sequences, gts, auto_scale):
    seq = min(sequences.keys())
    traj0 = sequences[seq]['traj']
    gt0 = gts[seq]['traj']
    matches, ref_traj_interpolated = associate.associate_with_interpolation(gt0, traj0, 0, 0.5)
    if len(matches) < 1:
        sequences[seq]['ate'] = {}
        sequences[seq]['ate_rmse'] = float('nan')
        sequences[seq]['ate_num'] = 0
        sequences[seq]['oe'] = {}
        sequences[seq]['aoe_rmse'] = float('nan')
        return
    first_xyz = np.matrix([[float(value) for value in ref_traj_interpolated[a][0:3]] for a,b in matches]).transpose()
    second_xyz = np.matrix([[float(value) for value in traj0[b][0:3]] for a,b in matches]).transpose()
    if auto_scale:
        rot, trans, scale, ate = evaluate_ate.umeyama_align(second_xyz, first_xyz)
    else:
        rot, trans, ate = evaluate_ate.align(second_xyz, first_xyz)
        scale = 1
    sequences[seq]['ate'] = {m[1]: e for m, e in zip(matches, ate)}
    #print sequences[seq]['ate']
    sequences[seq]['ate_rmse'] = np.sqrt(np.dot(ate, ate) / len(ate))
    sequences[seq]['ate_num'] = len(ate)
    tf_matrix = evaluate_ate.compose_transform_matrix(rot, trans, scale)
    for seq in sequences:
        traj_trans = {t: tf_matrix_to_values(np.dot(tf_matrix, tf_values_to_matrix(p))) for t,p in sequences[seq]['traj'].items()}
        sequences[seq]['traj'] = traj_trans
    seq = min(sequences.keys())
    sequences[seq]['oe'] = {b: angle_diff_from_quaternions(ref_traj_interpolated[a][3:], sequences[seq]['traj'][b][3:]) for a,b in matches}
    aoe = np.abs(np.array(sequences[seq]['oe'].values()))
    sequences[seq]['aoe_rmse'] = np.sqrt(np.dot(aoe, aoe) / len(aoe)) if len(aoe) > 0 else float('nan')
    #print(sequences[seq]['oe'].values())

def calculate_ate(sequences, gts):
    for seq in sequences:
        if 'ate' in sequences[seq]: continue
        matches, ref_traj_interpolated = associate.associate_with_interpolation(gts[seq]['traj'], sequences[seq]['traj'], 0, 0.5)
        first_xyz = np.matrix([[float(value) for value in ref_traj_interpolated[a][0:3]] for a,b in matches]).transpose()
        second_xyz = np.matrix([[float(value) for value in sequences[seq]['traj'][b][0:3]] for a,b in matches]).transpose()
        error = second_xyz - first_xyz
        ate = np.sqrt(np.sum(np.multiply(error, error), 0)).A[0]
        sequences[seq]['ate'] = {m[1]: e for m, e in zip(matches, ate)}
        sequences[seq]['oe'] = {b: angle_diff_from_quaternions(ref_traj_interpolated[a][3:], sequences[seq]['traj'][b][3:]) for a,b in matches}
        aoe = np.abs(np.array(sequences[seq]['oe'].values()))
        sequences[seq]['ate_rmse'] = np.sqrt(np.dot(ate, ate) / len(ate)) if len(ate) > 0 else float('nan')
        sequences[seq]['aoe_rmse'] = np.sqrt(np.dot(aoe, aoe) / len(aoe)) if len(aoe) > 0 else float('nan')
        sequences[seq]['ate_num'] = len(ate)
        #print(sequences[seq]['oe'].values())

def calculate_correctness(sequences, gts, ate_threshold, aoe_threshold, max_pose_interval, reloc_score_factor):
    is_pose_correct = lambda ate, oe: ate <= ate_threshold and abs(oe) <= aoe_threshold
    for seq in sequences:
        ate = sequences[seq]['ate']
        oe = sequences[seq]['oe']
        c_ate = [ate[t] for t in ate if is_pose_correct(ate[t], oe[t])]
        c_oe = [oe[t] for t in oe if is_pose_correct(ate[t], oe[t])]
        sequences[seq]['c_ate_rmse'] = np.sqrt(np.dot(c_ate, c_ate) / len(c_ate)) if len(c_ate) > 0 else float('nan')
        sequences[seq]['c_aoe_rmse'] = np.sqrt(np.dot(c_oe, c_oe) / len(c_oe)) if len(c_oe) > 0 else float('nan')
        sequences[seq]['c_ate_num'] = len(c_ate)
        if len(ate) is 0:
            sequences[seq]['reloc_correct'] = False
            sequences[seq]['reloc_time'] = float('inf')
            sequences[seq]['reloc_score'] = 0
            sequences[seq]['track_time'] = 0
            sequences[seq]['track_cr'] = 0
            sequences[seq]['correct_time'] = 0
            sequences[seq]['cr'] = 0
            continue
        t0 = min(ate.keys())
        tmin = min(gts[seq]['traj'].keys())
        tmax = max(ate.keys() + gts[seq]['traj'].keys())
        gt = gts[seq]['traj']

        # re-localization metrics
        reloc_time = t0 - min(gt.keys())
        if reloc_time < 0: reloc_time = 0
        sequences[seq]['reloc_time'] = reloc_time
        sequences[seq]['reloc_correct'] = is_pose_correct(ate[t0], oe[t0])
        sequences[seq]['reloc_score'] = math.exp(-reloc_time / reloc_score_factor) if sequences[seq]['reloc_correct'] else 0

        # tracking metrics
        tracked = 0 # accumulated time of correct tracking
        stamps = sorted(ate.keys())
        for t in stamps:
            next_t = tmax if t == stamps[-1] \
                else stamps[stamps.index(t) + 1]
            if next_t <= t: break
            if is_pose_correct(ate[t], oe[t]):
                tracked += min(max_pose_interval, next_t - t)
        sequences[seq]['track_time'] = tmax - t0
        sequences[seq]['track_cr'] = tracked / (tmax - t0)
        sequences[seq]['correct_time'] = tracked
        sequences[seq]['cr'] = tracked / (tmax - tmin)

def angle_diff_from_quaternions(q1, q2):
    tf1 = transformations.compose_matrix(angles=transformations.euler_from_quaternion(q1))
    tf2 = transformations.compose_matrix(angles=transformations.euler_from_quaternion(q2))
    angle,_,_ = transformations.rotation_from_matrix(np.dot(np.linalg.inv(tf1), tf2))
    return angle / np.pi * 180.

def plot_all_traj(sequences, gts):
    fig = plt.figure()
    traj_num = max(sequences.keys())
    for seq in sequences:
        ax = fig.add_subplot(traj_num * 10 + 100 + seq)
        traj = sequences[seq]['traj']
        tt = sorted(traj.keys())
        tx = [traj[t][0] for t in tt]
        ty = [traj[t][1] for t in tt]
        ax.plot(tx, ty, '-r')
        ax.hold(True)
        if len(tx) > 0:
            ax.plot(tx[0], ty[0], 'or')
        traj = gts[seq]['traj']
        tt = sorted(traj.keys())
        tx = [traj[t][0] for t in tt]
        ty = [traj[t][1] for t in tt]
        ax.plot(tx, ty, '-g')
        ax.plot(tx[0], ty[0], 'og')
    plt.show()

def evaluate(sequences, inputs, gts, gt_info, ate_threshold, aoe_threshold, max_pose_interval, reloc_score_factor, auto_scale, print_results=True):
    seqs = sorted(sequences.keys())
    res = []
    print_str = lambda s: [print(s) if print_results else None, res.append(s)]
    column1 = '%-12s'
    print_str(column1 % 'inputs' + ' '.join(['%8s' % inputs[seq].rstrip('.slam') for seq in seqs]))
    print_str(column1 % 'poses' + ' '.join(['%8d' % len(sequences[seq]['traj']) for seq in seqs]))
    #transform_target_frame(gts, info['scene'], gt_info['frame'], info['frame'])
    transform_world_frame(sequences, gts, auto_scale)
    calculate_ate(sequences, gts)
    print_str(column1 % 'matches' + ' '.join(['%8d' % sequences[seq]['ate_num'] for seq in seqs]))
    print_str(column1 % 'ATE RMSE' + ' '.join(['%8.3f' % sequences[seq]['ate_rmse'] for seq in seqs]))
    calculate_correctness(sequences, gts, ate_threshold, aoe_threshold, max_pose_interval, reloc_score_factor)
    print_str(column1 % 'C-ATE RMSE' + ' '.join(['%8.3f' % sequences[seq]['c_ate_rmse'] for seq in seqs]))
    print_str(column1 % 'AOE RMSE' + ' '.join(['%8.3f' % sequences[seq]['aoe_rmse'] for seq in seqs]))
    print_str(column1 % 'C-AOE RMSE' + ' '.join(['%8.3f' % sequences[seq]['c_aoe_rmse'] for seq in seqs]))
    print_str(column1 % 'correct time' + ' '.join(['%8.3f' % sequences[seq]['correct_time'] for seq in seqs]))
    print_str(column1 % 'correct rate' + ' '.join(['%8.3f' % sequences[seq]['cr'] for seq in seqs]))
    #print_str(column1 % ('CR-T') + ' '.join(['%8.3f' % sequences[seq]['track_cr'] for seq in seqs]))
    #print_str(column1 % 'reloc time' + ' '.join(['%8.3f' % sequences[seq]['reloc_time'] for seq in seqs]))
    print_str(column1 % 'reloc score' + ' '.join(['%8.3f' % sequences[seq]['reloc_score'] for seq in seqs]))
    print_str('(ATE threshold: %.1f m. AOE threshold: %.1f deg. Reloc time factor: %.1f s.)' % (ate_threshold, aoe_threshold, reloc_score_factor))
    return '\n'.join(res)

def main():
    parser = argparse.ArgumentParser(usage='%(prog)s [options] RESULT_FILE.txt [RESULT_FILE2.txt ...]')
    parser.add_argument('-g', '--gt-path', help='path to ground-truth files, default is the same folder with results', type=str, default='')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-r', '--remove-repeated-pose', help='ignore repeated poses', dest='remove_repeat', action='store_true')
    group.add_argument('-k', '--keep-repeated-pose', help='keep repeated poses', dest='remove_repeat', action='store_false')
    parser.add_argument('-t', '--ate-threshold', type=float, help='ATE threshold of correctness (meter)', default=float('inf'))
    parser.add_argument('-o', '--aoe-threshold', type=float, help='AOE threshold of correctness (degree)', default=float('inf'))
    parser.add_argument('-m', '--max-pose-interval', help='consider lost after no pose for such time (sec)', type=float, default=1.)
    parser.add_argument('-f', '--reloc-score-factor', help='a factor to score time for re-localization (sec)', type=float, default=60.)
    parser.add_argument('-s', '--scale', help='find optimal scale', dest='scale', action='store_true')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('-p', '--plot', help='plot trajectories', dest='plot', action='store_true')
    group.add_argument('-np', '--no-plot', help='not plot trajectories', dest='plot', action='store_false')
    parser.set_defaults(remove_repeat=True)
    parser.set_defaults(scale=False)
    parser.set_defaults(plot=True)
    args, left = parser.parse_known_args()
    if len(left) < 1:
        parser.print_help(sys.stderr)
    for filename in left:
        info, sequences, inputs = parse_input(filename, args.remove_repeat)
        print('%s: %d trajectories' % (filename, len(sequences)))
        gt_path = args.gt_path if args.gt_path != '' else os.path.dirname(filename)
        gts = {}
        for seq, input in inputs.items():
            gt_file = gt_path + '/%s.gt' % input.rstrip('.slam')
            gt_info, gt_sequences, _ = parse_input(gt_file, input=input)
            gts[seq] = gt_sequences[1] # assume only one sequence in each gt file
        evaluate(sequences, inputs, gts, gt_info, args.ate_threshold, args.aoe_threshold, args.max_pose_interval, args.reloc_score_factor, args.scale)
        if args.plot:
            plot_all_traj(sequences, gts)

if __name__ == "__main__":
    main()
