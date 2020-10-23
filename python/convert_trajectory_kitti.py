import quaternion
import pathlib
import argparse
import numpy as np
import math

def files_in_path(directory):
    """TODO: Docstring for files_in_path.

    :function: TODO
    :returns: TODO

    """
    directory_path = pathlib.Path(directory)
    assert(directory_path.is_dir())
    file_list = []
    for x in sorted(directory_path.iterdir()):
        if x.is_file():
            file_list.append(x)
        elif x.is_dir():
            file_list.extend(files_in_path(x))
    return file_list

def load_file_list(file_list):
    """Loads the files into a list of numpy arrays

    :function: TODO
    :returns: TODO

    """
    trajectory_list = []
    for filename in file_list:
        trajectory_list.append(np.genfromtxt(filename, delimiter="\t", autostrip=True, usecols=(0, 1, 2, 3), max_rows=4))

    return trajectory_list

def load_trajectory_file(poses_path):
    poses_path = pathlib.Path(poses_path)
    if not poses_path.exists():
        assert False, f"Camera trajectory file does not exist at: {str(poses_path)}"

    trajectory = np.loadtxt(poses_path)
    trajectory = np.vsplit(trajectory, trajectory.shape[0]/trajectory.shape[1])

    return trajectory

def parse_rgbd_dataset(args):
    """TODO: Docstring for parse_rgbd_dataset.

    :function: TODO
    :returns: TODO

    """
    trajectory_list = load_trajectory_file(args.poses_path)
    gt_file_list = files_in_path(args.gt_poses_path)
    gt_trajectory_list = load_file_list(gt_file_list)

    num_poses = len(gt_trajectory_list);
    print(f"Number of poses in Ground truth trajectory: {num_poses}")

    gt_poses_list, poses_list = [], []
    for idx, (gt_pose, pose) in enumerate(zip(gt_trajectory_list, trajectory_list)):
        gt_pose = gt_pose[0:3, 0:4]
        pose = pose[0:3, 0:4]
        gt_poses_list.append(gt_pose.flatten())
        poses_list.append(pose.flatten())

    gt_poses_list = np.vstack(gt_poses_list)
    poses_list = np.vstack(poses_list)
    print(f"Number of associated files: {gt_poses_list.shape[0]}")
    np.savetxt("gt_poses.txt", gt_poses_list)
    np.savetxt("poses.txt", poses_list)

def associate(first_list, second_list, offset,max_difference):
    """
    Associate two dictionaries of (stamp,data). As the time stamps never match exactly, we aim
    to find the closest match for every input tuple.

    Input:
    first_list -- first dictionary of (stamp,data) tuples
    second_list -- second dictionary of (stamp,data) tuples
    offset -- time offset between both dictionaries (e.g., to model the delay between the sensors)
    max_difference -- search radius for candidate generation
    Output:
    matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))

    """
    first_keys = list(first_list)
    second_keys = list(second_list)
    potential_matches = [(abs(a - (b + offset)), a, b)
                         for a in first_keys
                         for b in second_keys
                         if abs(a - (b + offset)) < max_difference]
    potential_matches.sort()
    matches = []
    for diff, a, b in potential_matches:
        if a in first_keys and b in second_keys:
            idx1 = first_keys.index(a)
            idx2 = second_keys.index(b)
            matches.append((idx1, idx2))

    matches.sort()
    return matches

def read_file_list(filename):
    """
    Reads a trajectory from a text file.

    File format:
    The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp (to be matched)
    and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) associated to this timestamp.

    Input:
    filename -- File name

    Output:
    dict -- dictionary of (stamp,data) tuples

    """
    file = open(filename)
    data = file.read()
    lines = data.replace(","," ").replace("\t"," ").split("\n")
    list1 = [[v.strip() for v in line.split(" ") if v.strip()!=""] for line in lines if len(line) > 0 and line[0] != "#"]

    list1 = [(float(l[0]),l[1:]) for l in list1 if len(l)>1]
    return dict(list1)

def parse_tum_dataset(args):
    """TODO: Docstring for parse_tum_dataset.

    :function: TODO
    :returns: TODO

    """
    gt_file = args.gt_pose_file
    association_file = args.association_file
    gt_list = read_file_list(gt_file)
    files_list = read_file_list(association_file)

    num_poses = len(gt_list);
    print(f"Number of poses in Ground truth trajectory: {num_poses}")
    matches = associate(gt_list, files_list, 0.0, 0.02)
    print(len(matches))
    trajectory_list = load_trajectory_file(args.poses_path)
    print(len(trajectory_list))

    gt_list = list(gt_list.values())
    gt_poses_list, poses_list = [], []
    for idx1, idx2 in matches:
        gt_pose = gt_list[idx1]
        gt_pose = np.array(gt_pose).astype(np.float)
        t = np.expand_dims(gt_pose[0:3], 1)
        q = quaternion.as_quat_array(gt_pose[3:])
        R = quaternion.as_rotation_matrix(q)
        gt_pose = np.hstack((R, t))
        pose = np.asarray(trajectory_list[idx2])[0:3, 0:4]
        gt_poses_list.append(gt_pose.flatten())
        poses_list.append(pose.flatten())

    gt_poses_list = np.vstack(gt_poses_list)
    poses_list = np.vstack(poses_list)
    print(gt_poses_list.shape)
    print(poses_list.shape)
    np.savetxt("gt_poses.txt", gt_poses_list)
    np.savetxt("poses.txt", poses_list)



if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Calculate absolute trajectory error for dataset")

    parser.add_argument("poses_path", help="Path to the poses generated from object-slam")
    subparsers = parser.add_subparsers(title="Dataset type", help="Subparsers for dataset")

    parser_rgbd = subparsers.add_parser("rgbd-scenes", help="Arguments for RGBD scenes dataset type")
    parser_rgbd.add_argument("gt_poses_path", help="Path to ground truth poses of the dataset")
    parser_rgbd.set_defaults(func=parse_rgbd_dataset)

    parser_tum = subparsers.add_parser("tum", help="Arguments for TUM dataset type")
    parser_tum.add_argument("gt_pose_file", help="Path to the ground truth poses txt file")
    parser_tum.add_argument("association_file", help="Path to the synchronized rgbd file list")
    parser_tum.set_defaults(func=parse_tum_dataset)

    args = parser.parse_args()
    args.func(args)

