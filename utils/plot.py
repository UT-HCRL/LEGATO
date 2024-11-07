import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

# Define colors for each axis
AXIS_COLORS = ['r', 'g', 'b']  # Red for x, Green for y, Blue for z


def plot_figs(plt, save=False, save_path=None, **kwargs):

    if save:
        if save_path is None:
            save_path = './out.png'
        plt.savefig(save_path)
    else:
        plt.show()


def DHB(linear_motion_invariants, angular_motion_invariants, **kwargs):
    # Plot 4D invariants

    fig, axs = plt.subplots(2,3)

    axs[0,0].plot(linear_motion_invariants[:,0])
    axs[0,0].set_ylabel('$m_p$')
    axs[0,0].set_xlabel('Time [sec]')

    axs[0,1].plot(linear_motion_invariants[:,1])
    axs[0,1].set_ylabel('${\\theta}_p^1$')
    axs[0,1].set_xlabel('Time [sec]')

    axs[0,2].plot(linear_motion_invariants[:,2])
    axs[0,2].set_ylabel('${\\theta}_p^2$')
    axs[0,2].set_xlabel('Time [sec]')

    axs[1,0].plot(angular_motion_invariants[:,0])
    axs[1,0].set_ylabel('$m_{\omega}$')
    axs[1,0].set_xlabel('Time [sec]')

    axs[1,1].plot(angular_motion_invariants[:,1])
    axs[1,1].set_ylabel('${\\theta}_{\omega}^1$')
    axs[1,1].set_xlabel('Time [sec]')

    axs[1,2].plot(angular_motion_invariants[:,2])
    axs[1,2].set_ylabel('${\\theta}_{\omega}^2$')
    axs[1,2].set_xlabel('Time [sec]')

    plot_figs(plt=plt, **kwargs)


def SE3(positions, quaternions, **kwargs):

    assert positions.shape[0] == quaternions.shape[0]
    num_points = positions.shape[0]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot position
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='Trajectory')

    # Optionally plot orientation quaternions
    for i in range(0, num_points, 10):
        # Create a basis in the orientation of the quaternion
        rot = R.from_quat(quaternions[i])
        basis = rot.apply(np.eye(3))
        # Plot each basis vector
        for j in range(3):
            start_point = positions[i]
            end_point = positions[i] + 0.1 * basis[j]
            ax.plot([start_point[0], end_point[0]], 
                    [start_point[1], end_point[1]], 
                    [start_point[2], end_point[2]], color=AXIS_COLORS[j])

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()

    plot_figs(plt=plt, **kwargs)


def SE3s(trajectories, show_quaternions=False, show_legend=True, **kwargs):

    if not trajectories.__class__ == dict:
        trajectories = {'demo_{}'.format(i): trajectories[i] for i in range(len(trajectories))}

    assert len(trajectories) > 0

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    trajecory_colors = ['c', 'm', 'y', 'k']
    idx_trajectories = 1

    # Plot position
    for key, trajectory in trajectories.items():
        positions = trajectory['positions']
        quaternions = trajectory['quaternions']

        assert positions.shape[0] == quaternions.shape[0]
        num_points = positions.shape[0]

        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                label=key, 
                color=trajecory_colors[idx_trajectories%4])

        if show_quaternions:
            # Optionally plot orientation quaternions
            for i in range(0, num_points, 10):
                # Create a basis in the orientation of the quaternion
                rot = R.from_quat(quaternions[i])
                basis = rot.apply(np.eye(3))
                
                # Plot each basis vector
                for j in range(3):
                    start_point = positions[i]
                    end_point = positions[i] + 0.1 * basis[j]
                    ax.plot([start_point[0], end_point[0]], 
                            [start_point[1], end_point[1]], 
                            [start_point[2], end_point[2]], color=AXIS_COLORS[j])

        idx_trajectories += 1

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    if show_legend:
        ax.legend()

    plot_figs(plt=plt, **kwargs)


def DHBs(trajectories, show_legend=True, mode='stastics', **kwargs):

    if not trajectories.__class__ == dict:
        trajectories = {'demo_{}'.format(i): trajectories[i] for i in range(len(trajectories))}

    assert len(trajectories) > 0

    fig, axs = plt.subplots(2,3)
    trajecory_colors = ['c', 'm', 'y', 'k']
    idx_trajectories = 1


    if mode == 'stastics':
        # find the max length of trajectories
        max_length = 0
        for key, trajectory in trajectories.items():
            linear_motion_invariants = trajectory['linear_motion_invariants']
            angular_motion_invariants = trajectory['angular_motion_invariants']
            assert linear_motion_invariants.shape[0] == angular_motion_invariants.shape[0]
            max_length = max(max_length, linear_motion_invariants.shape[0])

        linear_motion_invariants = [[] for _ in range(max_length)]
        angular_motion_invariants = [[] for _ in range(max_length)]
        # compute mean and std of invariants
        for idx in range(max_length):
            for trajectory in trajectories.values():
                if idx < trajectory['linear_motion_invariants'].shape[0]:
                    # print(idx, trajectory['linear_motion_invariants'].shape)
                    linear_motion_invariants[idx].append(trajectory['linear_motion_invariants'][idx])
                    angular_motion_invariants[idx].append(trajectory['angular_motion_invariants'][idx])
        linear_motion_invariants_mean = np.array([np.mean(np.array(linear_motion_invariants[idx]), axis=0) for idx in range(max_length)])
        linear_motion_invariants_std = np.array([np.std(np.array(linear_motion_invariants[idx]), axis=0) for idx in range(max_length)])
        angular_motion_invariants_mean = np.array([np.mean(np.array(angular_motion_invariants[idx]), axis=0) for idx in range(max_length)])
        angular_motion_invariants_std = np.array([np.std(np.array(angular_motion_invariants[idx]), axis=0) for idx in range(max_length)])

        # plot mean and std of invariants
        for i in range(3):
            axs[0,i].plot(
                linear_motion_invariants_mean[:,i], 
                label=key, 
                color=trajecory_colors[idx_trajectories%4]
                )
            axs[0,i].fill_between(
                np.arange(max_length), 
                linear_motion_invariants_mean[:,i] - linear_motion_invariants_std[:,i], 
                linear_motion_invariants_mean[:,i] + linear_motion_invariants_std[:,i], 
                color=trajecory_colors[idx_trajectories%4], 
                alpha=0.1
                )
            axs[1,i].plot(
                angular_motion_invariants_mean[:,i], 
                label=key, 
                color=trajecory_colors[idx_trajectories%4]
                )
            axs[1,i].fill_between(
                np.arange(max_length), 
                angular_motion_invariants_mean[:,i] - angular_motion_invariants_std[:,i], 
                angular_motion_invariants_mean[:,i] + angular_motion_invariants_std[:,i], 
                color=trajecory_colors[idx_trajectories%4], 
                alpha=0.1
                )

    else:
        for key, trajectory in trajectories.items():

            linear_motion_invariants = trajectory['linear_motion_invariants']
            angular_motion_invariants = trajectory['angular_motion_invariants']

            assert linear_motion_invariants.shape[0] == angular_motion_invariants.shape[0]

            for i in range(3):
                axs[0,i].plot(
                    linear_motion_invariants[:,i], 
                    label=key, 
                    color=trajecory_colors[idx_trajectories%4]
                    )
                axs[1,i].plot(
                    angular_motion_invariants[:,i], 
                    label=key, 
                    color=trajecory_colors[idx_trajectories%4]
                    )
            idx_trajectories += 1

    axs[0,0].set_ylabel('$m_p$')
    axs[0,1].set_ylabel('${\\theta}_p^1$')
    axs[0,2].set_ylabel('${\\theta}_p^2$')
    axs[1,0].set_ylabel('$m_{\omega}$')
    axs[1,1].set_ylabel('${\\theta}_{\omega}^1$')
    axs[1,2].set_ylabel('${\\theta}_{\omega}^2$')

    for i in range(3):
        for j in range(2):
            axs[j,i].set_xlabel('Time [sec]')
            if show_legend:
                axs[j,i].legend()

    plot_figs(plt=plt, **kwargs)
