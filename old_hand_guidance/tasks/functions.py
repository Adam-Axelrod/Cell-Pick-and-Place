import numpy as np
from numpy.linalg import inv

def wrench_transform(w, r, t):
    """
            Transform a wrench (force and torque) from one reference frame to another

            Args:
                w: The wrench vector [fx, fy, fz, tx, ty, tz] in source frame
                r: Euler angles [rx, ry, rz] in XYZ convention (in degrees)
                t: Translation vector (3x1) from source to target frame

            Returns:
                w_trans: The transformed wrench vector in target frame
            """
    # Convert Euler angles from degrees to radians
    r_rad = np.radians(r)

    # Convert Euler angles to rotation matrix
    R = euler_to_rotation_matrix(r_rad)
    R = inv(R)
    # Extract force and torque components
    force = w[:3]
    torque = w[3:]

    # Transform force
    force_trans = R @ force

    # Transform torque: t' = R*t + p×(R*f)
    # Where p is the translation vector and × is the cross product
    torque_trans = R @ torque + np.cross(t, force_trans)

    # Combine transformed force and torque
    w_trans = np.concatenate([force_trans, torque_trans])

    return w_trans


def euler_to_rotation_matrix(euler_angles):
    """
    Convert Euler angles XYZ to rotation matrix

    Args:
        euler_angles: [rx, ry, rz] in radians

    Returns:
        3x3 rotation matrix
    """
    rx, ry, rz = euler_angles

    # Rotation matrix around X axis
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx), np.cos(rx)]
    ])

    # Rotation matrix around Y axis
    Ry = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])

    # Rotation matrix around Z axis
    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz), np.cos(rz), 0],
        [0, 0, 1]
    ])

    # Combined rotation matrix (XYZ order)
    R = Rz @ Ry @ Rx

    return R


def twist_transform(twist, r, t):
    """
        Transform a twist (linear and angular velocity) from one reference frame to another

        Args:
            twist: The twist vector [vx, vy, vz, wx, wy, wz] in source frame
            r: Euler angles [rx, ry, rz] in XYZ convention (in degrees)
            t: Translation vector (3x1) from source to target frame

        Returns:
            twist_trans: The transformed twist vector in target frame
        """
    # Convert Euler angles from degrees to radians
    r_rad = np.radians(r)

    # Convert Euler angles to rotation matrix
    R = euler_to_rotation_matrix(r_rad)

    # Extract linear and angular velocity components
    linear_vel = twist[:3]
    angular_vel = np.radians(twist[3:])

    # Transform angular velocity
    angular_vel_trans = R @ angular_vel

    # Transform linear velocity: v' = R*v + ω'×(-R^T*t)
    # Where ω' is the transformed angular velocity and × is the cross product
    linear_vel_trans = R @ linear_vel + np.cross(angular_vel_trans, t)

    # Combine transformed linear and angular velocities
    twist_trans = np.concatenate([linear_vel_trans, np.degrees(angular_vel_trans)])

    return twist_trans


def limit(n, minn, maxn):
    """Clamp n to the inclusive range [minn, maxn].

    Replaces duplicated implementations across `tasks/` modules.
    """
    if n < minn:
        return minn
    elif n > maxn:
        return maxn
    else:
        return n
