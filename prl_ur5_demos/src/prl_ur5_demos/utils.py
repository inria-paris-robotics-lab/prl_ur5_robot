import os
from copy import deepcopy

import rospkg
import rospy
import tf
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import (Point, Pose, PoseStamped, Quaternion, Vector3,
                               Vector3Stamped)
from moveit_msgs.msg import Grasp, GripperTranslation, PlaceLocation
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

__all__ = ('make_pose', 'shift_pose', 'make_grasp', 'make_location',
           'full_path', 'gazebo_spawn_urdf_model', 'gazebo_delete_model')


def make_pose(position, orientation, frame_id=None):
    """Initialize the geometry_msgs/Pose(Stamped) message.

    Arguments:
        position {list} -- position values xyz
        orientation {list} -- quaternion values xyzw or euler angles rpy

    Keyword Arguments:
        frame_id {str} -- id of the frame in which the pose is defined (default: {None})

    Returns:
        Pose | PoseStamped -- message
    """
    if len(orientation) == 3:
        orientation = tf.transformations.quaternion_from_euler(*orientation)
    pose = Pose(
        position=Point(*position),
        orientation=Quaternion(*orientation))
    if frame_id is not None:
        return PoseStamped(header=Header(frame_id=frame_id), pose=pose)
    return pose


def shift_pose(pose, dx=0.0, dy=0.0, dz=0.0):
    """Translate pose.

    Arguments:
        pose {Pose | PoseStamped} -- initial pose

    Keyword Arguments:
        dx {float} -- x step (default: {0.0})
        dy {float} -- y step (default: {0.0})
        dz {float} -- z step (default: {0.0})

    Returns:
        Pose | PoseStamped -- message
    """
    pose = deepcopy(pose)
    if isinstance(pose, PoseStamped):
        position = pose.pose.position
    else:
        position = pose.position
    position.x += dx
    position.y += dy
    position.z += dz
    return pose


def make_grasp(arm, gripper, eef_grasp_pose, approach_distance=0.2, lift_distance=0.2):
    """Initialize the moveit_msgs/Grasp message.

    Arguments:
        arm {MoveGroupCommander} -- arm group
        gripper {MoveGroupCommander} -- gripper group
        eef_grasp_pose {PoseStamped} -- pose of the end effector in which it should attempt grasping

    Keyword Arguments:
        approach_distance {float} -- distance from which to approach the object (default: {0.2})
        lift_distance {float} -- distance to take after a grasp has been completed (default: {0.2})

    Returns:
        Grasp -- message
    """
    return Grasp(
        id='grasp',
        grasp_quality=1.0,
        grasp_pose=eef_grasp_pose,
        pre_grasp_posture=_make_posture(
            gripper.get_named_target_values('open')),
        grasp_posture=_make_posture(
            gripper.get_named_target_values('close')),
        pre_grasp_approach=_make_gripper_translation(
            arm.get_end_effector_link(), [0, 0, 1], approach_distance),
        post_grasp_retreat=_make_gripper_translation(
            arm.get_planning_frame(), [0, 0, 1], lift_distance),
        post_place_retreat=_make_gripper_translation(
            arm.get_end_effector_link(), [0, 0, -1], approach_distance),
        max_contact_force=10.0,
        allowed_touch_objects=[])


def make_location(arm, gripper, place_pose, descent_distance=0.2, retreat_distance=0.2):
    """Initialize the moveit_msgs/Grasp message.

    Arguments:
        arm {MoveGroupCommander} -- arm group
        gripper {MoveGroupCommander} -- gripper group
        place_pose {PoseStamped} -- target object pose

    Keyword Arguments:
        descent_distance {float} -- descent distance to take before placing an object (default: {0.2})
        retreat_distance {float} -- retreat distance to take after a place has been completed (default: {0.2})

    Returns:
        PlaceLocation -- message
    """
    return PlaceLocation(
        id='grasp',
        post_place_posture=_make_posture(
            gripper.get_named_target_values('open')),
        place_pose=place_pose,
        pre_place_approach=_make_gripper_translation(
            arm.get_planning_frame(), [0, 0, -1], descent_distance),
        post_place_retreat=_make_gripper_translation(
            arm.get_end_effector_link(), [0, 0, -1], retreat_distance),
        allowed_touch_objects=[])


def _make_vector_stamped(frame_id, vector):
    return Vector3Stamped(
        header=Header(frame_id=frame_id),
        vector=Vector3(*vector))


def _make_gripper_translation(frame_id, direction, desired_distance, min_distance=None):
    return GripperTranslation(
        direction=_make_vector_stamped(frame_id, direction),
        desired_distance=desired_distance,
        min_distance=min_distance or desired_distance * 0.6)


def _make_posture(named_joint_values, time_from_start=rospy.Duration(5.0)):
    return JointTrajectory(
        joint_names=[name for name, _ in named_joint_values.items()],
        points=[JointTrajectoryPoint(
            positions=[val for _, val in named_joint_values.items()],
            time_from_start=time_from_start)])


def full_path(path):
    """Return the absolute path to a package resource.

    Arguments:
        path {str} -- relative path

    Returns:
        str -- absolute path
    """
    r = rospkg.RosPack()
    return os.path.join(r.get_path('prl_ur5_demos'), path)


def gazebo_spawn_urdf_model(name, filename, pose, ns=''):
    """Spawn the model.

    Arguments:
        name {str} -- name of the model to be spawn
        filename {str} -- path to the model description file
        pose {PoseStamped} -- initial pose

    Keyword Arguments:
        ns {str} -- spawn the model under this namespace (default: {''})
    """
    with open(filename) as f:
        model_xml = f.read()
    rospy.wait_for_service('gazebo/spawn_urdf_model', timeout=5.0)
    spawn_model = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
    spawn_model(name, model_xml, ns, pose.pose, pose.header.frame_id)


def gazebo_delete_model(name):
    """Delete the model.

    Arguments:
        name {str} -- name of the Gazebo Model to be deleted
    """
    rospy.wait_for_service('/gazebo/delete_model', timeout=5.0)
    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    delete_model(name)
