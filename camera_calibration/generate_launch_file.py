import argparse
import pickle
import xml.etree.ElementTree as ET

import numpy as np
import rospy
import tf
from scipy.spatial.transform import Rotation


def create_launch_file(node_name, pkg_name, type_name, file_name, args):
    root = ET.Element("launch")

    node = ET.SubElement(root, "node")
    node.set("name", node_name)
    node.set("pkg", pkg_name)
    node.set("type", type_name)
    node.set("args", args)

    tree = ET.ElementTree(root)
    xml_str = ET.tostring(root, encoding="utf8", method="xml").decode()

    with open(file_name + ".launch", "w") as f:
        f.write(xml_str)


def main():
    parser = argparse.ArgumentParser(
        "Generate a launch file that published the tf of parent -> child"
    )
    parser.add_argument("--parent", type=str, required=True)
    parser.add_argument("--child", type=str, required=True)
    parser.add_argument("--filename", type=str, required=True)
    parser.add_argument("--parentName", type=str, required=True)
    parser.add_argument("--childName", type=str, required=True)
    args = parser.parse_args()

    rospy.init_node("tf_listener_node")

    listener = tf.TransformListener()

    # the frame where the data originated
    # parent_frame = "/camera_color_optical_frame"
    parent_frame = args.parent

    # the frame where the data should be transformed
    # child_frame = "/panda_link0"
    child_frame = args.child

    listener.waitForTransform(
        parent_frame, child_frame, rospy.Time(), rospy.Duration(4.0)
    )

    # lookupTransform(target_frame, source_frame) -> ^{target}T_{source}
    # lookupTransform(parent_frame, child_frame) -> ^{child}T_{parent}
    (trans, rot) = listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))

    node_name = args.filename
    launchfile_args = (
        " ".join(map(str, trans))
        + "   "
        + " ".join(map(str, rot))
        + f" {args.parentName} {args.childName}"
    )

    create_launch_file(
        node_name,
        "tf2_ros",
        "static_transform_publisher",
        args.filename,
        launchfile_args,
    )

    print(f"*** Created file {args.filename}.launch ***")


if __name__ == "__main__":
    main()
