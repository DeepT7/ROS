/* Copyright (C) 2019-2020 hnqiu. All Rights Reserved.
 * Licensed under the BSD-3-Clause license. See LICENSE for details.
 */


#include <ros/ros.h>
#include "smb_highlevel_controller/SmbHighlevelController.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "smb_highlevel_controller");
    ros::NodeHandle nodeHandle("~");

    smb_highlevel_controller::SmbHighlevelController smbHighlevelController(nodeHandle);
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
