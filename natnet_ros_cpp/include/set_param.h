#ifndef SET_PARAM_H
#define SET_PARAM_H

#include <string.h>
#include <vector>

#include "ros/node_handle.h"
#include "ros/console.h"

struct object_data
{
    std::string name;
    float x;
    float y;
    float z;
    bool detected;
    int marker_config;
};

class SetParam
{

public:
    bool log_internals=false; // 记录internal参数和动捕系统的测量数据
    bool log_frames=false; // 记录帧数据
    bool log_latencies=false; // 记录系统延迟
    bool pub_rigid_body = false; 
    bool pub_rigid_body_marker = false; 
    bool pub_individual_marker = false; 
    bool pub_pointcloud = false; 
    std::string serverIP;
    std::string clientIP;
    std::string serverType;
    std::string multicastAddress;
    int serverCommandPort;
    int serverDataPort;

    bool nearest_nbr = true;
    bool kalman = false;
    bool individual_error = false;
    float E=0.01*0.01, E_x=0.01, E_y=0.01, E_z=0.01;
    float error_amp = 1.0;
    
    std::vector<std::string> object_names;
    std::vector<object_data> object_list;
    std::vector<object_data> object_list_prev;
    std::vector<float> temp_pose = {0.0,0.0,0.0};
    object_data temp_obj;

    void getNset(ros::NodeHandle &n)
    {
        n.getParam("log_internals", log_internals);
        n.getParam("log_frames", log_frames);
        n.getParam("log_latencies", log_latencies);
        n.getParam("pub_rigid_body", pub_rigid_body);
        n.getParam("pub_rigid_body_marker", pub_rigid_body_marker);
        n.getParam("pub_individual_marker", pub_individual_marker);
        n.getParam("pub_pointcloud", pub_pointcloud);

        if (n.getParam("serverIP", serverIP))
        {
            ROS_INFO("Got server IP : %s", serverIP.c_str());
        }
        else
        {
            ROS_WARN("Failed to get server IP, using default 10.1.83.130");
            serverIP = "10.1.83.130";
        }

        if (n.getParam("clientIP", clientIP))
        {
            ROS_INFO("Got client IP : %s", clientIP.c_str());
        }
        else
        {
            ROS_WARN("Failed to get client IP, using default 10.1.82.91");
            clientIP = "10.1.82.91";
        }

        if (n.getParam("serverType", serverType))
        {
            ROS_INFO("Got server Type : %s", serverType.c_str());
        }
        else
        {
            ROS_WARN("Failed to get server type, using default multicast");
            serverType = "multicast";
        }

        if (serverType == "multicast")
        {
            if (n.getParam("multicastAddress", multicastAddress))
                ROS_INFO("Got server Type : %s", multicastAddress.c_str());
            else
            {
                ROS_WARN("Failed to get server IP, using default multicast address 239.255.42.99");
                multicastAddress = "239.255.42.99";
            }
        }

        if (n.getParam("serverCommandPort", serverCommandPort))
        {
            ROS_INFO("Got server Command Port : %i", serverCommandPort);
        }
        else
        {
            ROS_WARN("Failed to get server command port, using default 1510");
            serverCommandPort = 1510;
        }

        if (n.getParam("serverDataPort", serverDataPort))
        {
            ROS_INFO("Got server Command Port : %i", serverDataPort);
        }
        else
        {
            ROS_WARN("Failed to get server command port, using default 1511");
            serverDataPort = 1511;
        }
        if (pub_individual_marker)
        {
            n.getParam("individual_error", individual_error);
            n.getParam("E", E);
            E = E*E;
            n.getParam("E_x", E_x);
            n.getParam("E_y", E_y);
            n.getParam("E_z", E_z);

            if (n.getParam("object_names", object_names))
            {
                ROS_INFO("Got list of %li objects", object_names.size());
            }
            else
            {
                ROS_WARN("Unable to get the list of objects");
                ros::shutdown();
            }

            for (int i=0 ; i < (int)object_names.size() ; i++)
            {
                n.getParam(object_names[i]+"/pose/position",temp_pose);
                temp_obj.name = object_names[i];
                temp_obj.x = temp_pose[0];
                temp_obj.y = temp_pose[1];
                temp_obj.z = temp_pose[2];
                temp_obj.detected = false;
                object_list.push_back(temp_obj);
                n.getParam(object_names[i]+"/marker_config",temp_obj.marker_config);
                ROS_INFO("Got initial position of %s : [%f %f %f]",temp_obj.name.c_str(),temp_obj.x , temp_obj.y, temp_obj.z);
            }
            object_list_prev = object_list;
        }
    }

};

#endif

