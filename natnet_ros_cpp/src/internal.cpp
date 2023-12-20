#include "../include/internal.h"

void Internal::Init(ros::NodeHandle &n)
{
    this->rosparam.getNset(n);
}

int Internal::ConnectClient(NatNetClient* g_pClient, sNatNetClientConnectParams &g_connectParams)
{
    // 每次重新启动即释放之前的服务器
    g_pClient->Disconnect();

    // 初始化客户端并且连接至natnet服务器
    int retCode = g_pClient->Connect( g_connectParams );
    if (retCode != ErrorCode_OK)
    {
        ROS_ERROR("Unable to connect to server.  Error code: %d. Exiting.", retCode);
        return ErrorCode_Internal;  // 返回错误1
    }
    else
    {
        // 连接成功
        void* pResult;      // 应用程序定义的响应
        int nBytes = 0;     // 响应中的字节数
        ErrorCode ret = ErrorCode_OK;

        // 打印服务器信息
        memset( &g_serverDescription, 0, sizeof( g_serverDescription ) );
        ret = g_pClient->GetServerDescription( &g_serverDescription );
        if ( ret != ErrorCode_OK || ! g_serverDescription.HostPresent )
        {
            ROS_ERROR("Unable to connect to server. Host not present. Exiting.");
            return 1;
        }
        

        // 获得动捕帧率
        ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);    // 第一个变量是natnet命令
        if (ret == ErrorCode_OK)
        {
            float fRate = *((float*)pResult);
            // 因为pResult是空指针，需要先变成float再取值
            ROS_INFO("Mocap Framerate : %3.2f", fRate);
        }
        else
            ROS_ERROR("Error getting frame rate.");

        // // get # of analog samples per mocap frame of data
        // ret = g_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
        // if (ret == ErrorCode_OK)
        // {
        //     //g_analogSamplesPerMocapFrame = *((int*)pResult);
        //     ROS_INFO("Analog Samples Per Mocap Frame : %d", *((int*)pResult));
        // }
        // else
        //     ROS_ERROR("Error getting Analog frame rate.");
    }

    return ErrorCode_OK;
}

void Internal::MessageHandler( Verbosity msgType, const char* msg )
{
    // Optional: Filter out debug messages
    if ( msgType < Verbosity_Info )
    {
        return;
    }

    printf( "\n[NatNetLib]" );

    switch ( msgType )
    {
        case Verbosity_Debug:
            printf( " [DEBUG]" );
            break;
        case Verbosity_Info:
            printf( "  [INFO]" );
            break;
        case Verbosity_Warning:
            printf( "  [WARN]" );
            break;
        case Verbosity_Error:
            printf( " [ERROR]" );
            break;
        default:
            printf( " [?????]" );
            break;
    }

    printf( ": %s", msg );
}

/*********************************************************数据说明******************************************************************/
// 从motive中取回数据说明，包括刚体，骨架的个数、id等
void Internal::Info(NatNetClient* g_pClient, ros::NodeHandle &n)
{
    ROS_INFO("Requesting Data Descriptions...");
    sDataDescriptions* pDataDefs = NULL;        //数据说明储存在这个里面
    int iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
    if (iResult != ErrorCode_OK || pDataDefs == NULL)
    {
        ROS_INFO("Unable to retrieve Data Descriptions.");
    }
    else
    {
        ROS_INFO("Received %d Data Descriptions:", pDataDefs->nDataDescriptions );      // 有多少个数据说明

        for(int i = 0; i < pDataDefs->nDataDescriptions; i++)
        {
            if(pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton)           // 2为骨架
            {
                sSkeletonDescription* pSK = pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;
                printf("Skeleton Name : %s\n", pSK->szName);
                printf("Skeleton ID : %d\n", pSK->skeletonID);
                printf("RigidBody (Bone) Count : %d\n", pSK->nRigidBodies);
                for (int j = 0; j < pSK->nRigidBodies; j++)
                {
                    sRigidBodyDescription* pRB = &pSK->RigidBodies[j];
                    ROS_INFO("RigidBody found : %s", pRB->szName);
                    ROS_INFO_COND(rosparam.log_internals, "RigidBody ID : %d", pRB->ID);
                    ROS_INFO_COND(rosparam.log_internals, "RigidBody Parent ID : %d", pRB->parentID);
                    ROS_INFO_COND(rosparam.log_internals, "Parent Offset : %3.2f,%3.2f,%3.2f", pRB->offsetx, pRB->offsety, pRB->offsetz);
                
                    // 将刚体信息通过ros发布
                    std::string body_name(pRB->szName);     // 用字面值常量进行初始化
                    if(rosparam.pub_rigid_body)
                    {
                        // 在这里定义了直接从natnet获取到的刚体数据发布的话题，以body_name+"/pose"的名字进行发布，缓冲区50个
                        this->ListRigidBodies[pRB->ID] = body_name;
                        this->RigidbodyPub[pRB->szName] = n.advertise<geometry_msgs::PoseStamped>(body_name+"/pose", 50);
                    }
                    // if ( pRB->MarkerPositions != NULL && pRB->MarkerRequiredLabels != NULL )
                    // {
                    //     for ( int markerIdx = 0; markerIdx < pRB->nMarkers; ++markerIdx )
                    //     {
                    //         const MarkerData& markerPosition = pRB->MarkerPositions[markerIdx];
                    //         const int markerRequiredLabel = pRB->MarkerRequiredLabels[markerIdx];
                    //         // Creating publisher for the markers of the rigid bodies
                    //         if(rosparam.pub_rigid_body_marker)
                    //             this->RigidbodyMarkerPub[std::to_string(pRB->ID)+std::to_string(markerIdx+1)] = n.advertise<geometry_msgs::PointStamped>(body_name+"/marker"+std::to_string(markerIdx)+"/pose", 50);
                    //         ROS_INFO_COND(rosparam.log_internals,  "\tMarker #%d:", markerIdx );
                    //         ROS_INFO_COND(rosparam.log_internals,  "\t\tPosition: %.2f, %.2f, %.2f", markerPosition[0], markerPosition[1], markerPosition[2] );

                    //         if ( markerRequiredLabel != 0 )
                    //         {
                    //             ROS_INFO_COND(rosparam.log_internals,  "\t\tRequired active label: %d", markerRequiredLabel );
                    //         }
                    //     }
                    // }
                }
            }
            // else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Camera)
            // {
            //     // Camera
            //     sCameraDescription* pCamera = pDataDefs->arrDataDescriptions[i].Data.CameraDescription;
            //     ROS_INFO_COND(rosparam.log_internals, "Camera Name : %s", pCamera->strName);
            //     ROS_INFO_COND(rosparam.log_internals, "Camera Position (%3.2f, %3.2f, %3.2f)", pCamera->x, pCamera->y, pCamera->z);
            //     ROS_INFO_COND(rosparam.log_internals, "Camera Orientation (%3.2f, %3.2f, %3.2f, %3.2f)", pCamera->qx, pCamera->qy, pCamera->qz, pCamera->qw);
            // }
            else
            {
                ROS_WARN_COND(rosparam.log_internals, "Unknown data type detected.");
                // Unknown
            }
        }
        if (rosparam.pub_pointcloud)
        {
            this->PointcloudPub = n.advertise<sensor_msgs::PointCloud>("pointcloud",50);
        }
        if (rosparam.pub_individual_marker)
        {
            for (int i=0; i<(int) rosparam.object_list.size(); i++)
            {
                this->IndividualMarkerPub[rosparam.object_list[i].name] = n.advertise<geometry_msgs::PoseStamped>(rosparam.object_list[i].name+"/pose", 50);
            }
        }
    }
}

void Internal::LatenciInfo(sFrameOfMocapData* data, void* pUserData, Internal &internal)
{
    NatNetClient* pClient = (NatNetClient*) pUserData;
    // This figure may appear slightly higher than the "software latency" reported in the Motive user interface,
    // because it additionally includes the time spent preparing to stream the data via NatNet.
    const uint64_t softwareLatencyHostTicks = data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
    const double softwareLatencyMillisec = (softwareLatencyHostTicks * 1000) / static_cast<double>(internal.g_serverDescription.HighResClockFrequency);
    // Transit latency is defined as the span of time between Motive transmitting the frame of data, and its reception by the client (now).
    // The SecondsSinceHostTimestamp method relies on NatNetClient's internal clock synchronization with the server using Cristian's algorithm.
    const double transitLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->TransmitTimestamp ) * 1000.0;
    ROS_INFO_COND(internal.rosparam.log_latencies, "Software latency : %.2lf milliseconds", softwareLatencyMillisec);
    // If it's unavailable (for example, with USB camera systems, or during playback), this field will be zero.
    const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;
    if ( bSystemLatencyAvailable )
    {
        const uint64_t systemLatencyHostTicks = data->TransmitTimestamp - data->CameraMidExposureTimestamp;
        const double systemLatencyMillisec = (systemLatencyHostTicks * 1000) / static_cast<double>(internal.g_serverDescription.HighResClockFrequency);
        
        const double clientLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->CameraMidExposureTimestamp ) * 1000.0;
        ROS_INFO_COND(internal.rosparam.log_latencies,  "System latency : %.2lf milliseconds", systemLatencyMillisec );
        ROS_INFO_COND(internal.rosparam.log_latencies,  "Total client latency : %.2lf milliseconds (transit time +%.2lf ms)", clientLatencyMillisec, transitLatencyMillisec );
    }
    else
    {
        ROS_INFO_COND(internal.rosparam.log_latencies,  "Transit latency : %.2lf milliseconds", transitLatencyMillisec );
    }
}

void Internal::DataHandler(sFrameOfMocapData* data, void* pUserData, Internal &internal)
{   
    // // 帧ID
    // ROS_INFO_COND(internal.rosparam.log_frames, "FrameID : %d", data->iFrame);

    // // 骨架个数
    // ROS_INFO_COND(internal.rosparam.log_frames, "Skeletons [Count=%d]\n", data->nSkeletons);
    
    for (int i = 0; i < data->nSkeletons; i++)
    {
        sSkeletonData skData = data->Skeletons[i];
        // ROS_INFO_COND(internal.rosparam.log_frames, "Skeleton [ID=%d  Bone count=%d]\n", skData.skeletonID, skData.nRigidBodies);
        // // 刚体（骨架中包含的）
        // ROS_INFO_COND(internal.rosparam.log_frames, "Rigid Bodies in Skeleton%d [Count=%d]", skData.skeletonID, data->nRigidBodies);
        for(int j=0; j < skData.nRigidBodies; j++)
        {
            sRigidBodyData rbData = skData.RigidBodyData[j];
            if(internal.rosparam.pub_rigid_body && (rbData.ID == 13 || rbData.ID == 10))
            {
                // PubRigidbodyPose(rbData, internal);
                // ROS_INFO_COND(internal.rosparam.log_frames, "Rigid Body [ID=%d  Error=%3.2f]", rbData.ID, rbData.MeanError);//, bTrackingValid);
                // ROS_INFO_COND(internal.rosparam.log_frames, "x\ty\tz\tqx\tqy\tqz\tqw");
                // ROS_INFO_COND(internal.rosparam.log_frames, "%3.2f \t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f",
                //     rbData.x, rbData.y, rbData.z, rbData.qx, rbData.qy, rbData.qz, rbData.qw);
            }
            // natnet_message_safequeue_.push(std::make_shared<sRigidBodyData>(rbData));
            internal.natnet_message_safequeue_.push(rbData);
        }
    }

    this->caculate_duration(internal);

}



void Internal::caculate_duration( Internal &internal)
{
    auto start_test = std::chrono::steady_clock::now();
    // std::cout<<" 时间: "<<std::chrono::time_point_cast<std::chrono::milliseconds>(start_test).time_since_epoch().count()<<" ms"<<std::endl;
    internal.start = std::chrono::time_point_cast<std::chrono::milliseconds>(start_test).time_since_epoch().count();
    
    if(internal.conut != 0&&((internal.start - internal.end)<=4))
    std::cout<<" 时间间隔: "<<(internal.start - internal.end)<<" ms"<<std::endl;

    // if(internal.conut != 0)
    // std::cout<<" 时间间隔: "<<(internal.start - internal.end)<<" ms"<<std::endl;

    auto end_test = std::chrono::steady_clock::now();
    // std::cout<<" 时间: "<<std::chrono::time_point_cast<std::chrono::milliseconds>(end_test).time_since_epoch().count()<<" ms"<<std::endl;
    internal.end = std::chrono::time_point_cast<std::chrono::milliseconds>(end_test).time_since_epoch().count();
    internal.conut++;
}

void Internal::PubRigidbodyPose(sRigidBodyData &data, Internal &internal)
{
    // Creating a msg to put data related to the rigid body and 
    geometry_msgs::PoseStamped msgRigidBodyPose;
    msgRigidBodyPose.header.frame_id = "base";

    msgRigidBodyPose.header.stamp = ros::Time::now();
    msgRigidBodyPose.pose.position.x = data.x;
    msgRigidBodyPose.pose.position.y = data.y;
    msgRigidBodyPose.pose.position.z = data.z;
    msgRigidBodyPose.pose.orientation.x = data.qx;
    msgRigidBodyPose.pose.orientation.y = data.qy;
    msgRigidBodyPose.pose.orientation.z = data.qz;
    msgRigidBodyPose.pose.orientation.w = data.qw;
    internal.RigidbodyPub[internal.ListRigidBodies[data.ID]].publish(msgRigidBodyPose);
    
    // creating tf frame to visualize in the rviz
    static tf2_ros::TransformBroadcaster tfRigidBodies;
    geometry_msgs::TransformStamped msgTFRigidBodies;
    msgTFRigidBodies.header.stamp = ros::Time::now();
    msgTFRigidBodies.header.frame_id = "world";
    msgTFRigidBodies.child_frame_id = internal.ListRigidBodies[data.ID];
    msgTFRigidBodies.transform.translation.x = data.x;
    msgTFRigidBodies.transform.translation.y = data.y;
    msgTFRigidBodies.transform.translation.z = data.z;
    msgTFRigidBodies.transform.rotation.x = data.qx;
    msgTFRigidBodies.transform.rotation.y = data.qy;
    msgTFRigidBodies.transform.rotation.z = data.qz;
    msgTFRigidBodies.transform.rotation.w = data.qw;
    tfRigidBodies.sendTransform(msgTFRigidBodies);
    
}

void Internal::PubMarkerPose(sMarker &data, Internal &internal)
{   
    int update = nn_filter(internal.rosparam.object_list, data, internal.rosparam.E,  internal.rosparam.E_x, internal.rosparam.E_y, internal.rosparam.E_z, internal.rosparam.individual_error, internal.rosparam.error_amp);
    if (update>=0)
    {   internal.UnlabeledCount+=1;
        internal.rosparam.object_list[update].detected = true;
        internal.rosparam.object_list[update].x = data.x;
        internal.rosparam.object_list[update].y = data.y;
        internal.rosparam.object_list[update].z = data.z;
    
        geometry_msgs::PoseStamped msgMarkerPose;
        msgMarkerPose.header.frame_id = "world";

        msgMarkerPose.header.stamp = ros::Time::now();
        msgMarkerPose.pose.position.x = data.x;
        msgMarkerPose.pose.position.y = data.y;
        msgMarkerPose.pose.position.z = data.z;
        msgMarkerPose.pose.orientation.x = 0.0;
        msgMarkerPose.pose.orientation.y = 0.0;
        msgMarkerPose.pose.orientation.z = 0.0;
        msgMarkerPose.pose.orientation.w = 1.0;
        internal.IndividualMarkerPub[internal.rosparam.object_list[update].name].publish(msgMarkerPose);

        // creating tf frame to visualize in the rviz
        static tf2_ros::TransformBroadcaster tfMarker;
        geometry_msgs::TransformStamped msgTFMarker;
        msgTFMarker.header.stamp = ros::Time::now();
        msgTFMarker.header.frame_id = "world";
        msgTFMarker.child_frame_id = internal.rosparam.object_list[update].name;
        msgTFMarker.transform.translation.x = data.x;
        msgTFMarker.transform.translation.y = data.y;
        msgTFMarker.transform.translation.z = data.z;
        msgTFMarker.transform.rotation.x = 0;
        msgTFMarker.transform.rotation.y = 0;
        msgTFMarker.transform.rotation.z = 0;
        msgTFMarker.transform.rotation.w = 1;
        tfMarker.sendTransform(msgTFMarker);
    }
}

void Internal::PubPointCloud(sMarker &data, Internal &internal)
{
    internal.msgPointcloud.header.frame_id="world";
    internal.msgPointcloud.header.stamp=ros::Time::now();
    
    geometry_msgs::Point32 msgPoint;
    msgPoint.x = data.x;
    msgPoint.y = data.y;
    msgPoint.z = data.z;
    internal.msgPointcloud.points.push_back(msgPoint);
}

void Internal::PubRigidbodyMarker(sMarker &data, Internal &internal)
{
    int modelID, markerID;
    NatNet_DecodeID( data.ID, &modelID, &markerID );

    geometry_msgs::PointStamped msgMarkerPose;
    msgMarkerPose.header.frame_id = std::to_string(modelID)+std::to_string(markerID);
    msgMarkerPose.header.stamp = ros::Time::now();

    msgMarkerPose.point.x = data.x;
    msgMarkerPose.point.y = data.y;
    msgMarkerPose.point.z = data.z;

    internal.RigidbodyMarkerPub[std::to_string(modelID)+std::to_string(markerID)].publish(msgMarkerPose);

}

