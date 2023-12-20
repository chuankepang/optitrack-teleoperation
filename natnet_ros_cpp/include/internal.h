#ifndef INTERNAL_H
#define INTERNAL_H

#include <NatNetCAPI.h>
#include <NatNetClient.h>

#include <map>

#include <ros/publisher.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/PointCloud.h>
#include <chrono>
#include <thread>
#include <queue>
#include <condition_variable>
#include <type_traits>


#include "nn_filter.h"

class Internal
{
public:

    template <typename T>
    class SafeQueue {

    public:
    using lock_type = std::unique_lock<std::mutex>;

    public:
    SafeQueue() = default;

    ~SafeQueue() = default;

    // template<typename IT>
    void push(T &item) {
        // static_assert(std::is_same<T, std::decay_t<IT>>::value , "Item type is not convertible!!!");
        {
        lock_type lock{mutex_};
        queue_.emplace(std::forward<T>(item));
        }
        cv_.notify_one();
    }

    auto pop() -> T {
        lock_type lock{mutex_};

        cv_.wait(lock, [&]() { return !queue_.empty(); });
        auto front = std::move(queue_.front());
        queue_.pop();
        return front;
    }

    private:
    std::queue<T> queue_;
    std::mutex mutex_;
    std::condition_variable cv_;

    };
    
    // Keeping count on the assets
    // NOTE: It will be only counted at the start. It is not advised to add rigid body after starting the node 
    std::map<int32_t,std::string> ListRigidBodies; 
    std::map<double,std::string> ListSkeletons;
    std::map<double,std::string> ListForcePlates;
    std::map<double,std::string> ListDevices;

    //Initializing the publishers for the ros
    std::map<std::string, ros::Publisher> RigidbodyPub;
    std::map<std::string, ros::Publisher> RigidbodyMarkerPub;
    std::map<std::string, ros::Publisher> IndividualMarkerPub;
    std::map<std::string, ros::Publisher> SkeletonPub;
    std::map<std::string, ros::Publisher> ForcePlatePub;
    std::map<std::string, ros::Publisher> DevicePub;
    ros::Publisher PointcloudPub;
    sensor_msgs::PointCloud msgPointcloud; // point cloud msg

    int markerCount = 0; // number of individual markers (unlabled + labled)
    int UnlabeledCount = 0;
    sServerDescription g_serverDescription;
    SetParam rosparam;
    int64_t start=0;
    int64_t end=0;
    size_t conut = 0;

    SafeQueue<sRigidBodyData> natnet_message_safequeue_;

    void Pass(){}; //Void function to do nothing
    void Init(ros::NodeHandle &n); //take care of ros params

    // Establish a NatNet Client connection
    int ConnectClient(NatNetClient* g_pClient, sNatNetClientConnectParams &g_connectParams);

    // MessageHandler receives NatNet error/debug messages
    static void MessageHandler( Verbosity msgType, const char* msg );

    // Function to get some useful information from the motion capture system
    // Enable log_internal parameter to have a look at it. 
    void Info(NatNetClient* g_pClient, ros::NodeHandle &n);

    //Provides information on the latencies of the different systems
    void LatenciInfo(sFrameOfMocapData* data, void* pUserData, Internal &internal);
    
    // Handles the data from the frame and publish it as ROS topics
    void DataHandler(sFrameOfMocapData* data, void* pUserData, Internal &internal);

    // Publishes the data of rigidbodies
    void PubRigidbodyPose(sRigidBodyData &data, Internal &internal);

    // Publish markers of the rigidbodies
    void PubRigidbodyMarker(sMarker &data, Internal &internal);

    // Publish Single marker as the Rigidbody and TF
    void PubMarkerPose(sMarker &data, Internal &internal);

    // Publish Point cloud from the marker
    void PubPointCloud(sMarker &data, Internal &internal);

    //caculate duration
    void caculate_duration(Internal &internal);






};



#endif
