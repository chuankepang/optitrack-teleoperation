
#include "../include/internal.h"
#include "../deps/libxl/include/libxl.h"

#include "ros/ros.h"
#include <thread>
#include "ros/package.h"

 
#include <queue>
#include <mutex>
#include <condition_variable>


Internal internal;
libxl::Book* book_;
libxl::Sheet* sheet_;

int row_num_;
int col_num_;
int file_num_;
int file_num_old_;
size_t data_num_;



//Function to make callback to the datahandler when recieve the new frame.
void NATNET_CALLCONV FrameCallback(sFrameOfMocapData *data, void* pUserData)
{
    auto pClient = reinterpret_cast<Internal *>(pUserData);
    if (internal.rosparam.log_latencies)
        pClient->LatenciInfo(data, pUserData, internal);
    pClient->DataHandler(data, pUserData, internal);
}

void consumer()
{
    while(ros::ok)
    {
        auto rigidbodydata = internal.natnet_message_safequeue_.pop();
        int index = data_num_/5000;
        file_num_ = index;
        if(file_num_ == file_num_old_ + 1)
        {
            row_num_ = 1;
            book_->release();
            book_ = xlCreateBook();
            sheet_ = book_->addSheet("Sheet1");//添加一个工作表
        }
        std::string package_name = "natnet_ros_cpp";
        std::string package_path = ros::package::getPath(package_name);
        // std::cout << package_path << std::endl;
        std::string save_path = package_path + "/excel/save" + std::to_string(index) + ".xlsx";

        if (book_)//是否创建实例成功
        {
            //一个excel文件既是一个工作簿，你可以把工作簿看作是一个本子，而本子是由一页一页的纸张装订在一起的，excel中的sheet就是这些纸张。
            if (sheet_)
            {
                if(row_num_ == 1)
                {
                    sheet_->writeStr(row_num_, 0, "rigidbodydata.ID");
                    sheet_->writeStr(row_num_, 1, "x");
                    sheet_->writeStr(row_num_, 2, "y");
                    sheet_->writeStr(row_num_, 3, "z");
                    sheet_->writeStr(row_num_, 4, "qx");
                    sheet_->writeStr(row_num_, 5, "qy");
                    sheet_->writeStr(row_num_, 6, "qz");
                    sheet_->writeStr(row_num_, 7, "qw");
                    sheet_->writeStr(row_num_, 8, "MeanError");
                }
                else
                {
                    sheet_->writeNum(row_num_, 0, rigidbodydata.ID);
                    sheet_->writeNum(row_num_, 1, rigidbodydata.x);
                    sheet_->writeNum(row_num_, 2, rigidbodydata.y);
                    sheet_->writeNum(row_num_, 3, rigidbodydata.z);
                    sheet_->writeNum(row_num_, 4, rigidbodydata.qx);
                    sheet_->writeNum(row_num_, 5, rigidbodydata.qy);
                    sheet_->writeNum(row_num_, 6, rigidbodydata.qz);
                    sheet_->writeNum(row_num_, 7, rigidbodydata.qw);
                    sheet_->writeNum(row_num_, 8, rigidbodydata.MeanError);
                }
            }
        }
        row_num_++;
        data_num_++;
        file_num_old_ = file_num_;
        book_->save(save_path.c_str());

    }
}



int main( int argc, char **argv)
{
    // defining the default connection type ConnectionType_Multicast/ConnectionType_Unicast
    ConnectionType kDefaultConnectionType = ConnectionType_Multicast;
    // variable to handle the parameters to be passed to the natnet
    sNatNetClientConnectParams g_connectParams;
    // variable to store the server description 
    //sServerDescription g_serverDescription;

    ros::init(argc, argv,"natnet_ros_cpp");
    ros::NodeHandle n("~");
    // create NatNet client
    NatNetClient* g_pClient = new NatNetClient();
    // print version info
    unsigned char ver[4];
    NatNet_GetVersion( ver );
    ROS_INFO("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)", ver[0], ver[1], ver[2], ver[3]);
    
    // You must init before using the object to get required parameters from the rosparam server
    internal.Init(n);

    if (internal.rosparam.serverType == "unicast")
        kDefaultConnectionType = ConnectionType_Unicast;

    // Setting up parameters for the natnet connection
    g_connectParams.connectionType = kDefaultConnectionType;
    g_connectParams.serverCommandPort = internal.rosparam.serverCommandPort;
    g_connectParams.serverDataPort = internal.rosparam.serverDataPort;
    g_connectParams.serverAddress = internal.rosparam.serverIP.c_str();
    g_connectParams.localAddress = internal.rosparam.clientIP.c_str();
    g_connectParams.multicastAddress = internal.rosparam.serverType=="multicast" ? internal.rosparam.multicastAddress.c_str() : NULL;

    int iResult;
    // Connect to Motive
    iResult = internal.ConnectClient(g_pClient, g_connectParams);
    if (iResult != ErrorCode_OK)
    {
        ROS_ERROR("Error initializing client. See log for details. Exiting.");
        return 1;
    }
    else
    {
        ROS_INFO("Client initialized and ready.");
    }

    // Assembling the info from the optitrack, counting number of bodies etc..
    // Must extract the info before starting to publish the data.
    internal.Info(g_pClient, n);

    ROS_INFO("Client is connected to server and listening for data...");

    // Install natnet logging callback for some internal details
    internal.rosparam.log_internals ? NatNet_SetLogCallback( internal.MessageHandler ): internal.Pass();

    auto start_test = std::chrono::steady_clock::now();
    int64_t start = std::chrono::time_point_cast<std::chrono::milliseconds>(start_test).time_since_epoch().count();
    std::cout << start << std::endl;
    book_ = xlCreateBook();
    sheet_ = book_->addSheet("Sheet1");//添加一个工作表
    // if (book_)//是否创建实例成功
    // {
    //     libxl::Sheet* sheet = book_->addSheet("Sheet1");//添加一个工作表
    //     //一个excel文件既是一个工作簿，你可以把工作簿看作是一个本子，而本子是由一页一页的纸张装订在一起的，excel中的sheet就是这些纸张。
    //     if (sheet)
    //     {
    //         sheet->writeStr(1, 0, "Hello, World !");//在第二行 第二列（B列）的表格中写入字符串"Hello, World !"。程序中从0开始计数。第0行就是execl的第1行
    //         sheet->writeNum(2, 1, 1000);//在第三行 第二列（B列）的表格中写入数字 "1000"。
    //         sheet->writeNum(3, 1, 2000);

    //         libxl::Font* font = book_->addFont();//创建一个字体对象
    //         font->setColor(libxl::COLOR_RED);  //设置对象颜色
    //         font->setBold(true);        //设置粗体
    //         libxl::Format* boldFormat = book_->addFormat();//设置字体格式指针
    //         boldFormat->setFont(font);             //应用上面设置的字体
    //         sheet->writeFormula(6, 1, "SUM(B3:B4)", boldFormat); //用新的字体格式 在第七行 B列 写入 B3(第三行，第二列)+B4 的和

    //         libxl::Format* dateFormat = book_->addFormat();
    //         dateFormat->setNumFormat(libxl::NUMFORMAT_DATE);//设置日期格式，依赖于你本机的设置
    //         sheet->writeNum(8, 1, book_->datePack(2008, 4, 29), dateFormat);

    //         sheet->setCol(1, 1, 12);//设置列宽，格式等
    //     }

    //     if (book_->save("/home/wrs/test.xls"))//保存到example.xls
    //     {
    //         //.....
    //     }
    //     else
    //     {
    //         std::cout << book_->errorMessage() << std::endl;
    //     }
    //     book_->release();//释放对象！！！！！
    // }
    auto end_test = std::chrono::steady_clock::now();
    int64_t end = std::chrono::time_point_cast<std::chrono::milliseconds>(start_test).time_since_epoch().count();
    std::cout << end << std::endl;

    row_num_ = 1;
    col_num_ = 0;
    file_num_old_ = 0;
    file_num_ = 0;
    data_num_ = 0;

    // std::thread
    std::thread thread_consumer(consumer);

    while(ros::ok())
    {   
        g_pClient->SetFrameReceivedCallback( FrameCallback);
        // std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }

    thread_consumer.join();




    // g_pClient->SetFrameReceivedCallback( FrameCallback);  // this function will receive data from the server

    // 关闭客户端连接
    // if (g_pClient)
    // {
    //     g_pClient->Disconnect();
    //     delete g_pClient;
    //     g_pClient = NULL;
    // }

    return ErrorCode_OK;
}

