//
// Created by zhouhua on 2020/3/31.
//

#include "serial/serialConnection.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "mserial");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle nh;

    std::string serial_port, acm_port;
    double serial_baund, acm_baund;
    nh.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    nh.param<std::string>("acm_port", acm_port, "/dev/ttyACM0");
    nh.param<double>("serial_baund", serial_baund, 921600);
    nh.param<double>("acm_baund", acm_baund, 57600);
    //创建一个serial类
    serial::Serial serial_port_USB;
    serial::Serial serial_port_ACM;
    serial::Serial serial_port_ACM1;
    //创建timeout
    serial::Timeout serial_time = serial::Timeout::simpleTimeout(10000000);
    //设置要打开的串口名称
    serial_port_USB.setPort(serial_port);
    //设置串口通信的波特率
    serial_port_USB.setBaudrate(serial_baund);
    //串口设置timeout
    serial_port_USB.setTimeout(serial_time);

    serial_port_ACM.setPort(acm_port);
    serial_port_ACM.setBaudrate(acm_baund);
    serial_port_ACM.setTimeout(serial_time);
    bool use_acm0 = false;


    try {
        //打开串口
        serial_port_USB.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable serial_time open port.");
        return -1;
    }

    sleep(10); // waiting for px4 weak up.

    try
    {
        //打开串口
        serial_port_ACM.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Waiting for ACM0 port.");
    }

    //判断串口是否打开成功
    if(serial_port_ACM.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyACM0 is opened" );
        use_acm0 = true;
    } else {
        use_acm0 = false;
    }

    //判断串口是否打开成功
    if(serial_port_USB.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(1000);

    uint8_t buffer_USB[10000];
    uint8_t buffer_ACM[10000];
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t USB_port = serial_port_USB.available();
        if (use_acm0) {
            size_t ACM_port = serial_port_ACM.available();
            if (ACM_port != 0) {
                ACM_port = serial_port_ACM.read(buffer_ACM, ACM_port);
/*//                cout << hex <<buffer_ACM << endl;
                for(int i=0; i<ACM_port; i++)
                {
                    //16进制的方式打印到屏幕
                    std::cout << std::hex << (buffer_ACM[i] & 0xff) << " ";
                }
                std::cout << std::endl;
//                std::cout << hex <<buffer_ACM << std::endl;*/
                serial_port_USB.write(buffer_ACM, ACM_port);
            }

        }

        if(USB_port != 0)
        {
            //读出数据
            USB_port = serial_port_USB.read(buffer_USB, USB_port);
            if (use_acm0) serial_port_ACM.write(buffer_USB, USB_port);
        }


        loop_rate.sleep();
    }

    //关闭串口
    serial_port_USB.close();
    serial_port_ACM.close();

    return 0;
}


SerialConnection::SerialConnection() {

}

SerialConnection::~SerialConnection() {

}
