//
// Created by zhouhua on 2020/3/31.
//

#include "serial/serialConnection.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "mserial");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle nh("~");

    std::string serial_port, acm_port;
    double serial_baund, acm_baund;
    nh.param<std::string>("serial_port", serial_port, "/dev/NoopS0");
//    nh.param<std::string>("acm_port", acm_port, "/dev/ttyACM0");
    nh.param<std::string>("acm_port", acm_port, "/dev/MasterUAV");
    nh.param<double>("serial_baund", serial_baund, 921600);
    nh.param<double>("acm_baund", acm_baund, 921600);
//    nh.param<double>("acm_baund", acm_baund, 57600);

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
    bool use_noop = false;


    try {
        //打开串口
        serial_port_USB.open();
    }
    catch(serial::IOException& e)  {
        util_log("Unable serial_time open port.%s" , serial_port.c_str()) ;
    }

    sleep(10); // waiting for px4 weak up.

    try {
        //打开串口
        serial_port_ACM.open();
    } catch(serial::IOException& e)  {
        util_log("Waiting for ..%s" , acm_port.c_str());
    }

    //判断串口是否打开成功
    if(serial_port_ACM.isOpen()) {
        util_log("opened. %s" , acm_port.c_str() );
        use_acm0 = true;
    } else {
        use_acm0 = false;
    }

    //判断串口是否打开成功
    if(serial_port_USB.isOpen()) {
        util_log("opened. %s", serial_port.c_str());
        use_noop = true;
    } else {
        use_noop = false;
    }

    ros::Rate loop_rate(1000);
    uint8_t buffer_USB[10000];
    uint8_t buffer_ACM[10000];
    while(ros::ok())
    {
        try {
            if (!use_acm0) {
                serial_port_ACM.setPort(acm_port);
                serial_port_ACM.setBaudrate(acm_baund);
                serial_port_ACM.setTimeout(serial_time);
                serial_port_ACM.open();
                use_acm0 = true;
                util_log("reconnect!  .. %s" , acm_port.c_str());
            }
            size_t ACM_port = serial_port_ACM.available();
            if (ACM_port != 0) {
                ACM_port = serial_port_ACM.read(buffer_ACM, ACM_port);
                if (use_noop) serial_port_USB.write(buffer_ACM, ACM_port);
            }
        } catch (serial::IOException& e) {
            if (use_acm0) {
                serial_port_ACM.close();
                use_acm0 = false;
            }
            util_log("Failed!  .. %s" , acm_port.c_str());
        }


        //获取缓冲区内的字节数
        try {
            if (!use_noop) {
                serial_port_USB.setPort(serial_port);
                serial_port_USB.setBaudrate(serial_baund);
                serial_port_USB.setTimeout(serial_time);

                serial_port_USB.open();
                use_noop = true;
                util_log("reconnect!  .. %s", serial_port.c_str());
            }
            size_t USB_port = serial_port_USB.available();

            if(USB_port != 0) {
                //读出数据
                USB_port = serial_port_USB.read(buffer_USB, USB_port);
                if (use_acm0) serial_port_ACM.write(buffer_USB, USB_port);
            }
        } catch (serial::IOException& e) {
            if (use_noop) {
                serial_port_USB.close();
                use_noop = false;
            }
            util_log("Failed!  .. %s", serial_port.c_str());
        }

        ros::spinOnce();
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
