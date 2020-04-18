//
// Created by zhouhua on 2020/3/31.
//

#include "serial/MasterDroneConnection.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "master_serial");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ROS_INFO("------> init!");
    MasterConnection mConnection;
    mConnection.init();
}


MasterConnection::MasterConnection() {

}

MasterConnection::~MasterConnection() {

}

void MasterConnection::init() {
    ros::NodeHandle nh;
    nh.param<std::string>("slave1_port", slave1_port, "/dev/NoopS0");
    nh.param<std::string>("slave2_port", slave2_port, "/dev/NoopS1");
//    nh.param<std::string>("slave3_port", slave3_port, "/dev/NoopS2");
    nh.param<std::string>("slave3_port", slave3_port, "/dev/NoopS3");
    nh.param<std::string>("master_port", master_port, "/dev/ttyACM0");
    nh.param<double>("slave_baund", slave_baund, 921600);
    nh.param<double>("master_baund", master_baund, 57600);
//    telem_port = "/dev/NoopS3";
    telem_port = "/dev/NoopS2";
    initSerial(slave1_serial_, slave1_port, slave_baund, serial_time);
    initSerial(slave2_serial_, slave2_port, slave_baund, serial_time);
    initSerial(slave3_serial_, slave3_port, slave_baund, serial_time);
    initSerial(telem_serial_, telem_port, slave_baund, serial_time);
    initSerial(master_serial_, master_port, master_baund, serial_time);

    sleep(10); // waiting for px4 weak up.

    master_ready = openSerial(master_serial_, master_port);
    slave1_ready = openSerial(slave1_serial_, slave1_port);
    slave2_ready = openSerial(slave2_serial_, slave2_port);
    slave3_ready = openSerial(slave3_serial_, slave3_port);
    telem_ready = openSerial(telem_serial_, telem_port);

    ros::Rate loop_rate(1000);

    while(ros::ok())
    {
        readSerial(master_serial_, telem_serial_, master_ready, telem_ready);
        readSerial(slave1_serial_, telem_serial_, slave1_ready, telem_ready);
        readSerial(slave2_serial_, telem_serial_, slave2_ready, telem_ready);
        readSerial(slave3_serial_, telem_serial_, slave3_ready, telem_ready);
        ros::spinOnce();
        loop_rate.sleep();
    }

    //关闭串口
    master_serial_.close();
    slave1_serial_.close();
    slave2_serial_.close();
    slave3_serial_.close();
    telem_serial_.close();
}


void MasterConnection::initSerial(serial::Serial &slave_port, string port, double baund, serial::Timeout &time) {
    //设置要打开的串口名称
    slave_port.setPort(port);
    //设置串口通信的波特率
    slave_port.setBaudrate(baund);
    //串口设置timeout
    slave_port.setTimeout(time);
}

bool MasterConnection::openSerial(serial::Serial &serial, string port) {
    try
    {
        //打开串口
        serial.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Waiting for port: " << port);
    }

    //判断串口是否打开成功
    if(serial.isOpen())
    {
        ROS_INFO_STREAM("Opened: " << port);
        return true;
    }
    return false;
}

bool MasterConnection::readSerial(serial::Serial &serial_in, serial::Serial &serial_out,
                                    bool serial_in_ready, bool serial_out_ready) {
    uint8_t buffer_back[10000];
    uint8_t buffer_in[10000];
    //获取缓冲区内的字节数
    if (serial_in_ready && serial_out_ready) {
        size_t port = serial_in.available();
        size_t serial_back = serial_out.available();
        if (port != 0) {
            port = serial_in.read(buffer_in, port);
            serial_out.write(buffer_in, port);

/*            for(int i=0; i<port; i++)
            {
                //16进制的方式打印到屏幕
                std::cout << std::hex << (buffer_in[i] & 0xff) << " ";
            }
            std::cout << std::endl;*/
        }

        if (serial_back != 0) {
            serial_back = serial_out.read(buffer_back, serial_back);
            serial_in.write(buffer_back, serial_back);
        }
        return true;
    }
    return false;
}
