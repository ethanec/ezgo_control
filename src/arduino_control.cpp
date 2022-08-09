#include <ezgo_control/SerialPort.hpp>
#include <ezgo_control/ezgo_vehicle.hpp>

using namespace mn::CppLinuxSerial;

pthread_mutex_t mutex;
vehicle_config_t vehicle_config;
vehicle_info_t vehicle_info;
vehicle_cmd_t vehicle_cmd;
vehicle_cmd_t prev_vehicle_cmd;
int willExit = 0;

ros::Publisher serial_pub;
ros::Publisher mode_pub;

// writer
// void SteeringControl(float cmd_steering_angle)
// {
//     // TODO TODO TODO FIXME: positive angle == left
//     int16_t int16_angle = (int16_t) cmd_steering_angle;
//     static char msg[2];

//     // Byte order: Big-endian
//     // high byte
//     msg[0] = (int16_angle >> 8) & 0x00ff;
//     // low byte
//     msg[1] = int16_angle & 0x00ff;
// }

static std::vector<uint8_t> setVehicleGearDrv()
{   
    std::vector<uint8_t> msg;
    
    // gear
    char current_gear = vehicle_info.shift;
    char cmd_gear = vehicle_cmd.shift;
    float speed = vehicle_info.velocity;

    if(cmd_gear == current_gear){
        msg[2] = (uint8_t) current_gear;
    }else if(cmd_gear != current_gear && speed == 0.){
        msg[2] = (uint8_t) cmd_gear;
    }

    // drv
    // #define WHEEL_TO_STEERING (v_config.STEERING_ANGLE_MAX / v_config.WHEEL_ANGLE_MAX)
    // static float pre_cmd_steering_angle = 0.0;
    // float cmd_steering_angle;
    float cmd_velocity = vehicle_cmd.linear_x;
    float current_velocity = vehicle_info.velocity;

    // if (fabs(v_cmd.linear_x) < 0.1) {  // just avoid divided by zero.
    //     cmd_steering_angle = pre_cmd_steering_angle;
    // } else {
    //     double phi_angle_pi = (v_cmd.angular_z / v_cmd.linear_x);
    //     double wheel_angle_pi = phi_angle_pi * v_config.WHEEL_BASE;
    //     double wheel_angle = (wheel_angle_pi / M_PI) * 180.0;
    //     cmd_steering_angle = wheel_angle * WHEEL_TO_STEERING;


    //     // Limit the steering angle
    //     if (cmd_steering_angle < -v_config.STEERING_ANGLE_MAX)
    //         cmd_steering_angle = -v_config.STEERING_ANGLE_MAX;
    //     if (cmd_steering_angle > v_config.STEERING_ANGLE_MAX)
    //         cmd_steering_angle = v_config.STEERING_ANGLE_MAX;

    //     pre_cmd_steering_angle = cmd_steering_angle;
    // }

    vehicle_control((double) current_velocity, (double) cmd_velocity);
    // SteeringControl(cmd_steering_angle);

    msg[0] = (uint8_t) vehicle_cmd.accel_stroke;
    msg[1] = (uint8_t) vehicle_cmd.brake_stroke;

    return msg;
}

// reader
static void publish_serial_msg()
{
    autoware_can_msgs::CANInfo serial_msg;

    serial_msg.header.frame_id = "/serail_port";
    serial_msg.header.stamp = ros::Time::now();
    serial_msg.drivepedal = vehicle_info.throttle;
    serial_msg.brakepedal = vehicle_info.brake;
    // serial_msg.angle = 
    serial_msg.speed = vehicle_info.velocity;
    serial_msg.driveshift = vehicle_info.shift;
    serial_msg.light = vehicle_info.light;
    serial_pub.publish(serial_msg);

    tablet_socket_msgs::mode_info mode_msg;
    mode_msg.header.frame_id = "/mode";
    mode_msg.header.stamp = ros::Time::now();
    mode_msg.mode = vehicle_info.control_mode;
    mode_pub.publish(mode_msg);
}

// main
static void *writer_handler(void *args)
{
    std::cout << YELLOW << "ENTER ezgo_vehicle_control Writer thread.\n" << RESET << std::endl;
    SerialPort *serialPort = (SerialPort *) args;
	cmd_reset();
	
    while (ros::ok() && !willExit) {
        ros::spinOnce();
        std::vector<uint8_t> data;
        std::vector<uint8_t> pub_data;
        switch (vehicle_cmd.modeValue) {
        case 0: // manual mode
            cmd_reset();
            break;
        case 1:  // autonomous mode
            pub_data = setVehicleGearDrv();
            pthread_mutex_lock(&mutex);
            serialPort->WriteBinary(pub_data);
            pthread_mutex_unlock(&mutex);
            break;
        case 2:  // UI direct control
            if (vehicle_cmd.accel_stroke != prev_vehicle_cmd.accel_stroke || vehicle_cmd.brake_stroke != prev_vehicle_cmd.brake_stroke 
                || vehicle_cmd.shift != prev_vehicle_cmd.shift) {
                data.push_back((uint8_t) vehicle_cmd.accel_stroke);
                data.push_back((uint8_t) vehicle_cmd.brake_stroke);
                data.push_back((uint8_t) vehicle_cmd.shift);
                pthread_mutex_lock(&mutex);
                serialPort->WriteBinary(data);
                pthread_mutex_unlock(&mutex);
                prev_vehicle_cmd = vehicle_cmd;
            }
            break;
        }
    }
    std::cout << YELLOW << "EXIT ezgo_vehicle_control Writer thread.\n" << RESET << std::endl;
    return nullptr;
}

static void *reader_handler(void *args)
{
    std::cout << YELLOW << "ENTER ezgo_vehicle_control Reader thread.\n" << RESET << std::endl;
    SerialPort *serialPort = (SerialPort *) args;
    while (ros::ok() && !willExit) {
        std::string readData;
        pthread_mutex_lock(&mutex);
        serialPort->Read(readData);
        pthread_mutex_unlock(&mutex);
        if (readData.size() == 5) {
            // std::cout << "readData[0]: "<< int(readData[0]) << std::endl;
            vehicle_info.throttle = (uint8_t) readData[0];
            vehicle_info.brake = (uint8_t) readData[1];
            vehicle_info.control_mode = (uint8_t) readData[2] & 0x01;
            vehicle_info.light = (uint8_t) readData[2] & 0x02;
            vehicle_info.shift = (uint8_t) readData[2] & 0x04;
            uint16_t vel = 0;
            vel = (((uint16_t) readData[3]) << 8) & 0xFF00;
            vel |= ((uint16_t) readData[4]) & 0xFF;
            vehicle_info.velocity = ((float) vel) / 1000;
            showVehicleInfo();
            publish_serial_msg();
        } else {
            std::cout << RED << "Without Receive DATA, Check connect ...." << RESET << std::endl;
        }
    }
    std::cout << YELLOW << "EXIT ezgo_vehicle_control Reader thread.\n" << RESET << std::endl;
    return nullptr;
}

int main(int argc, char **argv)
{
    int ret = app_setup_signals();
    if (ret == -1) {
        printf("Fail to app_setup_signals\n");
        return -1;
    }

    ros::init(argc, argv, "arduino_ezgo");
    ros::NodeHandle nh;

    ros::Subscriber sub[6];
    sub[0] = nh.subscribe("/twist_cmd", 1, twistCMDCallback);
    sub[1] = nh.subscribe("/mode_cmd", 1, modeCMDCallback);
    sub[2] = nh.subscribe("/gear_cmd", 1, gearCMDCallback);
    sub[3] = nh.subscribe("/accel_cmd", 1, accellCMDCallback);
    sub[4] = nh.subscribe("/steer_cmd", 1, steerCMDCallback);
    sub[5] = nh.subscribe("/brake_cmd", 1, brakeCMDCallback);

    serial_pub = nh.advertise<autoware_can_msgs::CANInfo>("serial_info", 100);
    mode_pub = nh.advertise<tablet_socket_msgs::mode_info>("mode_info", 100);

    pthread_t thread_writer;
    pthread_t thread_reader;

    // Create serial port object and open serial port
    SerialPort serialPort("/dev/ttyACM0", BaudRate::B_115200, NumDataBits::EIGHT, Parity::NONE, NumStopBits::ONE);
    serialPort.SetTimeout(-1);
    serialPort.Open();

    if (pthread_create(&thread_writer, NULL, writer_handler, &serialPort)) {
        perror("could not create thread for writer_handler");
        return -1;
    }

    if (pthread_create(&thread_reader, NULL, reader_handler, &serialPort)) {
        perror("could not create thread for reader_handler");
        return -1;
    }

    if (pthread_detach(thread_writer) != 0) {
        std::perror("pthread_detach");
        std::exit(1);
    }

    if (pthread_detach(thread_reader) != 0) {
        std::perror("pthread_detach");
        std::exit(1);
    }

    ros::AsyncSpinner spinner(4);  // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    serialPort.Close();
    return 0;
}
