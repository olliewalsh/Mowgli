/*
 * C++ Main
 *
 * contains the rosserial handlers
 *
 */

#include "board.h"

#include <cpp_main.h>
#include "main.h"
#include "panel.h"
#include "stm32f1xx_hal.h"
#include "ringbuffer.h"
#include "ros.h"
#include "ros/time.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt32.h"
#include "nav_msgs/Odometry.h"
#include "nbt.h"
#include "geometry_msgs/Twist.h"

#ifdef OPENMOWER_INTERFACE
#include "mower_msgs/MowerControlSrv.h"
#include "mower_msgs/EmergencyStopSrv.h"
#include "mower_msgs/Status.h"
#include "mower_msgs/ESCStatus.h"
#endif

#if SOFT_I2C_ENABLED
// IMU
#include "imu/imu.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"
#include "mowgli/magnetometer.h"
#endif

#define MAX_MPS	  	0.6		 	// Allow maximum speed of 0.6 m/s
#define PWM_PER_MPS 300.0		// PWM value of 300 means 1 m/s bot speed

#define WHEEL_BASE  0.325		// The distance between the center of the wheels in meters
#define WHEEL_DIAMETER 0.198 	// The diameter of the wheels in meters

#define BROADCAST_NBT_TIME_MS 20 	// 20ms interval where we set drive motors and read back values

extern uint8_t RxBuffer[RxBufferSize];
struct ringbuffer rb;

ros::Time last_cmd_vel(0, 0);
uint32_t last_cmd_vel_age;

// drive motor control
static uint8_t left_speed=0;
static uint8_t right_speed=0;
static uint8_t left_dir=0;
static uint8_t right_dir=0;

// blade motor control
static uint8_t blade_on_off=0;

ros::NodeHandle nh;

#ifdef OPENMOWER_INTERFACE
bool emergency_high_level = false;
#endif

// TF
geometry_msgs::Quaternion quat;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
char base_link[] = "base_link";
char odom[] = "odom";

//double radius = 0.04;                              //Wheel radius, in m
//double wheelbase = 0.187;                          //Wheelbase, in m
double two_pi = 6.28319;
double speed_act_left = 0.0;
double speed_act_right = 0.0;
double speed_req1 = 0.0;
double speed_req2 = 0.0;
double speed_dt = 0.0;
double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;
ros::Time current_time;
// ros::Time speed_time;
// ---
// double rate = 10.0;
double linear_scale_positive = 1.0;
double linear_scale_negative = 1.0;
double angular_scale_positive = 1.0;
double angular_scale_negative = 1.0;
bool publish_tf = true; // publish odom -> base_link transform
double dt = 0.0;
double dx = 0.0;
double dy = 0.0;
double dth = 0.0;
double dxy = 0.0;
double vx = 0.0;
double vy = 0.0;
// double vth = 0.0;
// ros::Duration d(0,1000000);
/*
double x = 1.0;
double y = 0.0;
double theta = 1.57;
*/

#if defined OPENMOWER_INTERFACE
mower_msgs::Status status_msg;
#else
// std_msgs::String str_msg;
std_msgs::Float32 f32_battery_voltage_msg;
std_msgs::Float32 f32_charge_voltage_msg;
std_msgs::Float32 f32_charge_current_msg;
std_msgs::Int16 int16_charge_pwm_msg;
std_msgs::Bool bool_blade_state_msg;
std_msgs::Bool bool_charging_state_msg;
nav_msgs::Odometry odom_msg;
std_msgs::UInt32 left_encoder_ticks_msg;
std_msgs::UInt32 right_encoder_ticks_msg;
#endif

#if defined SOFT_I2C_ENABLED && SOFT_I2C_ENABLED
// IMU
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField imu_mag_msg;
sensor_msgs::MagneticField imu_mag_calibration_msg;
//mowgli::magnetometer imu_mag_calibration_msg;
#endif

/*
 * PUBLISHERS
 */
#if defined OPENMOWER_INTERFACE
ros::Publisher pubStatus("mower/status", &status_msg);
#else
// ros::Publisher chatter("version", &str_msg);
ros::Publisher pubBatteryVoltage("battery_voltage", &f32_battery_voltage_msg);
ros::Publisher pubChargeVoltage("charge_voltage", &f32_charge_voltage_msg);
ros::Publisher pubChargeCurrent("charge_current", &f32_charge_current_msg);
ros::Publisher pubChargePWM("charge_pwm", &int16_charge_pwm_msg);
ros::Publisher pubChargeingState("charging_state", &bool_charging_state_msg);
ros::Publisher pubBladeState("blade_state", &bool_blade_state_msg);
ros::Publisher pubOdom("odom", &odom_msg);
ros::Publisher pubLeftEncoderTicks("left_encoder_ticks", &left_encoder_ticks_msg);
ros::Publisher pubRightEncoderTicks("right_encoder_ticks", &right_encoder_ticks_msg);
#endif

#if SOFT_I2C_ENABLED
// IMU
ros::Publisher pubIMU("imu/data_raw", &imu_msg);
ros::Publisher pubIMUMag("imu/mag", &imu_mag_msg);
ros::Publisher pubIMUMagCalibration("imu/mag_calibration", &imu_mag_calibration_msg);
#endif

/*
 * SUBSCRIBERS
 */
extern "C" void CommandVelocityMessageCb(const geometry_msgs::Twist& msg);
ros::Subscriber<geometry_msgs::Twist> subCommandVelocity("cmd_vel", CommandVelocityMessageCb);
#if defined OPENMOWER_INTERFACE
#else
extern "C" void CommandBladeOnMessageCb(const std_msgs::Bool& msg);
extern "C" void CommandBladeOffMessageCb(const std_msgs::Bool& msg);
ros::Subscriber<std_msgs::Bool> subBladeOn("cmd_blade_on", CommandBladeOnMessageCb);
ros::Subscriber<std_msgs::Bool> subBladeOff("cmd_blade_off", CommandBladeOffMessageCb);
#endif

// TODO ros::Subscriber<std_msgs::Bool> subLEDSet("cmd_panel_led_set", CommandLEDSetMessageCb);
// TODO ros::Subscriber<std_msgs::Bool> subLEDFlashSlow("cmd_panel_led_flash_slow", CommandLEDFlashSlowMessageCb);
// TODO ros::Subscriber<std_msgs::Bool> subLEDFlashFast("cmd_panel_led_flash_fast", CommandLEDFlashFastMessageCb);
// TODO ros::Subscriber<std_msgs::Bool> subLEDClear("cmd_panel_led_clear", CommandLEDClearMessageCb);

/*
 * SERVICES
 */

#if defined OPENMOWER_INTERFACE
extern "C" void SetMowEnableCb(mower_msgs::MowerControlSrvRequest &req, mower_msgs::MowerControlSrvResponse &res);
ros::ServiceServer<mower_msgs::MowerControlSrvRequest, mower_msgs::MowerControlSrvResponse> srvSetMowEnable(
	"mower_service/mow_enabled",
	(ros::ServiceServer<mower_msgs::MowerControlSrvRequest, mower_msgs::MowerControlSrvResponse>::CallbackT) SetMowEnableCb
);

extern "C" void SetEmergencyStopCb(mower_msgs::EmergencyStopSrvRequest &req, mower_msgs::EmergencyStopSrvResponse &res);
ros::ServiceServer<mower_msgs::EmergencyStopSrvRequest, mower_msgs::EmergencyStopSrvResponse> srvSetEmergencyStop(
	"mower_service/emergency",
	(ros::ServiceServer<mower_msgs::EmergencyStopSrvRequest, mower_msgs::EmergencyStopSrvResponse>::CallbackT)SetEmergencyStopCb
);
#else
#endif

/*
 * NON BLOCKING TIMERS
 */
static nbt_t ros_nbt;
static nbt_t publish_nbt;
static nbt_t motors_nbt;
static nbt_t panel_nbt;
static nbt_t broadcast_nbt;

extern "C" bool is_emergency() {
#if defined OPENMOWER_INTERFACE
    return emergency_high_level || emergency_state;
#else
    return emergency_state;
#endif
}

#if defined OPENMOWER_INTERFACE

extern "C" void SetMowEnableCb(mower_msgs::MowerControlSrvRequest &req, mower_msgs::MowerControlSrvResponse &res) {
    //if (req.mow_enabled && !is_emergency()) {
    if(req.mow_enabled) {
            blade_on_off = true;
    } else {
            blade_on_off = false;
    }
}

extern "C" void SetEmergencyStopCb(mower_msgs::EmergencyStopSrvRequest &req, mower_msgs::EmergencyStopSrvResponse &res) {
    if (req.emergency) {
        emergency_state |= 0x1;
    } else {
        emergency_state = 0;
    }
    emergency_high_level = req.emergency;
}

#else

/*
 * receive and parse cmd_blade_on messages
 * if True, turns ON the Blade Motor - (False is ignored, use the cmd_blade_off with a True message to turn it off)
 */
extern "C" void CommandBladeOnMessageCb(const std_msgs::Bool& msg)
{
    debug_printf("/cmd_blade_on: %d\r\n", msg.data);
    if (msg.data)
    {
        blade_on_off = true;
    }
}

/*
 * receive and parse cmd_blade_on messages
 * if True, turns OFF the Blade Motor - (False is ignored, use the cmd_blade_on with a True message to turn it ON)
 */
extern "C" void CommandBladeOffMessageCb(const std_msgs::Bool& msg)
{
    debug_printf("/cmd_blade_off: %d\r\n", msg.data);
    if (msg.data)
    {
        blade_on_off = false;
    }
}

#endif

/*
 * receive and parse cmd_vel messages
 * actual driving (updating drivemotors) is done in the drivemotors_nbt
 */
extern "C" void CommandVelocityMessageCb(const geometry_msgs::Twist& msg)
{
        last_cmd_vel = nh.now();

        //	debug_printf("x: %f  z: %f\r\n", msg.linear.x, msg.angular.z);

        // calculate twist speeds to add/substract
        float left_twist_mps = -1.0 * msg.angular.z * WHEEL_BASE * 0.5;
        float right_twist_mps =  msg.angular.z * WHEEL_BASE * 0.5;


        // add them to the linear speed
        float left_mps = msg.linear.x + left_twist_mps;
        float right_mps = msg.linear.x + right_twist_mps;

        // cap left motor speed to MAX_MPS
        if (left_mps > MAX_MPS)
        {
            left_mps = MAX_MPS;
        }
        else if (left_mps < -1*MAX_MPS)
        {
            left_mps = -1*MAX_MPS;
        }
        // cap right motor speed to MAX_MPS
        if (right_mps > MAX_MPS)
        {
            right_mps = MAX_MPS;
        }
        else if (right_mps < -1*MAX_MPS)
        {
            right_mps = -1*MAX_MPS;
        }

        // set directions
        left_dir = (left_mps >= 0)?1:0;
        right_dir = (right_mps >= 0)?1:0;

        // set drivemotors PWM values
        left_speed = abs(left_mps * PWM_PER_MPS);
        right_speed = abs(right_mps * PWM_PER_MPS);

    //	debug_printf("left_mps: %f (%c)  right_mps: %f (%c)\r\n", left_mps, left_dir?'F':'R', right_mps, right_dir?'F':'R');
}

extern "C" void cdc_receive_put(uint8_t value)
{
    ringbuffer_putchar(&rb, value);
}


/*
 * Update various chatters topics
 */
extern "C" void chatter_handler()
{
      if (NBT_handler(&publish_nbt))
      {
          /*
          char version[] = "version: 0.1";
          str_msg.data = version;
          chatter.publish(&str_msg);
          */
#if defined OPENMOWER_INTERFACE
#else
          f32_battery_voltage_msg.data = battery_voltage;
          pubBatteryVoltage.publish(&f32_battery_voltage_msg);

          f32_charge_voltage_msg.data = charge_voltage;
          pubChargeVoltage.publish(&f32_charge_voltage_msg);

          f32_charge_current_msg.data = charge_current;
          pubChargeCurrent.publish(&f32_charge_current_msg);

          int16_charge_pwm_msg.data = chargecontrol_pwm_val;
          pubChargePWM.publish(&int16_charge_pwm_msg);

          bool_charging_state_msg.data =  chargecontrol_is_charging;
          pubChargeingState.publish(&bool_charging_state_msg);
#endif

          //bool_blade_state_msg.data = true; // TODO: read blade status
//		  pubBladeState.publish(&bool_blade_state_msg);

          HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);         // flash LED
      }
}

/*
 *  Drive Motors handler
 *  Blade Motor handler
 */
extern "C" void motors_handler()
{
      if (NBT_handler(&motors_nbt))
      {
        if (is_emergency())
        {
            setDriveMotors(0,0,0,0);
            setBladeMotor(0);
        }
        else {
            last_cmd_vel_age = nh.now().sec - last_cmd_vel.sec;
            if (last_cmd_vel_age > 1) {
                setDriveMotors(0, 0, left_dir, right_dir);
            }
            else {
                setDriveMotors(left_speed, right_speed, left_dir, right_dir);
            }
            if (last_cmd_vel_age > 25) {
                setBladeMotor(0);
            }
            else {
                setBladeMotor(blade_on_off);
            }
        }
      }
}

/*
 *  Keyboard/LED Panel handler
 */
extern "C" void panel_handler()
{
      if (NBT_handler(&panel_nbt))
      {
        PANEL_Tick();
      }
}

extern "C" void broadcast_handler()
{
      if (NBT_handler(&broadcast_nbt))
      {
#if defined OPENMOWER_INTERFACE
        status_msg.stamp = nh.now();
        status_msg.mower_status = mower_msgs::Status::MOWER_STATUS_OK;
        status_msg.raspberry_pi_power = 1;
        status_msg.gps_power = 1;
        status_msg.esc_power = 1;
        status_msg.emergency = is_emergency();
        status_msg.v_battery = battery_voltage;
        status_msg.v_charge = charge_voltage;
        status_msg.charge_current = charge_current;

        auto l_status = mower_msgs::ESCStatus();
        if(left_wheel_speed_val > 0x05) {
                l_status.status = mower_msgs::ESCStatus::ESC_STATUS_RUNNING;
        }
        else {
                l_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
        }
        l_status.tacho = left_encoder_ticks;
        l_status.temperature_motor = 30;
        l_status.temperature_pcb = 30;

        auto r_status = mower_msgs::ESCStatus();
        if(right_wheel_speed_val > 0x05) {
                r_status.status = mower_msgs::ESCStatus::ESC_STATUS_RUNNING;
        }
        else {
                r_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
        }
        r_status.tacho = right_encoder_ticks;
        r_status.temperature_motor = 30;
        r_status.temperature_pcb = 30;

        auto m_status = mower_msgs::ESCStatus();
        if(blade_on_off) {
                m_status.status = mower_msgs::ESCStatus::ESC_STATUS_RUNNING;
        }
        else {
                m_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
        }
        m_status.temperature_motor = 30;
        m_status.temperature_pcb = 30;

        status_msg.left_esc_status = l_status;
        status_msg.right_esc_status = r_status;
        status_msg.mow_esc_status = m_status;
        pubStatus.publish(&status_msg);


#else
        // z = BROADCAST_NBT_TIME_MS/1000;
        // x = right_wheel_speed_val;
        // y = left_wheel_speed_val;
        current_time = nh.now();
        //////////////////////////////////////////////////
        // TF message
        //////////////////////////////////////////////////
        speed_act_left = left_wheel_speed_val/PWM_PER_MPS;			// wheel speed in m/s
        speed_act_right = right_wheel_speed_val/PWM_PER_MPS;			// wheel speed in m/s
        // debug_printf("speed_act_left: %f speed_act_right: %f\r\n",  speed_act_left, speed_act_right);
        dt = (BROADCAST_NBT_TIME_MS/1000.0);
        dxy = (speed_act_left+speed_act_right)*dt/2.0;
        dth = - 1.0 * ((speed_act_right-speed_act_left)*dt)/WHEEL_BASE;

        //debug_printf("dt: %f dxy: %f dth: %f\r\n",  dt, dxy, dth);

        if (dth > 0) dth *= angular_scale_positive;
        if (dth < 0) dth *= angular_scale_negative;
        if (dxy > 0) dxy *= linear_scale_positive;
        if (dxy < 0) dxy *= linear_scale_negative;

        dx = cos(dth) * dxy;
        dy = sin(dth) * dxy;

        x_pos += (cos(theta) * dx - sin(theta) * dy);
        y_pos += (sin(theta) * dx + cos(theta) * dy);
        theta += dth;

        if(theta >= two_pi) theta -= two_pi;
        if(theta <= -two_pi) theta += two_pi;

        quat = tf::createQuaternionFromYaw(theta);
        if(publish_tf) {
            geometry_msgs::TransformStamped t;
            t.header.frame_id = odom;
            t.child_frame_id = base_link;
            t.transform.translation.x = x_pos;
            t.transform.translation.y = y_pos;
            t.transform.translation.z = 0.0;
            t.transform.rotation = quat;
            t.header.stamp = current_time;
            broadcaster.sendTransform(t);
        }
        //////////////////////////////////////////////////
        // odom message
        //////////////////////////////////////////////////
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = odom;
        odom_msg.pose.pose.position.x = x_pos;
        odom_msg.pose.pose.position.y = y_pos;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = quat;
        if (speed_act_left == 0 && speed_act_right == 0){
            odom_msg.pose.covariance[0] = 1e-9;
            odom_msg.pose.covariance[7] = 1e-3;
            odom_msg.pose.covariance[8] = 1e-9;
            odom_msg.pose.covariance[14] = 1e6;
            odom_msg.pose.covariance[21] = 1e6;
            odom_msg.pose.covariance[28] = 1e6;
            odom_msg.pose.covariance[35] = 1e-9;
            odom_msg.twist.covariance[0] = 1e-9;
            odom_msg.twist.covariance[7] = 1e-3;
            odom_msg.twist.covariance[8] = 1e-9;
            odom_msg.twist.covariance[14] = 1e6;
            odom_msg.twist.covariance[21] = 1e6;
            odom_msg.twist.covariance[28] = 1e6;
            odom_msg.twist.covariance[35] = 1e-9;
        }
        else{
            odom_msg.pose.covariance[0] = 1e-3;
            odom_msg.pose.covariance[7] = 1e-3;
            odom_msg.pose.covariance[8] = 0.0;
            odom_msg.pose.covariance[14] = 1e6;
            odom_msg.pose.covariance[21] = 1e6;
            odom_msg.pose.covariance[28] = 1e6;
            odom_msg.pose.covariance[35] = 1e3;
            odom_msg.twist.covariance[0] = 1e-3;
            odom_msg.twist.covariance[7] = 1e-3;
            odom_msg.twist.covariance[8] = 0.0;
            odom_msg.twist.covariance[14] = 1e6;
            odom_msg.twist.covariance[21] = 1e6;
            odom_msg.twist.covariance[28] = 1e6;
            odom_msg.twist.covariance[35] = 1e3;
        }
        vx = (dt == 0)?  0 : (speed_act_left+speed_act_right)/2.0;
    //	vth = (dt == 0)? 0 : (speed_act_right-speed_act_left)/WHEEL_BASE;
        odom_msg.child_frame_id = base_link;
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.angular.z = dth;
        pubOdom.publish(&odom_msg);

        // pub encoder values as well
        left_encoder_ticks_msg.data = left_encoder_ticks;
        pubLeftEncoderTicks.publish(&left_encoder_ticks_msg);
        right_encoder_ticks_msg.data = right_encoder_ticks;
        pubRightEncoderTicks.publish(&right_encoder_ticks_msg);
/*
        double dx = 0.2;
        double dtheta = 0.18;

        x += cos(theta)*dx*0.1;
        y += sin(theta)*dx*0.1;
        theta += dtheta*0.1;
        if (theta > 3.14)
            theta = -3.14;
        t.header.frame_id = odom;
        t.child_frame_id = base_link;
        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.rotation = tf::createQuaternionFromYaw(theta);
        t.header.stamp = nh.now();

        broadcaster.sendTransform(t);
*/
#endif

#if defined SOFT_I2C_ENABLED && SOFT_I2C_ENABLED == 1
        ////////////////////////////////////////
        // IMU
        ////////////////////////////////////////
        float imu_x,imu_y,imu_z;

        imu_msg.header.frame_id = "imu";
        imu_msg.header.stamp = current_time;
         // Linear acceleration
        IMU_ReadAccelerometer(&imu_x, &imu_y, &imu_z);
        imu_msg.linear_acceleration.x = imu_x;
        imu_msg.linear_acceleration.y = imu_y;
        imu_msg.linear_acceleration.z = imu_z;
        /*
        imu_msg.linear_acceleration_covariance[0] = 1e-3;
        imu_msg.linear_acceleration_covariance[4] = 1e-3;
        imu_msg.linear_acceleration_covariance[8] = 1e-3;
        */
        // Angular velocity
        IMU_ReadGyro(&imu_x, &imu_y, &imu_z);
        imu_msg.angular_velocity.x = imu_x;
        imu_msg.angular_velocity.y = imu_y;
        imu_msg.angular_velocity.z = imu_z;
        /*
        imu_msg.angular_velocity_covariance[0] = 1e-3;
        imu_msg.angular_velocity_covariance[4] = 1e-3;
        imu_msg.angular_velocity_covariance[8] = 1e-3;
        */
        pubIMU.publish(&imu_msg);

        // Orientation (Magnetometer)
        imu_mag_msg.header.frame_id = "imu";
        imu_mag_msg.header.stamp = current_time;
        IMU_ReadMagnetometer(&imu_x, &imu_y, &imu_z);
        imu_mag_msg.magnetic_field.x = imu_x;
        imu_mag_msg.magnetic_field.y = imu_y;
        imu_mag_msg.magnetic_field.z = imu_z;
        pubIMUMag.publish(&imu_mag_msg);

        // Calibration (Magnetometer)
        imu_mag_msg.header.frame_id = "imu";
        imu_mag_msg.header.stamp = current_time;
        IMU_ReadMagnetometerRaw(&imu_x, &imu_y, &imu_z);
        imu_mag_calibration_msg.magnetic_field.x = imu_x;
        imu_mag_calibration_msg.magnetic_field.y = imu_y;
        imu_mag_calibration_msg.magnetic_field.z = imu_z;
        pubIMUMagCalibration.publish(&imu_mag_calibration_msg); // this is what ros-calibration_imu expects


        //imu_mag_calibration_msg.x = imu_x;
        //imu_mag_calibration_msg.y = imu_y;
        //imu_mag_calibration_msg.z = imu_z;
        //pubIMUMagCalibration.publish(&imu_mag_calibration_msg); // this is what ros-calibration_imu expects
#endif
      }
}

/*
 * ROS housekeeping
 */
extern "C" void spinOnce()
{
      if (NBT_handler(&ros_nbt))
      {
            nh.spinOnce();
      }
}

/*
 *  Initialize rosserial
 */
extern "C" void init_ROS()
{
    ringbuffer_init(&rb, RxBuffer, RxBufferSize);

    // Initialize ROS
    nh.initNode();

    // Initialize TF Broadcaster
    broadcaster.init(nh);

    // Initialize Pubs
#if defined OPENMOWER_INTERFACE
    nh.advertise(pubStatus);
#else
    nh.advertise(pubBatteryVoltage);
    nh.advertise(pubChargeVoltage);
    nh.advertise(pubChargeCurrent);
    nh.advertise(pubChargePWM);
    nh.advertise(pubOdom);
    nh.advertise(pubBladeState);
    nh.advertise(pubChargeingState);
    nh.advertise(pubLeftEncoderTicks);
    nh.advertise(pubRightEncoderTicks);
#endif

#if defined SOFT_I2C_ENABLED && SOFT_I2C_ENABLED == 1
    nh.advertise(pubIMU);
    nh.advertise(pubIMUMag);
    nh.advertise(pubIMUMagCalibration);
#endif

    // Initialize Subs
    nh.subscribe(subCommandVelocity);
#if defined OPENMOWER_INTERFACE
#else
    nh.subscribe(subBladeOn);
    nh.subscribe(subBladeOff);
#endif
	// Initialize Services
#if defined OPENMOWER_INTERFACE
	nh.advertiseService(srvSetMowEnable);
	nh.advertiseService(srvSetEmergencyStop);
#else
#endif

    // Initialize Timers
    NBT_init(&publish_nbt, 1000);
    NBT_init(&panel_nbt, 100);
    NBT_init(&motors_nbt, BROADCAST_NBT_TIME_MS);
    NBT_init(&broadcast_nbt, BROADCAST_NBT_TIME_MS);
    NBT_init(&ros_nbt, 10);
}
