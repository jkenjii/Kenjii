#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <signal.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include "ros/ros.h"

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#define TIME_STEP 32
#define NMOTORS 4
#define MAX_SPEED 6.4
#define OBSTACLE_THRESHOLD 0.1
#define DECREASE_FACTOR 0.9
#define BACK_SLOWDOWN 0.9

ros::NodeHandle *n;

static std::vector<float> lidarValues;

static int controllerCount;
static std::vector<std::string> controllerList;

ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;

static const char *motorNames[NMOTORS] = {"front_left_wheel", "front_right_wheel", "back_left_wheel", "back_right_wheel"};

static double GPSValues[3] = {0, 0, 0};
static double inertialUnitValues[4] = {0, 0, 0, 0};

static int lms291Resolution = 0;
static int halfResolution = 0;
static double maxRange = 0.0;
static double rangeThreshold = 0.0;
static std::vector<double> braitenbergCoefficients;
static bool areBraitenbergCoefficientsinitialized = false;

// gaussian 
double gaussian(double x, double mu, double sigma) {
  return (1.0 / (sigma * sqrt(2.0 * M_PI))) * exp(-((x - mu) * (x - mu)) / (2 * sigma * sigma));
}

void updateSpeed() {
  
  double leftObstacle = 0.0, rightObstacle = 0.0, obstacle = 0.0;
  double speeds[NMOTORS];
  
  for (int i = 0; i < halfResolution; ++i) {
    if (lidarValues[i] < rangeThreshold)  // obstaculos longe são ignorados
      leftObstacle += braitenbergCoefficients[i] * (1.0 - lidarValues[i] / maxRange);
    // near obstacle sensed on the right side
    int j = lms291Resolution - i - 1;
    if (lidarValues[j] < rangeThreshold)
      rightObstacle += braitenbergCoefficients[i] * (1.0 - lidarValues[j] / maxRange);
  }
  
  obstacle = leftObstacle + rightObstacle;
  // velocidade de acordo com os obstaculos a frente  
  if (obstacle > OBSTACLE_THRESHOLD) {
    const double speedFactor = (1.0 - DECREASE_FACTOR * obstacle) * MAX_SPEED / obstacle;
    speeds[0] = speedFactor * leftObstacle;
    speeds[1] = speedFactor * rightObstacle;
    speeds[2] = BACK_SLOWDOWN * speeds[0];
    speeds[3] = BACK_SLOWDOWN * speeds[1];
  } else {
    speeds[0] = MAX_SPEED;
    speeds[1] = MAX_SPEED;
    speeds[2] = MAX_SPEED;
    speeds[3] = MAX_SPEED;
  }
  // speeds
  for (int i = 0; i < NMOTORS; ++i) {
    ros::ServiceClient set_velocity_client;
    webots_ros::set_float set_velocity_srv;
    set_velocity_client = n->serviceClient<webots_ros::set_float>(std::string("pioneer3at/") + std::string(motorNames[i]) +
                                                                  std::string("/set_velocity"));
    set_velocity_srv.request.value = speeds[i];
    set_velocity_client.call(set_velocity_srv);
  }
}

void broadcastTransform() {
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(-GPSValues[2], GPSValues[0], GPSValues[1]));
  tf::Quaternion q(inertialUnitValues[0], inertialUnitValues[1], inertialUnitValues[2], inertialUnitValues[3]);
  q = q.inverse();
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
  transform.setIdentity();
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "pioneer3at/Sick_LMS_291"));
}

void GPSCallback(const sensor_msgs::NavSatFix::ConstPtr &values) {
  GPSValues[0] = values->latitude;
  GPSValues[1] = values->altitude;
  GPSValues[2] = values->longitude;
  broadcastTransform();
}

void inertialUnitCallback(const sensor_msgs::Imu::ConstPtr &values) {
  inertialUnitValues[0] = values->orientation.x;
  inertialUnitValues[1] = values->orientation.y;
  inertialUnitValues[2] = values->orientation.z;
  inertialUnitValues[3] = values->orientation.w;
  broadcastTransform();
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr &scan) {
  int scanSize = scan->ranges.size();
  lidarValues.resize(scanSize);
  for (int i = 0; i < scanSize; ++i)
    lidarValues[i] = scan->ranges[i];

  lms291Resolution = scanSize;
  halfResolution = scanSize / 2;
  maxRange = scan->range_max;
  rangeThreshold = maxRange / 20.0;
  if (!areBraitenbergCoefficientsinitialized) {
    braitenbergCoefficients.resize(lms291Resolution);
    for (int i = 0; i < lms291Resolution; ++i)
      braitenbergCoefficients[i] = gaussian(i, halfResolution, lms291Resolution / 5);
    areBraitenbergCoefficientsinitialized = true;
  }

  updateSpeed();
}

void controllerNameCallback(const std_msgs::String::ConstPtr &name) {
  controllerCount++;
  controllerList.push_back(name->data);
}

void quit(int sig) {
  ROS_INFO("User stopped the 'pioneer3at' node.");
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  std::string controllerName;
  // criação do node pioneer3at
  ros::init(argc, argv, "pioneer3at", ros::init_options::AnonymousName);
  n = new ros::NodeHandle;

  signal(SIGINT, quit);

  // subscreve o node ROS controller 
  ros::Subscriber nameSub = n->subscribe("model_name", 100, controllerNameCallback);
  while (controllerCount == 0 || controllerCount < nameSub.getNumPublishers()) {
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();
  }
  ros::spinOnce();

  timeStepClient = n->serviceClient<webots_ros::set_int>("pioneer3at/robot/time_step");
  timeStepSrv.request.value = TIME_STEP;
  } 
  nameSub.shutdown();

  // motores inicia
  for (int i = 0; i < NMOTORS; ++i) {
    // posição
    ros::ServiceClient set_position_client;
    webots_ros::set_float set_position_srv;
    set_position_client = n->serviceClient<webots_ros::set_float>(std::string("pioneer3at/") + std::string(motorNames[i]) +
                                                                  std::string("/set_position"));

    set_position_srv.request.value = INFINITY;

    // speed
    ros::ServiceClient set_velocity_client;
    webots_ros::set_float set_velocity_srv;
    set_velocity_client = n->serviceClient<webots_ros::set_float>(std::string("pioneer3at/") + std::string(motorNames[i]) +
                                                                  std::string("/set_velocity"));

    set_velocity_srv.request.value = 0.0;
  }

  // ativa lidar
  ros::ServiceClient set_lidar_client;
  webots_ros::set_int lidar_srv;
  ros::Subscriber sub_lidar_scan;

  set_lidar_client = n->serviceClient<webots_ros::set_int>("pioneer3at/Sick_LMS_291/enable");
  lidar_srv.request.value = TIME_STEP;
  if (set_lidar_client.call(lidar_srv) && lidar_srv.response.success) 
    sub_lidar_scan = n->subscribe("pioneer3at/Sick_LMS_291/laser_scan/layer0", 10, lidarCallback);
   
  // enable gps
  ros::ServiceClient set_GPS_client;
  webots_ros::set_int GPS_srv;
  ros::Subscriber sub_GPS;
  set_GPS_client = n->serviceClient<webots_ros::set_int>("pioneer3at/gps/enable");
  GPS_srv.request.value = 32;
  if (set_GPS_client.call(GPS_srv) && GPS_srv.response.success) 
    sub_GPS = n->subscribe("pioneer3at/gps/values", 1, GPSCallback);
    
  // ativa inertial unit
  ros::ServiceClient set_inertial_unit_client;
  webots_ros::set_int inertial_unit_srv;
  ros::Subscriber sub_inertial_unit;
  set_inertial_unit_client = n->serviceClient<webots_ros::set_int>("pioneer3at/inertial_unit/enable");
  inertial_unit_srv.request.value = 32;
  if (set_inertial_unit_client.call(inertial_unit_srv) && inertial_unit_srv.response.success) 
    sub_inertial_unit = n->subscribe("pioneer3at/inertial_unit/roll_pitch_yaw", 1, inertialUnitCallback);
    
  // main loop
  while (ros::ok()) {
    if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success) {
      ROS_ERROR("Failed to call service time_step for next step.");
      break;
    }
    ros::spinOnce();
  }
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);

  ros::shutdown();
  return 0;
}
