#include <TFmini.h>
#include <boost/thread.hpp>

ros::Time ts_now;
ros::Time ts_pre;
double ts_diff;

void errorCheck(int rate, double timeout_sec, std::string id, ros::Publisher pub_range) {
  sensor_msgs::Range TFmini_range;
  TFmini_range.radiation_type = sensor_msgs::Range::INFRARED;
  TFmini_range.field_of_view = 0.04;
  TFmini_range.min_range = 0.3;
  TFmini_range.max_range = 12;
  TFmini_range.header.frame_id = id;

  float dist = 0;
  ros::Rate r(rate);

  while (ros::ok())
  {
    ts_now = ros::Time::now();
    ts_diff = ts_now.toSec()-ts_pre.toSec();
    // printf("ts_diff: %lf\n", ts_diff);
    if(timeout_sec < ts_diff)
    {
      // printf("timeout ts_diff: %lf\n", ts_diff);
      ts_pre = ts_now;

      TFmini_range.range = -2.0;
      TFmini_range.header.stamp = ros::Time::now();
      pub_range.publish(TFmini_range);
    }

    r.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tfmini");
  ros::NodeHandle nh("~");
  std::string id;
  std::string portName;
  double timeout_sec;
  int main_hz;
  int baud_rate;
  float dist = -3.0;
  benewake::TFmini *tfmini_obj;

  // nh.param("serial_port", portName, std::string("/dev/ttyUSB0"));
  // nh.param("baud_rate", baud_rate, 115200);
  ros::param::get("~serial_port", portName);
  ros::param::get("~baud_rate", baud_rate);
  ros::param::get("~frame_id", id);
  ros::param::get("~timeout_sec", timeout_sec);
  ros::param::get("~main_hz", main_hz);

  ros::Publisher pub_range = nh.advertise<sensor_msgs::Range>(id, 1000, true);
  sensor_msgs::Range TFmini_range;
  TFmini_range.radiation_type = sensor_msgs::Range::INFRARED;
  TFmini_range.field_of_view = 0.04;
  TFmini_range.min_range = 0.3;
  TFmini_range.max_range = 12;
  TFmini_range.header.frame_id = id;
  TFmini_range.range = dist;
  TFmini_range.header.stamp = ros::Time::now();
  pub_range.publish(TFmini_range);
  ROS_INFO_STREAM("Start processing ...");

  tfmini_obj = new benewake::TFmini(portName, baud_rate);

  ros::Rate r(main_hz);

  ts_pre = ts_now = ros::Time::now();
  boost::thread threadErrorCheck(errorCheck, main_hz, timeout_sec, id, pub_range);

  while(ros::master::check() && ros::ok())
  {
    dist = tfmini_obj->getDist();

    // static int count = 0;
    // printf("dist : %f mm, %d cnt\r\n", dist, count++);

    #if 0
    if(dist > 0 && dist < TFmini_range.max_range)
    {
      TFmini_range.range = dist;
      TFmini_range.header.stamp = ros::Time::now();
      pub_range.publish(TFmini_range);
    }
    else if(dist == -1.0)
    {
      printf("Failed to read data. TFmini ros node stopped!\n");
      // ROS_ERROR_STREAM("Failed to read data. TFmini ros node stopped!");
      break;
    }
    else if(dist == 0.0)
    {
      printf("Data validation error!\n");
      // ROS_ERROR_STREAM("Data validation error!");
    }
    #else
    if(dist > 0 && dist < TFmini_range.max_range)
    {
      ts_pre = ts_now;
      TFmini_range.range = dist;
    }
    else
    {
      ts_pre = ts_now;
      TFmini_range.range = -1.0;
    }
    TFmini_range.header.stamp = ros::Time::now();
    pub_range.publish(TFmini_range);
    #endif

    ros::spinOnce();
    r.sleep();
  }

  TFmini_range.range = -4.0;
  TFmini_range.header.stamp = ros::Time::now();
  pub_range.publish(TFmini_range);

  threadErrorCheck.join();
  tfmini_obj->closePort();
}
