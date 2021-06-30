#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <dvs_msgs/EventArray.h>
#include <opencv2/core/core.hpp>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <dvs_integrator_conv/dvs_integrator_convConfig.h>

namespace dvs_integrator_conv
{

class Integrator {
public:
  Integrator(ros::NodeHandle & nh, ros::NodeHandle nh_private);
  virtual ~Integrator();

private:
  ros::NodeHandle nh_;

  // Callback functions
  void eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg);

  // Subscribers
  ros::Subscriber event_sub_;

  // Publishers
  image_transport::Publisher image_pub_;
  image_transport::Publisher time_map_pub_;
  cv::Mat state_time_map_;  // Time of the last event at each pixel
  cv::Mat state_image_;     // Brightness value at each pixel
  void publishState();

  // Dynamic reconfigure
  void reconfigureCallback(dvs_integrator_conv::dvs_integrator_convConfig &config, uint32_t level);
  boost::shared_ptr<dynamic_reconfigure::Server<dvs_integrator_conv::dvs_integrator_convConfig> > server_;
  dynamic_reconfigure::Server<dvs_integrator_conv::dvs_integrator_convConfig>::CallbackType dynamic_reconfigure_callback_;

  double alpha_cutoff_; // cut-off frequency, leaky parameter
  double c_pos_; // contrast threshold for positive events
  double c_neg_; // contrast threshold for negative events
  double t_last_; // time of the last event, used for plotting / publishing
  int convolution_mask_; // index of the spatial convolution mask

  // Convolution kernels
  void setKernel(cv::Mat& ker);
  const cv::Mat sobel_x_ = (cv::Mat_<double>(3,3) << 1, 0, -1, 2, 0, -2, 1, 0, -1) / 8.; // already flipped
  const cv::Mat sobel_y_ = (cv::Mat_<double>(3,3) << 1, 2, 1, 0, 0, 0, -1, -2, -1) / 8.; // already flipped
  const cv::Mat laplace_ = (cv::Mat_<double>(3,3) << 1, 4, 1, 4, -20, 4, 1, 4, 1) / 6.;  // Matlab fspecial function with default alpha = 0.2
  const cv::Mat blur_gauss_ = (cv::Mat_<double>(3,3) << 0.01134, 0.08381, 0.01134, 0.08381, 0.6194, 0.08381, 0.01134, 0.08381, 0.01134); // sigma = 0.5

  void minMaxLocRobust(const cv::Mat& image, double& min, double& max,
                       const double& percentage_pixels_to_discard);
  void normalize(const cv::Mat& src, cv::Mat& dst, const double& percentage_pixels_to_discard);
};

} // namespace
