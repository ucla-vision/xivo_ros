#include <string>
#include "xivo_ros/simple_node.h"
#include "glog/logging.h"
#include "opencv2/highgui/highgui.hpp"

#include "xivo/FeatureData.h"
#include "xivo/FeatureMap.h"
#include "xivo/FullState.h"
#include "xivo/MotionState2dNav.h"

#include "utils.h"

namespace xivo
{

ros::Time xivoTimestamp_to_rosTime(timestamp_t ts) {
  int nsec_total = ts.count();
  int nsec = nsec_total % 1000000000UL;
  int sec = nsec_total / 1000000000UL;
  ros::Time ros_ts(sec, nsec);
  return ros_ts;
}



template<class T> // T is either geometry_msgs/Vector3 or geometry_msgs/Point
void copy_vec3_to_ros(T &ros_dest, const Vec3 &vector)
{
  ros_dest.x = vector(0);
  ros_dest.y = vector(1);
  ros_dest.z = vector(2);
}


void copy_rot_to_ros(geometry_msgs::Quaternion &ros_quat, const SO3 &xivo_rot)
{
  Mat3 R = xivo_rot.matrix();
  Quat q(R);
  ros_quat.x = q.x();
  ros_quat.y = q.y();
  ros_quat.z = q.z();
  ros_quat.w = q.w();
}


template<class T, unsigned long l>
void copy_full_square_mat_to_ros(boost::array<double, l> &rosarr, T &matrix, int dim)
{
  int count = 0;
  for (int i=0; i<dim; i++) {
    for (int j=0; j<dim; j++) {
      rosarr[count] = matrix(i,j);
      count++;
    }
  }
}

template<class T, unsigned long l>
void copy_upper_triangle_to_ros(boost::array<double, l> &rosarr, T matrix, int dim)
{
  int count = 0;
  for (int i=0; i<dim; i++) {
    for (int j=i; j<dim; j++) {
      rosarr[count] = matrix(i,j);
      count++;
    }
  }
}


void ROSPublisherAdapter::Publish(const timestamp_t &ts, const cv::Mat &disp) {
  if (!disp.empty()) {
    DLOG(INFO) << "Display image is ready";
    cv_bridge::CvImage cv_msg;
    // cv_msg.header.stamp    = msg->header.stamp;
    cv_msg.header.frame_id = "image";
    cv_msg.header.stamp = xivoTimestamp_to_rosTime(ts);
    cv_msg.encoding        = disp.channels() > 1 ? "bgr8" : "mono8";
    cv_msg.image           = disp;
    sensor_msgs::Image ros_msg;
    cv_msg.toImageMsg(ros_msg);
    rospub_.publish(ros_msg);
    cv_msg.image.release();
  }
}


void ROSEgoMotionPublisherAdapter::Publish(const timestamp_t &ts,
  const SE3 &gsb, const Mat6 &cov)
{
  geometry_msgs::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "Robot State";
  msg.header.stamp = xivoTimestamp_to_rosTime(ts);

  copy_vec3_to_ros(msg.pose.pose.position, gsb.translation());
  // Quaternion in pose is inverse of quaternion in transformation
  copy_rot_to_ros(msg.pose.pose.orientation, gsb.rotation().inv());

  // Row-major covariance
  copy_full_square_mat_to_ros(msg.pose.covariance, cov, 6);

  rospub_.publish(msg);
}


void ROSMapPublisherAdapter::Publish(const timestamp_t &ts, const int npts,
  const Eigen::Matrix<number_t, Eigen::Dynamic, 3> &InstateXs,
  const Eigen::Matrix<number_t, Eigen::Dynamic, 6> &InstateCov,
  const Eigen::Matrix<number_t, Eigen::Dynamic, 2> &InstatePixels,
  const VecXi &feature_ids)
{
  FeatureMap msg;
  msg.header.frame_id = "Feature Map";
  msg.header.stamp = xivoTimestamp_to_rosTime(ts);
  msg.num_features = npts;

  for (int i=0; i<npts; i++) {
    FeatureData f;
    f.id = feature_ids(i);

    copy_vec3_to_ros(f.Xs, InstateXs.block<1,3>(i,0));

    f.covariance[0] = InstateCov(i,0);
    f.covariance[1] = InstateCov(i,1);
    f.covariance[2] = InstateCov(i,2);

    f.covariance[3] = InstateCov(i,1);
    f.covariance[4] = InstateCov(i,3);
    f.covariance[5] = InstateCov(i,4);

    f.covariance[6] = InstateCov(i,2);
    f.covariance[7] = InstateCov(i,4);
    f.covariance[8] = InstateCov(i,5);

    f.col_coord_px = InstatePixels(i,0);
    f.row_coord_px = InstatePixels(i,1);

    msg.features.push_back(f);
  }

  rospub_.publish(msg);
}


void ROSFullStatePublisherAdapter::Publish(const timestamp_t &ts,
  const State &X, const Mat3 &Ca, const Mat3 &Cg, const MatX &Cov,
  const bool MeasurementsInitialized, const Vec3 &inn_Wsb,
  const Vec3 &inn_Tsb, const Vec3 &inn_Vsb, const int gauge_group,
  const SE3 &gsc, const MatX &CameraCov)
{
  FullState msg;
  msg.header.frame_id = "full state";
  msg.header.stamp = xivoTimestamp_to_rosTime(ts);

  copy_vec3_to_ros(msg.gsb.translation, X.Tsb);
  copy_rot_to_ros(msg.gsb.rotation, X.Rsb);
  copy_vec3_to_ros(msg.Vsb, X.Vsb);
  copy_vec3_to_ros(msg.gbc.translation, X.Tbc);
  copy_rot_to_ros(msg.gbc.rotation, X.Rbc);
  copy_vec3_to_ros(msg.bg, X.bg);
  copy_vec3_to_ros(msg.ba, X.ba);
  copy_rot_to_ros(msg.qg, X.Rg);

  msg.td = X.td;

  copy_full_square_mat_to_ros(msg.Cg, Cg, 3);
  copy_full_square_mat_to_ros(msg.Ca, Ca, 3);

  for (int i=0; i<kMotionSize; i++) {
    for (int j=i; j<kMotionSize; j++) {
      msg.covariance.emplace_back(Cov(i,j));
    }
  }

  copy_vec3_to_ros(msg.gsc.translation, gsc.translation());
  copy_rot_to_ros(msg.gsc.rotation, gsc.rotation());

  msg.MeasurementUpdateInitialized = MeasurementsInitialized;
  msg.group = gauge_group;

  msg.MotionStateSize = kMotionSize;

  copy_vec3_to_ros(msg.inn_Wsb, inn_Wsb); 
  copy_vec3_to_ros(msg.inn_Tsb, inn_Tsb);
  copy_vec3_to_ros(msg.inn_Vsb, inn_Vsb);

  CameraManager *cameraptr = Camera::instance();
  Vec9 intrinsics = cameraptr->GetIntrinsics();

  // Camera distortion_model
  std::string distortion_model;
  int num_params;
  switch (cameraptr->GetDistortionType()) {
    case DistortionType::ATAN:
      distortion_model = "atan";
      num_params = 7;
      break;
    case DistortionType::EQUI:
      distortion_model = "equi";
      num_params = 8;
      break;
    case DistortionType::PINHOLE:
      distortion_model = "pinhole";
      num_params = 4;
      break;
    case DistortionType::RADTAN:
      distortion_model = "radtan";
      num_params = 9;
      break;
    default:
      LOG(ERROR) << "ROS node: Invalid camera type";
  }

  msg.num_camera_params = num_params;
  msg.distortion_model = distortion_model;
  for (int i=0; i<num_params; i++) {
    msg.camera_intrinsics[i] = intrinsics[i];
  }

  for (int i=0; i<num_params; i++) {
    for (int j=i; j<num_params; j++) {
      msg.camera_covariance.emplace_back(CameraCov(i,j));
    }
  }

  // Projection model
  msg.projection_model = "pinhole";


  rospub_.publish(msg);
}


void ROS2dNavPublisherAdapter::Publish(const timestamp_t &ts, const SE3 &gsb,
  const Vec3 &Vsb, const SO3 &Rg, const MatX &Cov)
{
  MotionState2dNav msg;
  msg.header.frame_id = "2d nav state";
  msg.header.stamp = xivoTimestamp_to_rosTime(ts);

  // For clarification
  SO3 Rsg = Rg;
  SO3 Rgs = Rsg.inv();
  SO3 Rgb = Rgs * gsb.R();

  Vec3 Xs = gsb.T();
  Mat3 Rsb = gsb.R().matrix();
  Mat3 posCov_sb = Cov.block<3,3>(Index::Tsb,Index::Tsb);
  Mat3 velCov_sb = Cov.block<3,3>(Index::Vsb,Index::Vsb);

  Vec3 Xg = Rgs * Xs;
  Vec3 Vgb = Rgs * Vsb; // I think this holds because g frame and s frame are
                        // permanently co-located
  
  Vec3 euler_angles = Rgb.matrix().eulerAngles(0, 1, 2);

  Mat3 posCov_gb = Rgs.matrix() * posCov_sb * Rsg.matrix();
  Mat3 velCov_gb = Rgs.matrix() * velCov_sb * Rsg.matrix();

  copy_vec3_to_ros(msg.position, Xg);
  copy_vec3_to_ros(msg.velocity, Vgb);
  copy_full_square_mat_to_ros(msg.position_covariance, posCov_gb, 3);
  copy_full_square_mat_to_ros(msg.velocity_covariance, velCov_gb, 3);
  copy_vec3_to_ros(msg.euler_angles, euler_angles);

  rospub_.publish(msg);
}


SimpleNode::~SimpleNode() {
  if (est_proc_) {
    est_proc_->Wait();
    est_proc_.reset();
  }
  if (adapter_) {
    adapter_.reset();
  }
  if (viewer_) {
    viewer_->Wait();
    viewer_.reset();
  }
}

SimpleNode::SimpleNode(): adapter_{nullptr}, viewer_{nullptr}, viz_{false}
{
  // parse node parameters
  ros::NodeHandle nh_priv("~");

  // get configuration path & create estimator process
  std::string cfg_path;
  int estimator_queue_size;

  nh_priv.param("config_path", cfg_path, cfg_path);
  LOG(INFO) << "Found configuration @ " << cfg_path;

  // get image and imu topics
  std::string imu_topic, image_topic;
  nh_priv.param("imu_topic", imu_topic, imu_topic);
  nh_priv.param("image_topic", image_topic, image_topic);

  // subscribe to topics
  imu_sub_ = nh_.subscribe(imu_topic, 1000, &SimpleNode::ImuMsgCallback, this);
  LOG(INFO) << "Subscribed to IMU topic: " << imu_topic;

  image_sub_ = nh_.subscribe(image_topic, 1000, &SimpleNode::ImageMsgCallback, this);
  LOG(INFO) << "Subscribed to image topic: " << image_topic;

  // estimator process
  nh_priv.param("estimator_queue_size", estimator_queue_size, estimator_queue_size);
  LOG(INFO) << "Size of estimator process queue = " << estimator_queue_size;

  est_proc_ = std::unique_ptr<EstimatorProcess>(
      new EstimatorProcess("Estimator", estimator_queue_size));
  est_proc_->Initialize(cfg_path);

  std::string viewer_type{"ros"};
  nh_priv.param("viewer_type", viewer_type, viewer_type);
  if (viewer_type.empty()) {
    LOG(WARNING) << "viewer type not set!";
  } else {
    viz_ = true;
    if (viewer_type == "ros") {
      // publisher
      viz_pub_ = nh_.advertise<sensor_msgs::Image>("TrackerView", 1);
      adapter_ = std::unique_ptr<ROSPublisherAdapter>(
          new ROSPublisherAdapter(viz_pub_));
      // set publisher to estimator process
      est_proc_->SetPublisher(adapter_.get());
    } else if (viewer_type == "pangolin") {
      std::string viewer_cfg_path;
      nh_priv.param("viewer_config_path", viewer_cfg_path, viewer_cfg_path);
      auto viewer_cfg = LoadJson(viewer_cfg_path);
      viewer_ = std::unique_ptr<ViewPublisher>(
          new ViewPublisher(viewer_cfg));
      // start the view-publisher thread
      viewer_->Start();
      // set publisher to estimator process
      est_proc_->SetPublisher(viewer_.get());
    } else {
      LOG(FATAL) << "unknown viewer type; only support ros & pangolin (all lower case)";
    }
  }

  int max_features_to_publish;
  nh_priv.param("publish_state", publish_egomotion_, false);
  nh_priv.param("publish_full_state", publish_full_state_, false);
  nh_priv.param("publish_2dnav_state", publish_2dnav_state_, false);
  nh_priv.param("publish_map", publish_map_, false);
  nh_priv.param("max_features_to_publish", max_features_to_publish, 100);
  if (publish_egomotion_) {
    ego_motion_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "xivo/pose", 1000);
    ego_motion_adapter_ = std::unique_ptr<ROSEgoMotionPublisherAdapter>(
      new ROSEgoMotionPublisherAdapter(ego_motion_pub_));
    est_proc_->SetPosePublisher(ego_motion_adapter_.get());
  }
  if (publish_map_) {
    map_pub_ = nh_.advertise<xivo::FeatureMap>("xivo/map", 1000);
    map_adapter_ = std::unique_ptr<ROSMapPublisherAdapter>(
      new ROSMapPublisherAdapter(map_pub_));
    est_proc_->SetMapPublisher(map_adapter_.get(), max_features_to_publish);
  }
  if (publish_full_state_) {
    full_state_pub_ = nh_.advertise<xivo::FullState>("xivo/fullstate", 1000);
    full_state_adapter_ = std::unique_ptr<ROSFullStatePublisherAdapter>(
      new ROSFullStatePublisherAdapter(full_state_pub_));
    est_proc_->SetFullStatePublisher(full_state_adapter_.get());
  }
  if (publish_2dnav_state_) {
    twod_nav_pub_ = nh_.advertise<xivo::MotionState2dNav>("xivo/twod_nav", 1000);
    twod_nav_adapter_ = std::unique_ptr<ROS2dNavPublisherAdapter>(
      new ROS2dNavPublisherAdapter(twod_nav_pub_));
    est_proc_->Set2dNavStatePublisher(twod_nav_adapter_.get());
  }


  // start the estimator thread
  est_proc_->Start();
}

void SimpleNode::ImuMsgCallback(sensor_msgs::ImuConstPtr msg)
{
  Vec3 gyro{msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
  Vec3 accel{msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
  timestamp_t ts{msg->header.stamp.toNSec()};

  est_proc_->Enqueue(std::move(std::make_unique<InertialMeas>(ts, gyro, accel, viz_)));
}

void SimpleNode::ImageMsgCallback(sensor_msgs::ImageConstPtr msg)
{
  timestamp_t ts{msg->header.stamp.toNSec()};
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat image;
  if (msg->encoding == sensor_msgs::image_encodings::MONO8) {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    cv_ptr->image.copyTo(image);
  } else if (msg->encoding == sensor_msgs::image_encodings::MONO16) {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
    cv_ptr->image.convertTo(image, CV_8UC1, 1.0/256);
  } else if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    cv::cvtColor(cv_ptr->image, image, CV_RGB2GRAY);
  } else if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::cvtColor(cv_ptr->image, image, CV_BGR2GRAY);
  } else {
    LOG(FATAL) << "unexpected image encoding";
  }

  est_proc_->Enqueue(std::move(std::make_unique<VisualMeas>(ts, image, viz_)));
}

} // namespace xivo

