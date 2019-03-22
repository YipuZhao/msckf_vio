/*
* COPYRIGHT AND PERMISSION NOTICE
* Penn Software MSCKF_VIO
* Copyright (C) 2017 The Trustees of the University of Pennsylvania
* All rights reserved.
*/

#ifndef MSCKF_VIO_H
#define MSCKF_VIO_H

#include <map>
#include <set>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>

#include <fstream>


#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "imu_state.h"
#include "cam_state.h"
#include "feature.hpp"
#include <msckf_vio/CameraMeasurement.h>


// #define LOGGING_LMK_LIFE


namespace msckf_vio {
/*
* @brief MsckfVio Implements the algorithm in
*    Anatasios I. Mourikis, and Stergios I. Roumeliotis,
*    "A Multi-State Constraint Kalman Filter for Vision-aided
*    Inertial Navigation",
*    http://www.ee.ucr.edu/~mourikis/tech_reports/TR_MSCKF.pdf
*/


class vioTimeLog {
public:
  vioTimeLog(const double &timeStamp_, const double &timeCost_) {
    time_stamp = timeStamp_;
    time_cost = timeCost_;
  };

  vioTimeLog(const double &timeStamp_, const double &timeCost_, const size_t &numStates_) {
    time_stamp = timeStamp_;
    time_cost = timeCost_;
    num_states = numStates_;
  };
  
  double time_stamp;
  double time_cost;
  size_t num_states;
};


class trackLog {
public:
  trackLog(const double &timeStamp_, const Eigen::Vector3d &position_, const Eigen::Vector4d &orientation_) {
    time_stamp = timeStamp_;
    position = position_;
    orientation = orientation_;
  };

  double time_stamp;
    // Orientation
  // Take a vector from the world frame to
  // the IMU (body) frame.
  Eigen::Vector4d orientation;

  // Position of the IMU (body) frame
  // in the world frame.
  Eigen::Vector3d position;
};


class lmkLog {
public:
  lmkLog(const int &id_, const int &life_) {
    id = id_;
    life = life_;
  };
  
  int id;
  int life;
};


class MsckfVio {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructor
    MsckfVio(ros::NodeHandle& pnh);
    // Disable copy and assign constructor
    MsckfVio(const MsckfVio&) = delete;
    MsckfVio operator=(const MsckfVio&) = delete;

    // Destructor
    ~MsckfVio() {
      // try call log saving?
    // saveTimeLog("/home/yipuzhao/catkin_ws/tmpLog_back.txt");
    saveLmkLog("/mnt/DATA/msckf_tmpLog_lmk.txt");
    // saveTimeLog("/home/yipuzhao/catkin_ws/tmpLog_back.txt");
    saveVIOTimeLog("/mnt/DATA/msckf_tmpLog_back.txt");
      // also save the real time track
    // saveAllFrameTrack("/home/yipuzhao/catkin_ws/tmpTrack.txt");
    saveAllFrameTrack("/mnt/DATA/msckf_tmpTrack.txt");
    }

    /*
    * @brief initialize Initialize the VIO.
    */
    bool initialize();

    /*
    * @brief reset Resets the VIO to initial status.
    */
    void reset();

    typedef boost::shared_ptr<MsckfVio> Ptr;
    typedef boost::shared_ptr<const MsckfVio> ConstPtr;

  private:
    /*
    * @brief StateServer Store one IMU states and several
    *    camera states for constructing measurement
    *    model.
    */
    struct StateServer {
      IMUState imu_state;
      CamStateServer cam_states;

      // State covariance matrix
      Eigen::MatrixXd state_cov;
      Eigen::Matrix<double, 12, 12> continuous_noise_cov;
    };

    
    
    // save the lmk life of msckf
    std::vector<lmkLog> logLmk;
    
    void saveLmkLog(const std::string &filename) {

    std::cout << std::endl << "Saving " << this->logLmk.size() << " records to lmk log file " << filename << " ..." << std::endl;

    std::ofstream fLmkLog;
    fLmkLog.open(filename.c_str());
    fLmkLog << std::fixed;
    fLmkLog << "#id life" << std::endl;
    for(size_t i=0; i<this->logLmk.size(); i++)
    {
	fLmkLog << std::setprecision(0)
		      << this->logLmk[i].id << " "
		      << this->logLmk[i].life << std::endl;
    }
    fLmkLog.close();

    std::cout << "Finished saving log! " << std::endl;
}
    
    // save the time cost of msckf
    std::vector<vioTimeLog> logTimeCost;
  
  void saveVIOTimeLog(const std::string &filename) {

    std::cout << std::endl << "Saving " << this->logTimeCost.size() << " VIO records to time log file " << filename << " ..." << std::endl;

    std::ofstream fFrameTimeLog;
    fFrameTimeLog.open(filename.c_str());
    fFrameTimeLog << std::fixed;
    fFrameTimeLog << "#frame_time_stamp time_proc num_states" << std::endl;
    for(size_t i=0; i<this->logTimeCost.size(); i++)
    {
	fFrameTimeLog << std::setprecision(6)
		      << this->logTimeCost[i].time_stamp << " "
		      << this->logTimeCost[i].time_cost  << " "
		      << std::setprecision(0)
		      << this->logTimeCost[i].num_states << std::endl;
    }
    fFrameTimeLog.close();

    std::cout << "Finished saving log! " << std::endl;
}


//
    std::vector<trackLog> logFramePose;
    
    
void saveAllFrameTrack(const std::string &filename) {
  
  std::cout << std::endl << "Saving " << this->logFramePose.size() << " records to track file " << filename << " ..." << std::endl;

    std::ofstream f_realTimeTrack;
    f_realTimeTrack.open(filename.c_str());
    f_realTimeTrack << std::fixed;
    f_realTimeTrack << "#TimeStamp Tx Ty Tz Qx Qy Qz Qw" << std::endl;
    for(size_t i=0; i<this->logFramePose.size(); i++)
    {
      f_realTimeTrack << std::setprecision(6)
			<< this->logFramePose[i].time_stamp << " "
			<< std::setprecision(7)
			<< this->logFramePose[i].position.transpose() << " "
			<< this->logFramePose[i].orientation.transpose() << std::endl;
    }
    f_realTimeTrack.close();

    std::cout << "Finished saving track! " << std::endl;
}



    /*
    * @brief loadParameters
    *    Load parameters from the parameter server.
    */
    bool loadParameters();

    /*
    * @brief createRosIO
    *    Create ros publisher and subscirbers.
    */
    bool createRosIO();

    /*
    * @brief imuCallback
    *    Callback function for the imu message.
    * @param msg IMU msg.
    */
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);

    /*
    * @brief featureCallback
    *    Callback function for feature measurements.
    * @param msg Stereo feature measurements.
    */
    void featureCallback(const CameraMeasurementConstPtr& msg);

    /*
    * @brief publish Publish the results of VIO.
    * @param time The time stamp of output msgs.
    */
    void publish(const ros::Time& time);

    /*
    * @brief initializegravityAndBias
    *    Initialize the IMU bias and initial orientation
    *    based on the first few IMU readings.
    */
    void initializeGravityAndBias();

    /*
    * @biref resetCallback
    *    Callback function for the reset service.
    *    Note that this is NOT anytime-reset. This function should
    *    only be called before the sensor suite starts moving.
    *    e.g. while the robot is still on the ground.
    */
    bool resetCallback(std_srvs::Trigger::Request& req,
	std_srvs::Trigger::Response& res);

    // Filter related functions
    // Propogate the state
    void batchImuProcessing(
	const double& time_bound);
    void processModel(const double& time,
	const Eigen::Vector3d& m_gyro,
	const Eigen::Vector3d& m_acc);
    void predictNewState(const double& dt,
	const Eigen::Vector3d& gyro,
	const Eigen::Vector3d& acc);

    // Measurement update
    void stateAugmentation(const double& time);
    void addFeatureObservations(const CameraMeasurementConstPtr& msg);
    // This function is used to compute the measurement Jacobian
    // for a single feature observed at a single camera frame.
    void measurementJacobian(const StateIDType& cam_state_id,
	const FeatureIDType& feature_id,
	Eigen::Matrix<double, 4, 6>& H_x,
	Eigen::Matrix<double, 4, 3>& H_f,
	Eigen::Vector4d& r);
    // This function computes the Jacobian of all measurements viewed
    // in the given camera states of this feature.
    void featureJacobian(const FeatureIDType& feature_id,
	const std::vector<StateIDType>& cam_state_ids,
	Eigen::MatrixXd& H_x, Eigen::VectorXd& r);
    void measurementUpdate(const Eigen::MatrixXd& H,
	const Eigen::VectorXd& r);
    bool gatingTest(const Eigen::MatrixXd& H,
	const Eigen::VectorXd&r, const int& dof);
    void removeLostFeatures();
    void findRedundantCamStates(
	std::vector<StateIDType>& rm_cam_state_ids);
    void pruneCamStateBuffer();
    // Reset the system online if the uncertainty is too large.
    void onlineReset();

    // Chi squared test table.
    static std::map<int, double> chi_squared_test_table;

    // State vector
    StateServer state_server;
    // Maximum number of camera states
    int max_cam_state_size;

    // Features used
    MapServer map_server;

    // IMU data buffer
    // This is buffer is used to handle the unsynchronization or
    // transfer delay between IMU and Image messages.
    std::vector<sensor_msgs::Imu> imu_msg_buffer;

    // Indicate if the gravity vector is set.
    bool is_gravity_set;

    // Indicate if the received image is the first one. The
    // system will start after receiving the first image.
    bool is_first_img;

    // The position uncertainty threshold is used to determine
    // when to reset the system online. Otherwise, the ever-
    // increaseing uncertainty will make the estimation unstable.
    // Note this online reset will be some dead-reckoning.
    // Set this threshold to nonpositive to disable online reset.
    double position_std_threshold;

    // Tracking rate
    double tracking_rate;

    // Threshold for determine keyframes
    double translation_threshold;
    double rotation_threshold;
    double tracking_rate_threshold;

    // Ros node handle
    ros::NodeHandle nh;

    // Subscribers and publishers
    ros::Subscriber imu_sub;
    ros::Subscriber feature_sub;
    ros::Publisher msf_odom_pub;
    ros::Publisher odom_pub;
    ros::Publisher feature_pub;
    tf::TransformBroadcaster tf_pub;
    ros::ServiceServer reset_srv;

    // Frame id
    std::string fixed_frame_id;
    std::string child_frame_id;

    // Whether to publish tf or not.
    bool publish_tf;

    // Framte rate of the stereo images. This variable is
    // only used to determine the timing threshold of
    // each iteration of the filter.
    double frame_rate;

    // Debugging variables and functions
    void mocapOdomCallback(
	const nav_msgs::OdometryConstPtr& msg);

    ros::Subscriber mocap_odom_sub;
    ros::Publisher mocap_odom_pub;
    geometry_msgs::TransformStamped raw_mocap_odom_msg;
    Eigen::Isometry3d mocap_initial_frame;
};

typedef MsckfVio::Ptr MsckfVioPtr;
typedef MsckfVio::ConstPtr MsckfVioConstPtr;

} // namespace msckf_vio

#endif
