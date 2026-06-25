#include <ros_estimators/car_estimator/car_estimator.h>

#include <pacejka_model/pacejka_car_input.h>
#include <pacejka_model/pacejka_car_state.h>
#include <pacejka_model/pacejka_discrete.h>
#include <ros_crs_utils/parameter_io.h>
#include <ros_crs_utils/state_message_conversion.h>
#include <ros/ros.h>
#include <mh_estimator/pacejka_mhe.h>

#include <tf/transform_broadcaster.h>

namespace ros_estimators
{
template <>
void RosCarEstimator<crs_models::pacejka_model::pacejka_car_state, crs_models::pacejka_model::pacejka_car_input,
                     parameter_io::empty_params>::publishState()
{
  if (has_valid_input_ && model_based_estimator && (base_estimator->getLastValidTs() > 0))
  {
    // Predict up until publishing time if estimator supports it.
    model_based_estimator->controlInputCallback(model_based_estimator->getLastInput(), ros::Time::now().toSec());
  }

  crs_msgs::car_state_cart msg =
      message_conversion::convertStateToRosMsg<crs_msgs::car_state_cart, crs_models::pacejka_model::pacejka_car_state,
                                               crs_models::pacejka_model::pacejka_car_input>(
          base_estimator->getStateEstimate(ros::Time::now().toSec()), {});

  // Convert to pose and publish with ros using tf
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(msg.x, msg.y, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, msg.yaw);
  transform.setRotation(q);
  auto now = ros::Time::now();
  if (last_publish_time != now)
  {
    tf_broadcaster.sendTransform(tf::StampedTransform(transform, last_publish_time, "crs_frame", car_frame_name));
    last_publish_time = now;
  }
  else
  {
    // Skip publishing if we are trying to publish at the same time
  }

  std::shared_ptr<crs_estimators::mhe::Pacejka_MHE> mhe_estimator =
      std::dynamic_pointer_cast<crs_estimators::mhe::Pacejka_MHE>(base_estimator);

  // Publish references and esimate if an mhe estimator is used.
  if (mhe_estimator)
  {
    visualization_msgs::Marker marker;
    marker.color.r = 1;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 1;
    marker.header.frame_id = "crs_frame";
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;

    int id = 0;
    for (const Eigen::Vector3d meas : mhe_estimator->mocap_reference)
    {
      marker.pose.position.x = meas[0];
      marker.pose.position.y = meas[1];
      marker.pose.position.z = 0;
      marker.id = id;
      id++;

      // Reference orientation (quaternion from only yaw)
      marker.pose.orientation.w = std::cos(meas[2] * 0.5);
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = std::sin(meas[2] * 0.5);
      marker.header.stamp = ros::Time::now();
      reference_pub.publish(marker);
    }

    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    id = 0;
    for (const Eigen::Vector3d est : mhe_estimator->last_solution.trajectory_estimate_)
    {
      marker.pose.position.x = est[0];
      marker.pose.position.y = est[1];
      marker.pose.position.z = 0;
      marker.id = id;
      id++;

      // Reference orientation (quaternion from only yaw)
      marker.pose.orientation.w = std::cos(est[2] * 0.5);
      marker.pose.orientation.x = 0;
      marker.pose.orientation.y = 0;
      marker.pose.orientation.z = std::sin(est[2] * 0.5);
      marker.header.stamp = ros::Time::now();
      // publish as estimate
      estimate_pub.publish(marker);
    }
  }

  if (base_estimator->getLastValidTs() > 0)
    msg.header.stamp = ros::Time(base_estimator->getLastValidTs());
  state_estimate_pub_.publish(msg);

  publishDiagnosticData();
};  // namespace ros_estimators

}  // namespace ros_estimators
