// Copyright 2025 TUM.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "autoware/multi_object_tracker/uncertainty/object_motion_classifier.hpp"

#include <rclcpp/logging.hpp>

#include <cmath>
#include <limits>
#include <list>
#include <map>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{
namespace uncertainty
{

ObstacleMotionClassifier::ObstacleMotionClassifier(
  const std::size_t observation_window_size,
  const std::map<std::uint8_t, double> & z_score_thresholds, const double variance_threshold,
  const std::size_t minimum_window_size, const bool enable_logging)
: observation_window_size{observation_window_size},
  z_score_thresholds{z_score_thresholds},
  variance_threshold{variance_threshold},
  minimum_window_size{minimum_window_size},
  enable_logging_{enable_logging}
{
  if (enable_logging_) {
    zscore_log_file_.open("/log/zscores.log", std::ios::app);
    if (!zscore_log_file_.is_open()) {
      RCLCPP_WARN(
        rclcpp::get_logger("multi_object_tracker"),
        "Failed to open z-score log file: /log/zscores.log");
    }
  }
}

void ObstacleMotionClassifier::update_observations(
  const rclcpp::Time & time_,
  const autoware_perception_msgs::msg::DetectedObjects & detected_objects,
  const std::unordered_map<int, int> & reverse_asignment,
  const std::list<std::shared_ptr<Tracker>> & tracker_list)
{
  using autoware::universe_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
  for (size_t i = 0; i < detected_objects.objects.size(); ++i) {
    auto detected_object{detected_objects.objects.at(i)};
    auto tracker_index{reverse_asignment.find(i)};
    if (
      tracker_index == reverse_asignment.end() || tracker_index->second < 0 ||
      processed_detections_.find(i) != processed_detections_.end()) {
      continue;
    }
    auto tracker_itr{std::next(tracker_list.begin(), tracker_index->second)};
    autoware_perception_msgs::msg::TrackedObject tracked_object;
    (*(tracker_itr))->getTrackedObject(time_, tracked_object);
    // Add to observation window
    ObjectObservation observation;
    observation.tracker = *(tracker_itr);
    observation.x_pos = detected_object.kinematics.pose_with_covariance.pose.position.x;
    observation.y_pos = detected_object.kinematics.pose_with_covariance.pose.position.y;
    observation.yaw = tf2::getYaw(detected_object.kinematics.pose_with_covariance.pose.orientation);
    observation.velocity_x = detected_object.kinematics.twist_with_covariance.twist.linear.x;
    observation.velocity_y = detected_object.kinematics.twist_with_covariance.twist.linear.y;
    observation.x_pos_uncertainty = std::max(
      this->variance_threshold,
      detected_object.kinematics.pose_with_covariance.covariance[XYZRPY_COV_IDX::X_X]);
    observation.y_pos_uncertainty = std::max(
      this->variance_threshold,
      detected_object.kinematics.pose_with_covariance.covariance[XYZRPY_COV_IDX::Y_Y]);
    observation.velocity_x_uncertainty = std::max(
      this->variance_threshold,
      detected_object.kinematics.twist_with_covariance.covariance[XYZRPY_COV_IDX::X_X]);
    observation.velocity_y_uncertainty = std::max(
      this->variance_threshold,
      detected_object.kinematics.twist_with_covariance.covariance[XYZRPY_COV_IDX::Y_Y]);
    // Find relevant observation in observation window based on tracked obstacle id
    std::string uuid_key{this->uuid_to_string(tracked_object.object_id)};
    processed_detections_.insert(std::make_pair(i, uuid_key));
    auto observation_window{observation_window_cache_.find(uuid_key)};
    if (observation_window == observation_window_cache_.end()) {
      // Create a new observation window if it doesn't exist
      ObservationWindow new_observation_window;
      new_observation_window.push_back(observation);
      observation_window_cache_.emplace(uuid_key, new_observation_window);
    } else {
      // Add the observation to the existing window
      observation_window->second.push_back(observation);
      // Remove oldest observation if window size exceeds the limit
      if (observation_window->second.size() > observation_window_size) {
        observation_window->second.pop_front();
      }
    }
  }
}

void ObstacleMotionClassifier::determine_object_motion_state(
  const rclcpp::Time & time_, const std::list<std::shared_ptr<Tracker>> & tracker_list,
  autoware_perception_msgs::msg::TrackedObjects & objects_message)
{
  for (auto & tracked_object : objects_message.objects) {
    std::string uuid{uuid_to_string(tracked_object.object_id)};
    auto observation_window{observation_window_cache_.find(uuid)};
    if (
      observation_window == observation_window_cache_.end() ||
      observation_window->second.size() < this->minimum_window_size) {
      continue;
    }
    uint8_t label =
      autoware::object_recognition_utils::getHighestProbLabel(tracked_object.classification);

    // Get the z-score threshold for this object class
    double class_z_score_threshold = 1.96;  // default value
    auto threshold_it = z_score_thresholds.find(label);
    if (threshold_it != z_score_thresholds.end()) {
      class_z_score_threshold = threshold_it->second;
    }

    auto z_scores{perform_movement_z_test(observation_window->second)};
    // Log z-scores to file
    if (enable_logging_ && zscore_log_file_.is_open()) {
      zscore_log_file_ << "label: " << static_cast<int>(label) << ", z_scores: ";
      zscore_log_file_ << z_scores.first.at(0) << ", ";
      zscore_log_file_ << z_scores.first.at(1) << ", ";
      zscore_log_file_ << "speed: "
                       << tracked_object.kinematics.twist_with_covariance.twist.linear.x << ", ";
      zscore_log_file_ << tracked_object.kinematics.twist_with_covariance.twist.linear.y << ", ";
      zscore_log_file_ << std::endl;
    }

    // Identifiy static objects, only based on position variance
    if ((z_scores.first.at(0) < class_z_score_threshold &&
         z_scores.first.at(1) < class_z_score_threshold)) {
      // Calculate the mean of x_pos, y_pos, and yaw over the observation_window
      double mean_x = 0.0;
      double mean_y = 0.0;
      double mean_yaw = 0.0;
      for (const auto & observation : observation_window->second) {
        mean_x += observation.x_pos;
        mean_y += observation.y_pos;
        mean_yaw += observation.yaw;
      }
      mean_x /= observation_window->second.size();
      mean_y /= observation_window->second.size();
      mean_yaw /= observation_window->second.size();
      ObjectObservation & last_observation{observation_window->second.back()};
      last_observation.tracker->setStaticState(mean_x, mean_y);
    }
  }

  autoware_perception_msgs::msg::TrackedObject tracked_object;
  // Collect tracked object uuids
  std::set<std::string> tracked_object_uuids;
  for (auto tracker : tracker_list) {
    tracker->getTrackedObject(time_, tracked_object);
    std::string uuid_key{this->uuid_to_string(tracked_object.object_id)};
    tracked_object_uuids.insert(uuid_key);
  }

  // Remove observation windows for tracked objects that are no longer present
  for (auto object_itr = observation_window_cache_.begin();
       object_itr != observation_window_cache_.end();) {
    if (tracked_object_uuids.find(object_itr->first) == tracked_object_uuids.end()) {
      object_itr = observation_window_cache_.erase(object_itr);
    } else {
      ++object_itr;
    }
  }
  processed_detections_.clear();
}

std::pair<std::array<double, 4>, std::array<double, 2>>
ObstacleMotionClassifier::perform_movement_z_test(const ObservationWindow & observation_window)
{
  std::uint32_t num_observations{static_cast<std::uint32_t>(observation_window.size())};
  std::uint32_t n{static_cast<std::uint32_t>(std::ceil(num_observations / 2))};
  std::uint32_t m{num_observations - n};
  // Mean and uncertainty values for the two sample z-test (x, y)
  std::array<double, 2> left_mean{};
  std::array<double, 2> right_mean{};
  std::array<double, 2> left_uncertainty{};
  std::array<double, 2> right_uncertainty{};
  // Mean and uncertainty values for the one sample z-test (vel_x, vel_y)
  std::array<double, 2> velocity_mean{};
  std::array<double, 2> velocity_uncertainty{};
  // Results of the z-test
  // (x_pos, y_pos, vel_x, vel_y)
  std::array<double, 4> z_test_results{};

  auto observation_window_it{observation_window.begin()};
  for (std::uint32_t i = 0; i < num_observations; ++i) {
    std::array<double, 2> & mean = (i < n) ? left_mean : right_mean;
    std::array<double, 2> & uncertainty = (i < n) ? left_uncertainty : right_uncertainty;

    mean.at(0) += observation_window_it->x_pos;
    mean.at(1) += observation_window_it->y_pos;
    uncertainty.at(0) += observation_window_it->x_pos_uncertainty;
    uncertainty.at(1) += observation_window_it->y_pos_uncertainty;

    velocity_mean.at(0) += observation_window_it->velocity_x;
    velocity_mean.at(1) += observation_window_it->velocity_y;
    velocity_uncertainty.at(0) += observation_window_it->velocity_x_uncertainty;
    velocity_uncertainty.at(1) += observation_window_it->velocity_y_uncertainty;
    ++observation_window_it;
  }
  for (std::size_t i = 0; i < left_mean.size(); ++i) {
    left_mean.at(i) /= n;
    right_mean.at(i) /= m;
    left_uncertainty.at(i) /= n * n;
    right_uncertainty.at(i) /= m * m;
    double z_score{std::abs(left_mean.at(i) - right_mean.at(i))};
    z_score /= std::sqrt(left_uncertainty.at(i) + right_uncertainty.at(i));
    z_test_results.at(i) = z_score;
  }

  for (std::size_t i = 0; i < velocity_mean.size(); ++i) {
    velocity_mean.at(i) /= num_observations;
    velocity_uncertainty.at(i) /= num_observations * num_observations;
    double z_score{std::abs(velocity_mean.at(i))};
    z_score /= std::sqrt(velocity_uncertainty.at(i));
    z_test_results.at(i + 2) = z_score;
  }
  return std::make_pair(z_test_results, right_mean);
}

}  // namespace uncertainty
}  // namespace autoware::multi_object_tracker
