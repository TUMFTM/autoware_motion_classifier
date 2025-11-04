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

#ifndef AUTOWARE__MULTI_OBJECT_TRACKER__UNCERTAINTY__OBJECT_MOTION_CLASSIFIER_HPP_
#define AUTOWARE__MULTI_OBJECT_TRACKER__UNCERTAINTY__OBJECT_MOTION_CLASSIFIER_HPP_

#include "autoware/multi_object_tracker/tracker/model/tracker_base.hpp"

#include <rclcpp/rclcpp.hpp>

#include "autoware_perception_msgs/msg/detected_objects.hpp"
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <deque>
#include <fstream>
#include <list>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace autoware::multi_object_tracker
{
namespace uncertainty
{

/**
 * @brief A class for determining the motion state of tracked objects based on
 * on their position, rotation and velocity together with model predicted uncertainty
 * using a z-test.
 */
class ObstacleMotionClassifier
{
public:
  /**
   * @brief Constructor for the ObstacleMotionClassifier.
   * @param observation_window_size The size of the observation window.
   * @param z_score_thresholds Map of object class labels to their z-score thresholds for rejecting
   * the movement hypothesis.
   * @param variance_threshold The lower bound for all variance values.
   * @param minimum_window_size The minimum windows size for the z-test to be performed.
   * @param enable_logging Whether to enable z-score logging to file.
   */
  explicit ObstacleMotionClassifier(
    const std::size_t observation_window_size = 10,
    const std::map<std::uint8_t, double> & z_score_thresholds = {},
    const double variance_threshold = 1E-5, const std::size_t minimum_window_size = 2,
    const bool enable_logging = false);

  /**
   * @brief Structure storing only the neccesarry information for each observed object
   */
  struct ObjectObservation
  {
    /// Pointer to the object tracker
    std::shared_ptr<Tracker> tracker;
    /// Predicted object state
    double x_pos;
    double y_pos;
    double yaw;
    double velocity_x;
    double velocity_y;
    /// Predicted parameter covariance
    double x_pos_uncertainty;
    double y_pos_uncertainty;
    double velocity_x_uncertainty;
    double velocity_y_uncertainty;
  };

  /**
   * @brief Determins if the newly and tracked obstacles are static.
   * If their velocity or position is statistically static, sets the tracked object's velocity to 0.
   * @param time_ Measurement time.
   * @param tracker_list A list of current trackers to be updated.
   * @param objects_message A list of tracked objects to be tested.
   */
  void determine_object_motion_state(
    const rclcpp::Time & time_, const std::list<std::shared_ptr<Tracker>> & tracker_list,
    autoware_perception_msgs::msg::TrackedObjects & objects_message);

  /**
   * @brief Adds detected objects into observation windows based on reverse assignment matching.
   * @param time_ Measurement time.
   * @param detected_objects A list of detected objects received from the object merger.
   * @param reverse_asignment An assigment map mapping detected object indices to their tracker
   * index.
   * @param tracker_list A list of trackers to retrieve tracked objects from.
   */
  void update_observations(
    const rclcpp::Time & time_,
    const autoware_perception_msgs::msg::DetectedObjects & detected_objects,
    const std::unordered_map<int, int> & reverse_asignment,
    const std::list<std::shared_ptr<Tracker>> & tracker_list);

  std::ofstream zscore_log_file_;

private:
  using ObservationWindow = std::deque<ObjectObservation>;
  using ObservationWindowMap =
    std::unordered_map<std::string, ObservationWindow>;  // Tracker UUID to observation window

  /**
   * @brief Performs a Z-test on the velocity magnitude history to determine motion state.
   * @param observation_window The deque of motion observations for a single track.
   * @return Array of 4 flags if the regression parameters x_pos, y_pos, vel_x
   * and vel_y set to true if the obstacle is statistically static and an array of the mean
   * positions.
   */
  std::pair<std::array<double, 4>, std::array<double, 2>> perform_movement_z_test(
    const ObservationWindow & observation_window);

  /**
   * @brief Converts a UUID to a string.
   * @param u The UUID to convert.
   * @return The string representation of the UUID.
   */
  std::string uuid_to_string(const unique_identifier_msgs::msg::UUID & u)
  {
    std::stringstream ss;
    for (auto i = 0; i < 16; ++i) {
      ss << std::hex << std::setfill('0') << std::setw(2) << +u.uuid[i];
    }
    return ss.str();
  }

  // Member variables
  ObservationWindowMap
    observation_window_cache_;          // A map of tracker UUIDs to their observation windows
  std::size_t observation_window_size;  // Number of observations to keep in the queue
  std::map<std::uint8_t, double>
    z_score_thresholds;             // Map of object class labels to z-score thresholds
  double variance_threshold;        // Variance lower bound
  std::size_t minimum_window_size;  // Minimum window size for z-test to be performed
  bool enable_logging_;             // Whether to enable z-score logging
  std::unordered_map<int, std::string> processed_detections_;  // Set of processed detecitons
};

}  // namespace uncertainty
}  // namespace autoware::multi_object_tracker

#endif  // AUTOWARE__MULTI_OBJECT_TRACKER__UNCERTAINTY__OBJECT_MOTION_CLASSIFIER_HPP_
