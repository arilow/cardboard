/*
 * Copyright 2019 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "head_tracker.h"

#include "sensors/neck_model.h"
#include "sensors/pose_prediction.h"
#include "util/logging.h"
#include "util/vector.h"
#include "util/vectorutils.h"
#include "include/cardboard.h"

namespace cardboard {

namespace {

  const Rotation ekf_to_head_tracker_LandscapeLeft =
      Rotation::FromYawPitchRoll(-M_PI / 2., 0, -M_PI / 2.);
  const Rotation sensor_to_display_LandscapeLeft =
      Rotation::FromAxisAndAngle(Vector3(0, 0, 1), M_PI / 2.);

  const Rotation ekf_to_head_tracker_LandscapeRight =
      Rotation::FromYawPitchRoll(M_PI / 2., 0, M_PI / 2.);
  const Rotation sensor_to_display_LandscapeRight =
      Rotation::FromAxisAndAngle(Vector3(0, 0, 1), -M_PI / 2.);

  const Rotation ekf_to_head_tracker_Portrait =
      Rotation::FromYawPitchRoll(M_PI / 2., M_PI / 2., M_PI / 2.);
  const Rotation sensor_to_display_Portrait =
      Rotation::FromAxisAndAngle(Vector3(0, 0, 1), 0.);

  const Rotation ekf_to_head_tracker_PortraitUpsideDown =
      Rotation::FromYawPitchRoll(-M_PI / 2., -M_PI / 2., -M_PI / 2.);
  const Rotation sensor_to_display_PortraitUpsideDown =
      Rotation::FromAxisAndAngle(Vector3(0, 0, 1), M_PI);

  constexpr void GetViewporOrientationRotation(
      const CardboardViewportOrientation ViewportOrientation,
      Rotation& ekf_to_head_tracker,
      Rotation& sensor_to_display) {
    switch (ViewportOrientation) {
      case CardboardViewportOrientation_LandscapeLeft:
        ekf_to_head_tracker = ekf_to_head_tracker_LandscapeLeft;
        sensor_to_display = sensor_to_display_LandscapeLeft;
        break;

      case CardboardViewportOrientation_LandscapeRight:
        ekf_to_head_tracker = ekf_to_head_tracker_LandscapeRight;
        sensor_to_display = sensor_to_display_LandscapeRight;
        break;

      case CardboardViewportOrientation_Portrait:
        ekf_to_head_tracker = ekf_to_head_tracker_Portrait;
        sensor_to_display = sensor_to_display_Portrait;
        break;

      default: // PortraitUpsideDown
        ekf_to_head_tracker = ekf_to_head_tracker_PortraitUpsideDown;
        sensor_to_display = sensor_to_display_PortraitUpsideDown;
        break;
    }
  }

}  // namespaces

HeadTracker::HeadTracker()
    : is_tracking_(false),
      sensor_fusion_(new SensorFusionEkf()),
      latest_gyroscope_data_({0, 0, Vector3::Zero()}),
      accel_sensor_(new SensorEventProducer<AccelerometerData>()),
      gyro_sensor_(new SensorEventProducer<GyroscopeData>()) {
  sensor_fusion_->SetBiasEstimationEnabled(/*kGyroBiasEstimationEnabled*/ true);
  on_accel_callback_ = [&](const AccelerometerData& event) {
    OnAccelerometerData(event);
  };
  on_gyro_callback_ = [&](const GyroscopeData& event) {
    OnGyroscopeData(event);
  };
}

HeadTracker::~HeadTracker() { UnregisterCallbacks(); }

void HeadTracker::Pause() {
  if (!is_tracking_) {
    return;
  }

  UnregisterCallbacks();

  // Create a gyro event with zero velocity. This effectively stops the
  // prediction.
  GyroscopeData event = latest_gyroscope_data_;
  event.data = Vector3::Zero();

  OnGyroscopeData(event);

  is_tracking_ = false;
}

void HeadTracker::Resume() {
  is_tracking_ = true;
  RegisterCallbacks();
}

void HeadTracker::GetPose(int64_t timestamp_ns,
                          std::array<float, 3>& out_position,
                          std::array<float, 4>& out_orientation) const {
  Rotation predicted_rotation;
  const PoseState pose_state = sensor_fusion_->GetLatestPoseState();
  if (!sensor_fusion_->IsFullyInitialized()) {
    CARDBOARD_LOGI(
        "Head Tracker not fully initialized yet. Using pose prediction only.");
    predicted_rotation = pose_prediction::PredictPose(timestamp_ns, pose_state);
  } else {
    predicted_rotation = pose_state.sensor_from_start_rotation;
  }

  // In order to update our pose as the sensor changes, we begin with the
  // inverse default orientation (the orientation returned by a reset sensor),
  // apply the current sensor transformation, and then transform into display
  // space.
  // TODO(b/135488467): Support different screen orientations.
//  CardboardViewportOrientation aux = CardboardViewportOrientation_LandscapeLeft;
   CardboardViewportOrientation aux = CardboardViewportOrientation_LandscapeRight;
  //CardboardViewportOrientation aux = CardboardViewportOrientation_PortraitUpsideDown;

  Rotation ekf_to_head_tracker;
  Rotation sensor_to_display;


  GetViewporOrientationRotation(
                  aux,
                  ekf_to_head_tracker, 
                  sensor_to_display);

  // switch(aux) {
  //   case CardboardViewportOrientation_LandscapeLeft:
  //         ekf_to_head_tracker = Rotation::FromYawPitchRoll(-M_PI / 2., 0, -M_PI / 2.);
  //         sensor_to_display = Rotation::FromAxisAndAngle(Vector3(0, 0, 1), M_PI / 2.);
  //         break;

  //   case CardboardViewportOrientation_LandscapeRight:
  //         ekf_to_head_tracker = Rotation::FromYawPitchRoll(M_PI / 2., 0, M_PI / 2.);
  //         sensor_to_display = Rotation::FromAxisAndAngle(Vector3(0, 0, 1), -M_PI / 2.);
  //         break;

  //   case CardboardViewportOrientation_Portrait:
  //         ekf_to_head_tracker = Rotation::FromYawPitchRoll(M_PI / 2., M_PI / 2., M_PI / 2.);
  //         sensor_to_display = Rotation::FromAxisAndAngle(Vector3(0, 0, 1), 0.);
  //         break;

  //   default: // Portrait and PortraitUpsideDown
  //         ekf_to_head_tracker = Rotation::FromYawPitchRoll(-M_PI / 2., -M_PI / 2., -M_PI / 2.);
  //         sensor_to_display = Rotation::FromAxisAndAngle(Vector3(0, 0, 1), M_PI);
  //         break;
  // }

  // const Rotation ekf_to_head_tracker =
  //     Rotation::FromYawPitchRoll(-M_PI / 2.0, 0, -M_PI / 2.0);
  // const Rotation sensor_to_display =
  //     Rotation::FromAxisAndAngle(Vector3(0, 0, 1), M_PI / 2.0);

  const Vector4 q =
      (sensor_to_display * predicted_rotation * ekf_to_head_tracker)
          .GetQuaternion();
  Rotation rotation;
  rotation.SetQuaternion(q);

  out_orientation[0] = static_cast<float>(rotation.GetQuaternion()[0]);
  out_orientation[1] = static_cast<float>(rotation.GetQuaternion()[1]);
  out_orientation[2] = static_cast<float>(rotation.GetQuaternion()[2]);
  out_orientation[3] = static_cast<float>(rotation.GetQuaternion()[3]);

  out_position = ApplyNeckModel(out_orientation, 1.0);
}

Rotation HeadTracker::GetDefaultOrientation() const {
  return Rotation::FromRotationMatrix(
      Matrix3x3(0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0));
}

void HeadTracker::RegisterCallbacks() {
  accel_sensor_->StartSensorPolling(&on_accel_callback_);
  gyro_sensor_->StartSensorPolling(&on_gyro_callback_);
}

void HeadTracker::UnregisterCallbacks() {
  accel_sensor_->StopSensorPolling();
  gyro_sensor_->StopSensorPolling();
}

void HeadTracker::OnAccelerometerData(const AccelerometerData& event) {
  if (!is_tracking_) {
    return;
  }
  sensor_fusion_->ProcessAccelerometerSample(event);
}

void HeadTracker::OnGyroscopeData(const GyroscopeData& event) {
  if (!is_tracking_) {
    return;
  }
  latest_gyroscope_data_ = event;
  sensor_fusion_->ProcessGyroscopeSample(event);
}

// constexpr void HeadTracker::GetViewporOrientationRotation(
//                   const CardboardViewportOrientation ViewportOrientation,
//                   Rotation& ekf_to_head_tracker, 
//                   Rotation& sensor_to_display) const {
//   switch (ViewportOrientation) {
//     case CardboardViewportOrientation_LandscapeLeft:
//           ekf_to_head_tracker = Rotation::FromYawPitchRoll(-M_PI / 2., 0, -M_PI / 2.);
//           sensor_to_display = Rotation::FromAxisAndAngle(Vector3(0, 0, 1), M_PI / 2.);
//           break;

//     case CardboardViewportOrientation_LandscapeRight:
//           ekf_to_head_tracker = Rotation::FromYawPitchRoll(M_PI / 2., 0, M_PI / 2.);
//           sensor_to_display = Rotation::FromAxisAndAngle(Vector3(0, 0, 1), -M_PI / 2.);
//           break;

//     case CardboardViewportOrientation_Portrait:
//           ekf_to_head_tracker = Rotation::FromYawPitchRoll(M_PI / 2., M_PI / 2., M_PI / 2.);
//           sensor_to_display = Rotation::FromAxisAndAngle(Vector3(0, 0, 1), 0.);
//           break;

//     default: // PortraitUpsideDown
//           ekf_to_head_tracker = Rotation::FromYawPitchRoll(-M_PI / 2., -M_PI / 2., -M_PI / 2.);
//           sensor_to_display = Rotation::FromAxisAndAngle(Vector3(0, 0, 1), M_PI);
//           break;
//   }
// }


}  // namespace cardboard
