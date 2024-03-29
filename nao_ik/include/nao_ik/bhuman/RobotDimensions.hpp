// Copyright 2021 Kenji Brameld
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

#pragma once

struct RobotDimensions
{
  float yHipOffset;  // The y offset of the left hip.
  float zHipOffset;  // The z offset of the hip.
  float upperLegLength;  // Length between leg joints HipPitch and KneePitch in z-direction.
  float lowerLegLength;  // Length between leg joints KneePitch and AnklePitch in z-direction.
  float footHeight;  // Height between the sole of the foot and the foot joint AnkleRoll.
};
