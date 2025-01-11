#pragma once

#include <frc/geometry/Translation2d.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

inline constexpr int kOperatorPort = 0;
inline constexpr int kDriverControllerPort = 5;

inline constexpr double kJoystickDeadzone = 0.15;

}  // namespace OperatorConstants

namespace RobotConstants {
inline constexpr double kDriveLength = 13.0026;
inline constexpr double kDriveWidth = 29.4878;
}

namespace MotorConstants {
// falcon encoder ticks per 1 revolution
inline constexpr int kFalconUnitsPerRev = 2048;
}  // namespace MotorConstants

namespace SwerveConstants {

inline constexpr double kMaxSpeedMps = 3.0;
inline constexpr double kMaxAccelerationMpsSq = 3.0;
inline constexpr double kMaxAngularSpeedRadPerSecond = 3.142;

inline constexpr int kGyroID = 35;

/**
 * Module Numbering Scheme:
 * m = number engraved on module
 * Drive ID: 2m - 1
 * Steering ID: 2m
 * Encoder ID: 16 + m
*/
inline constexpr int FL_ID = 1;
inline constexpr int FR_ID = 2;
inline constexpr int BL_ID = 3;
inline constexpr int BR_ID = 7;

inline constexpr double CANCODER_OFFSETS[8] = {
    715.01,         // Offset for Swerve Module 1
    63.471,         // Offset for Swerve Module 2
    -197.93 + 180,  // Offset for Swerve Module 3
    508.6 - 360,    // Offset for Swerve Module 4
    -216.17,        // Offset for Swerve Module 5
    35.77,          // Offset for Swerve Module 6
    99.22,          // Offset for Swerve Module 7
    246.2,          // Offset for Swerve Module 8
};

// the peak supply current, in amps
inline constexpr double kDriveMotorTriggerThresholdCurrent = 30.0;
// the time at the peak supply current before the limit triggers, in seconds
inline constexpr double kDriveMotorTriggerThresholdTime = 0.1;
// the current to maintain if the peak supply limit is triggered
inline constexpr double kDriveMotorCurrentLimit = 20.0;

// the peak supply current, in amps -- Set this lower than the drive motor threshold since the steering motors
// shouldn't need as much to get into position.
inline constexpr double kSteerMotorTriggerThresholdCurrent = 20.0;
// the time at the peak supply current before the limit triggers, in seconds
inline constexpr double kSteerMotorTriggerThresholdTime = 0.1;
// the current to maintain if the peak supply limit is triggered
inline constexpr double kSteerMotorCurrentLimit = 15.0;

// the distance we were getting from the wheel was not quite right, so we multiply them by this constant to get the right distance
inline constexpr double kDistanceAdjustmentFactor = 1.09789;

inline constexpr double kWheelDiameterM = 0.095;

// @todo Measure this value again to verify it is correct
// ratio is drive motor rotations / wheel rotations
inline constexpr double kSwerveDriveMotorGearRatio = 6.75 / 1.0;
}  // namespace SwerveConstants