#include "Constants.h"
#include "Misc.h"
#include "subsystems/SwerveSubsystem.h"
#include <frc/smartdashboard/SmartDashboard.h>

SwerveSubsystem::SwerveSubsystem():
    m_flModule(misc::GetFLDrive(), misc::GetFLSteer(), misc::GetFLCANCoder()),
    m_frModule(misc::GetFRDrive(), misc::GetFRSteer(), misc::GetFRCANCoder()),
    m_blModule(misc::GetBLDrive(), misc::GetBLSteer(), misc::GetBLCANCoder()),
    m_brModule(misc::GetBRDrive(), misc::GetBRSteer(), misc::GetBRCANCoder()),
    m_pigeonIMU(SwerveConstants::kGyroID),
    m_turnAnglePid(0.014, 0.0, 0.0),
    m_xProfiledPid(.5, .2, 0.0, m_linearConstraints),
    m_yProfiledPid(.5, .2, 0.0, m_linearConstraints),
    m_rotProfiledPid(.5, 0.1, 0.0, m_rotationalConstraints),
    m_odometry{
        m_kinematics,
        frc::Rotation2d(units::radian_t(GetHeadingDegrees()*((2*M_PI)/360.0))),
        GetModulePositions()
    },
    m_poseEstimator{
        m_kinematics,
        frc::Rotation2d(units::radian_t(GetHeadingDegrees()*((2*M_PI)/360.0))),
        GetModulePositions(),
        frc::Pose2d(units::meter_t(0.0), units::meter_t(0.0), units::radian_t(0.0))
    }
{
    // Implementation of subsystem constructor goes here.

    // When the SwerveSubsystem is instantiated, zero the heading from the IMU to the direction the bot
    // is currently facing
    ZeroHeading();
}

SwerveSubsystem::~SwerveSubsystem() {
    // Add any cleanup code here in the destructor
}

frc2::CommandPtr SwerveSubsystem::ZeroHeadingCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return RunOnce([this] { ZeroHeading(); });
}

bool SwerveSubsystem::ExampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
}

void SwerveSubsystem::Periodic() {
    // Implementation of subsystem periodic method goes here.

    frc::SmartDashboard::PutNumber("Heading", GetHeadingDegrees());
}

void SwerveSubsystem::SimulationPeriodic() {
    // Implementation of subsystem simulation periodic method goes here.
}

wpi::array<frc::SwerveModulePosition, 4U> SwerveSubsystem::GetModulePositions() {
    return {m_flModule.GetPosition(), m_frModule.GetPosition(), m_blModule.GetPosition(), m_brModule.GetPosition()};
}

void SwerveSubsystem::ZeroHeading() {
    m_pigeonIMU.SetYaw(0);
}

double SwerveSubsystem::GetHeadingDegrees() {
    return misc::clampDegrees(m_pigeonIMU.GetYaw());
}

frc::Rotation2d SwerveSubsystem::GetRotation2d() {
    return frc::Rotation2d(units::degree_t(GetHeadingDegrees()));
}

void SwerveSubsystem::Stop() {
    m_flModule.setDrivePercent(0.0);
    m_frModule.setDrivePercent(0.0);
    m_blModule.setDrivePercent(0.0);
    m_brModule.setDrivePercent(0.0);

    m_flModule.setSteerPercent(0.0);
    m_frModule.setSteerPercent(0.0);
    m_blModule.setSteerPercent(0.0);
    m_brModule.setSteerPercent(0.0);
}

void SwerveSubsystem::Drive(double xSpeedMps, double ySpeedMps, double rotRadPerSecond, bool fieldOriented) {
    // Makes joystick inputs field oriented
    if (fieldOriented) {
        float gyroRadians = GetHeadingDegrees() * M_PI / 180;
        float temp = ySpeedMps * cos(gyroRadians) + xSpeedMps * sin(gyroRadians);
        xSpeedMps = -ySpeedMps * sin(gyroRadians) + xSpeedMps * cos(gyroRadians);
        ySpeedMps = temp;
    }

    float radius = sqrt((RobotConstants::kDriveLength * RobotConstants::kDriveLength) +
                        (RobotConstants::kDriveWidth * RobotConstants::kDriveWidth)); // radius of the drive base

    // temp variables to simplify math
    float a = xSpeedMps - rotRadPerSecond * (RobotConstants::kDriveLength / radius);
    float b = xSpeedMps + rotRadPerSecond * (RobotConstants::kDriveLength / radius);
    float c = ySpeedMps - rotRadPerSecond * (RobotConstants::kDriveWidth / radius);
    float d = ySpeedMps + rotRadPerSecond * (RobotConstants::kDriveWidth / radius);

    float flSpeed = sqrt(b * b + c * c);
    float frSpeed = sqrt(b * b + d * d);
    float blSpeed = sqrt(a * a + d * d);
    float brSpeed = sqrt(a * a + c * c);

    float flAngle = atan2(b, c) * 180 / M_PI; // calculates wheel angles and converts to radians
    float frAngle = atan2(b, d) * 180 / M_PI;
    float blAngle = atan2(a, c) * 180 / M_PI;
    float brAngle = atan2(a, d) * 180 / M_PI;

    if (flAngle < 0) {
        flAngle = flAngle + 360;
    }

    if (frAngle < 0) {
        frAngle = frAngle + 360;
    }

    if (blAngle < 0) {
        blAngle = blAngle + 360;
    }

    if (brAngle < 0) {
        brAngle = brAngle + 360;
    }

    float max = std::max(std::max(frSpeed, flSpeed), std::max(brSpeed, blSpeed)); // find max speed value

    // scale inputs respectively so no speed is greater than 1
    if (max > 1) {
        flSpeed /= max;
        frSpeed /= max;
        blSpeed /= max;
        brSpeed /= max;
    }

    float scalar = 1; // scalar to adjust if speed is too high
    flSpeed *= scalar;
    frSpeed *= scalar;
    blSpeed *= scalar;
    brSpeed *= scalar;

    bool inDeadzone = (sqrt(xSpeedMps * xSpeedMps + ySpeedMps * ySpeedMps) < OperatorConstants::kJoystickDeadzone)
                       && (fabs(rotRadPerSecond) < OperatorConstants::kJoystickDeadzone);

    // Joysticks have some amound of deadzone around 0. If the joystick is near 0, stop the swerves
    if (inDeadzone) {
        Stop();
    } else {
        if (m_flModule.adjustAngle(flAngle)) {
            m_flModule.setDrivePercent(-flSpeed);
        } else {
            m_flModule.setDrivePercent(flSpeed);
        }

        if (m_frModule.adjustAngle(frAngle)) {
            m_frModule.setDrivePercent(-frSpeed);
        } else {
            m_frModule.setDrivePercent(frSpeed);
        }

        if (m_blModule.adjustAngle(blAngle)) {
            m_blModule.setDrivePercent(-blSpeed);
        } else {
            m_blModule.setDrivePercent(blSpeed);
        }

        if (m_brModule.adjustAngle(brAngle)) {
            m_brModule.setDrivePercent(-brSpeed);
        } else {
            m_brModule.setDrivePercent(brSpeed);
        }
    }
}

// void SwerveSubsystem::setModuleStates(frc::SwerveModuleState desiredStates[]) {
//     // Is this needed?
// }
