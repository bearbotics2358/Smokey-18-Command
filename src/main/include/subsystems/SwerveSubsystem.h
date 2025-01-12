#pragma once

#include "SwerveModule.h"
#include <frc/controller/ProfiledPIDController.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

class SwerveSubsystem : public frc2::SubsystemBase {
public:
    SwerveSubsystem();
    virtual ~SwerveSubsystem();

    /**
     * Public command for zeroing the IMU heading
     */
    frc2::CommandPtr ZeroHeadingCommand();

    /**
     * An example method querying a boolean state of the subsystem (for example, a
     * digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    bool ExampleCondition();

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() override;

    /**
     * Will be called periodically whenever the CommandScheduler runs during
     * simulation.
     */
    void SimulationPeriodic() override;

    void Drive(double xSpeedMps, double ySpeedMps, double rotRadPerSecond, bool fieldOriented);

private:
    // Components (e.g. motor controllers and sensors) should generally be
    // declared private and exposed only through public methods.

    SwerveModule m_flModule;
    SwerveModule m_frModule;
    SwerveModule m_blModule;
    SwerveModule m_brModule;

    ctre::phoenix::sensors::PigeonIMU m_pigeonIMU;

    // pid when using turn to angle
    frc::PIDController m_turnAnglePid;

    frc::TrapezoidProfile<units::meters>::Constraints m_linearConstraints{
        units::meters_per_second_t(1.0),
        units::meters_per_second_squared_t(1.0)}
    ;
    frc::ProfiledPIDController<units::meters> m_xProfiledPid;
    frc::ProfiledPIDController<units::meters> m_yProfiledPid;

    frc::TrapezoidProfile<units::radian>::Constraints m_rotationalConstraints{
        units::radians_per_second_t(2.0),
        units::radians_per_second_squared_t(2.0)
    };
    frc::ProfiledPIDController<units::radian> m_rotProfiledPid;

    frc::SwerveDriveOdometry<4> m_odometry;

    // Frame size: 36 in 29.5 in
    frc::SwerveDriveKinematics<4> m_kinematics{
        frc::Translation2d{+0.11_m, +0.375_m},  // Front Left
        frc::Translation2d{+0.2_m, -0.375_m},   // Front Right
        frc::Translation2d{-0.11_m, +0.375_m},  // Back Left
        frc::Translation2d{-0.2_m, -0.375_m}    // Back Right
    };

    frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

    wpi::array<frc::SwerveModulePosition, 4U> GetModulePositions();
    double GetHeadingDegrees();
    void ZeroHeading();
    frc::Rotation2d GetRotation2d();

    void BrakeOnStop();

    void Stop();
};
