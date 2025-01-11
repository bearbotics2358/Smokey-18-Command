#pragma once

#include <ctre/Phoenix.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/Counter.h>

class SwerveModule {
public:
    SwerveModule(int driveID, int steerID, int canCoderID);

    // Returns position of the distance encoder in meters
    double getDistance();

    double getVelocity();
    double getRadians();
    frc::SwerveModulePosition GetPosition();
    frc::SwerveModuleState getState();

    // sets the drive encoder to 0 ticks
    void resetDriveEncoder();

    // scaled angle between 0 and 360
    float getAngle();
    // get angle from relative encoder in degrees, does not take into consideration currently set zero point
    double getRelativeAngle();

    void goToPosition(float meters); // Position PID control, moves drive wheel to specified position
    void steerToAng(float degrees); // Angle PID control

    // sets drive speed in percent
    void setDrivePercent(float percent);
    // sets steer speed in percent
    void setSteerPercent(float percent);

    // set wheel speed in meters per second
    float setDriveSpeed(float speed);

    // set the brake mode of the drive motor
    void setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode mode);

    // sets drive and steer p, i, and d values for pid
    void setDrivePID(double pNew, double iNew, double dNew);
    void setSteerPID(double pNew, double iNew, double dNew);

    // steers to the given targetAngle by taking the shortest possible path of rotation
    // this means the wheel may end up facing backwards
    // if that is the case, this returns true to indicate that the wheel speed should be opposite of what it would normally be
    bool adjustAngle(float targetAngle);

private:
    TalonFX m_driveMotor;
    TalonFX m_steerMotor;

    TalonFXSensorCollection m_driveEncoder;
    TalonFXSensorCollection m_steerEncoder;
    CANCoder m_canCoder;
    frc::PIDController m_steerPID;

    int m_canCoderID;

    // how many degrees away from the actual zero degrees
    // that the relative encoder's zero point is
    double m_encoderZeroPointDeg { 0.0 };

    // speed is meters per second
    static double wheelSpeedToRpm(double speed);
    static double metersToMotorTicks(double meters);
    static double motorTicksToMeters(double motorTicks);
    static double motorTicksToRadians(double motorTicks);
};