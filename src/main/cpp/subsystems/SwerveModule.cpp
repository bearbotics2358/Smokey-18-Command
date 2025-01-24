#include "Constants.h"
#include "Misc.h"
#include "subsystems/SwerveModule.h"

SwerveModule::SwerveModule(int driveID, int steerID, int canCoderID):
    m_driveMotor(driveID),
    m_steerMotor(steerID),
    m_driveEncoder(m_driveMotor),
    m_steerEncoder(m_steerMotor),
    m_canCoder(canCoderID),
    m_steerPID(0, 0, 0)
{
    // @todo Change 17 to a named constant OR find a clearer way to map the ID values
    m_canCoderID = canCoderID - 17;

    m_steerMotor.SetInverted(true);

    ctre::phoenix::motorcontrol::can::TalonFXConfiguration drive_config;

    // Current limits for the drive motor to prevent it from drawing too much current and browning out the system
    drive_config.supplyCurrLimit.triggerThresholdCurrent = SwerveConstants::kDriveMotorTriggerThresholdCurrent;
    drive_config.supplyCurrLimit.triggerThresholdTime = SwerveConstants::kDriveMotorTriggerThresholdTime;
    drive_config.supplyCurrLimit.currentLimit = SwerveConstants::kDriveMotorCurrentLimit;
    drive_config.supplyCurrLimit.enable = true;

    drive_config.velocityMeasurementPeriod = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_20Ms;
    // this allows the motor to actually turn, pid values are set later
    drive_config.slot0.kP = 1.0;

    m_driveMotor.ConfigAllSettings(drive_config);

    ctre::phoenix::motorcontrol::can::TalonFXConfiguration steer_config;

    // Current limits for the steer motor to prevent it from drawing too much current and browning out the system
    steer_config.supplyCurrLimit.triggerThresholdCurrent = SwerveConstants::kSteerMotorTriggerThresholdCurrent;
    steer_config.supplyCurrLimit.triggerThresholdTime = SwerveConstants::kSteerMotorTriggerThresholdTime;
    steer_config.supplyCurrLimit.currentLimit = SwerveConstants::kSteerMotorCurrentLimit;
    steer_config.supplyCurrLimit.enable = true;

    steer_config.velocityMeasurementPeriod = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_25Ms;

    // this allows the motor to actually turn, pid values are set later
    steer_config.slot0.kP = 1.0;
    m_steerMotor.ConfigAllSettings(steer_config);
    m_steerMotor.ConfigNeutralDeadband(0.001);

    m_steerPID.EnableContinuousInput(0.0, 360.0);
}

double SwerveModule::getDistance() {
    return motorTicksToMeters(m_driveMotor.GetSelectedSensorPosition());
}
double SwerveModule::getVelocity() {
    return 10.0*((motorTicksToMeters(m_driveMotor.GetSelectedSensorVelocity())));
}
double SwerveModule::getRadians() {
    return motorTicksToRadians(m_steerMotor.GetSelectedSensorPosition());
}

void SwerveModule::resetDriveEncoder() {
    m_driveEncoder.SetIntegratedSensorPosition(0);
}

double SwerveModule::getRelativeAngle() {
    double angle = (m_canCoder.GetAbsolutePosition() - SwerveConstants::CANCODER_OFFSETS[m_canCoderID]);

    float adjusted = angle;
    if (angle < 0) {
        adjusted += 360; // bounds to 0-360
    }

    return adjusted;
}

float SwerveModule::getAngle() {
    // @todo Find a better way to show the angle in SmartDashboard
    // if((m_canCoderID + 17) == misc::GetFLCANCoder()) {
    //     double position = m_canCoder.GetAbsolutePosition() - CANCODER_OFFSETS[m_canCoderID];
    //     frc::SmartDashboard::PutNumber("fl position", position);
    // }
    // if((m_canCoderID + 17) == misc::GetBLCANCoder()) {
    //     double position = m_canCoder.GetAbsolutePosition() - CANCODER_OFFSETS[m_canCoderID];
    //     frc::SmartDashboard::PutNumber("bl position", position);
    // }
    // if((m_canCoderID + 17) == misc::GetFRCANCoder()) {
    //     double position = m_canCoder.GetAbsolutePosition() - CANCODER_OFFSETS[m_canCoderID];
    //     frc::SmartDashboard::PutNumber("fr position", position);
    // }
    // if((m_canCoderID + 17) == misc::GetBRCANCoder()) {
    //     double position = m_canCoder.GetAbsolutePosition() - CANCODER_OFFSETS[m_canCoderID];
    //     frc::SmartDashboard::PutNumber("br position", position);
    // }
    return misc::clampDegrees(getRelativeAngle() + m_encoderZeroPointDeg);
}


void SwerveModule::goToPosition(float meters) {
    float ticks = SwerveModule::metersToMotorTicks(meters);
    m_driveMotor.Set(TalonFXControlMode::Position, ticks);
}

void SwerveModule::steerToAng(float degrees) {
    float speed = std::clamp(m_steerPID.Calculate(getAngle(), degrees) / 270.0, -0.5, 0.5);
    m_steerMotor.Set(TalonFXControlMode::PercentOutput, speed);
}

void SwerveModule::setDrivePercent(float percent) {
    m_driveMotor.Set(TalonFXControlMode::PercentOutput, percent);
}

void SwerveModule::setSteerPercent(float percent) {
    m_steerMotor.Set(TalonFXControlMode::PercentOutput, percent);
}

float SwerveModule::setDriveSpeed(float speed) {
    float rpm = SwerveModule::wheelSpeedToRpm(speed);

    m_driveMotor.Set(TalonFXControlMode::Velocity, misc::rpmToTalonVel(rpm));

    return speed;
}

void SwerveModule::setBrakeMode(ctre::phoenix::motorcontrol::NeutralMode mode) {
    m_driveMotor.SetNeutralMode(mode);
}

void SwerveModule::setDrivePID(double pNew, double iNew, double dNew) {
    m_driveMotor.Config_kP(0, pNew);
    m_driveMotor.Config_kI(0, iNew);
    m_driveMotor.Config_kD(0, dNew);
}

void SwerveModule::setSteerPID(double pNew, double iNew, double dNew) {
    m_steerPID.SetP(pNew);
    m_steerPID.SetI(iNew);
    m_steerPID.SetD(dNew);
}

bool SwerveModule::adjustAngle(float targetAngle) {
    float tempCurrent = getAngle();
    float tempTarget = targetAngle;
    bool changeMade = false;

    if (tempCurrent - tempTarget > 180) {
        tempCurrent -= 360;
    } else if (tempCurrent - tempTarget < -180) {
        tempCurrent += 360;
    }
    float distOfAngle = tempTarget - tempCurrent;

    if (distOfAngle > 90) {
        tempTarget -= 180;
        changeMade = true;
    }

    if (distOfAngle < -90) {
        tempTarget += 180;
        changeMade = true;
    }

    if (tempTarget < 0) {
        tempTarget += 360;
    } else if (tempTarget > 360) {
        tempTarget -= 360;
    }

    steerToAng(tempTarget);

    return changeMade;
}

double SwerveModule::wheelSpeedToRpm(double speed) {
    double adjustedSpeed = speed / SwerveConstants::kDistanceAdjustmentFactor;
    // radians per second
    double angularVelocity = adjustedSpeed / (0.5 * SwerveConstants::kWheelDiameterM);
    // convert to rpm
    double rpm = (60.0 * angularVelocity) / (2 * M_PI);
    // convert from wheel rpm to motor rpm
    return rpm * SwerveConstants::kSwerveDriveMotorGearRatio;
}

double SwerveModule::metersToMotorTicks(double meters) {
    double adjustedMeters = meters / SwerveConstants::kDistanceAdjustmentFactor;
    // angular position in radians
    double angularPosition = adjustedMeters / (0.5 * SwerveConstants::kWheelDiameterM);
    // convert to encoder ticks
    double ticks = (MotorConstants::kFalconUnitsPerRev * angularPosition) / (2 * M_PI);
    // scale by gear ratio
    return ticks * SwerveConstants::kSwerveDriveMotorGearRatio;
}

double SwerveModule::motorTicksToMeters(double motorTicks) {
    // like ticks of the wheel
    double scaledTicks = motorTicks / SwerveConstants::kSwerveDriveMotorGearRatio;
    double rotations = (scaledTicks / MotorConstants::kFalconUnitsPerRev);
    // angular position in radians
    double angularPosition = rotations * 2 * M_PI;
    return angularPosition * 0.5 * SwerveConstants::kWheelDiameterM;
}
double SwerveModule::motorTicksToRadians(double motorTicks) {
    // like ticks of the wheel
    double scaledTicks = motorTicks / SwerveConstants::kSwerveDriveMotorGearRatio;
    double rotations = (scaledTicks / MotorConstants::kFalconUnitsPerRev);
    // angular position in radians
    double angularPosition = rotations * 2 * M_PI;
    return angularPosition;
}
frc::SwerveModulePosition SwerveModule::GetPosition(){
  return {units::meter_t{getDistance()},
          frc::Rotation2d(units::degree_t{getAngle()})};
}

frc::SwerveModuleState SwerveModule::getState(){
    return frc::SwerveModuleState(units::meters_per_second_t(getVelocity()), frc::Rotation2d(units::degree_t((getAngle()))));
}