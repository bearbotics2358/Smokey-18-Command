#include "commands/SwerveJoystickCmd.h"

SwerveJoystickCmd::SwerveJoystickCmd(SwerveSubsystem* subsystem)
    : m_swerveSubsystem{subsystem} {
  // Register that this command requires the subsystem.
  AddRequirements(m_swerveSubsystem);
}
