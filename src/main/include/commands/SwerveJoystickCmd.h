#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveSubsystem.h"

/**
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SwerveJoystickCmd
    : public frc2::CommandHelper<frc2::Command, SwerveJoystickCmd> {
public:
    /**
     * Creates a new SwerveJoystickCmd.
     *
     * @param subsystem The subsystem used by this command.
    */
    explicit SwerveJoystickCmd(SwerveSubsystem* subsystem);

private:
    SwerveSubsystem* m_swerveSubsystem;
};
