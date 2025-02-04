// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/RunCommand.h>

#include "commands/Autos.h"
#include "commands/SwerveJoystickCmd.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();

  m_swerveSubsystem.SetDefaultCommand(frc2::RunCommand(
    [this] {
        m_swerveSubsystem.Drive(
            m_driverController.GetLeftX(),
            m_driverController.GetLeftY(),
            m_driverController.GetRightX(),
            true);
      },
      {&m_swerveSubsystem})
  );
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  m_driverController.LeftBumper().OnTrue(m_swerveSubsystem.ZeroHeadingCommand());

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  // frc2::Trigger([this] {
  //   return m_subsystem.ExampleCondition();
  // }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  // m_driverController.B().WhileTrue(m_subsystem.ExampleMethodCommand());
}

// frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
//   // An example command will be run in autonomous
//   return autos::ExampleAuto(&m_swerveSubsystem);
// }
