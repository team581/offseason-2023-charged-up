// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visionActions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.vision.Limelight;

public class Actions {
  /* Aggregates vision based actions into commands for the robot. */
  private final Limelight limelight;
  private final SwerveSubsystem swerve;

  public Actions(Limelight limelight, SwerveSubsystem swerve) {
    this.limelight = limelight;
    this.swerve = swerve;
  }

  public Command getAutoScoreMidCone() {
    // Return full command to auto score cone mid.

    // TODO(Simon): Finish command.
    // Do we drive to position, raise up, then score? Do we raise up, drive to position, then raise
    // up?
    return new DriveToMiddleConeNode(limelight, swerve);
  }
}
