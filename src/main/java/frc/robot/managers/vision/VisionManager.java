// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.LimelightSubsystem;

public class VisionManager extends LifecycleSubsystem {
  /* Aggregates vision based actions into commands for the robot. */
  private final LimelightSubsystem limelight;
  private final SwerveSubsystem swerve;

  public VisionManager(LimelightSubsystem limelight, SwerveSubsystem swerve) {
    super(SubsystemPriority.VISION_MANAGER);

    this.limelight = limelight;
    this.swerve = swerve;
  }

  public Command getAutoScoreMidCone() {
    // Return full command to auto score cone mid.

    // TODO(Simon): Finish command.
    // Do we drive to position, raise up, then score? Do we raise up, drive to position, then raise
    // up?
    return limelight
        .setPipelineCommand(123)
        .andThen(alignWithVisionTargetCommand().until(() -> atLocation()))
        .finallyDo(
            (boolean interrupted) -> {
              // TODO(Simon): Fill in.

              // Set drive speeds to 0.

              // Turn off LEDs.
            });
  }

  private CommandBase alignWithVisionTargetCommand() {
    return Commands.run(
        () -> {
          // TODO(Simon): Fill in.

          // Set swerve speeds with calculateSwerveSpeeds()
        },
        limelight,
        swerve);
  }

  private boolean atLocation() {
    // Return true once at location

    return false;
  }

  private ChassisSpeeds calculateSwerveSpeeds() {
    // Get closest middle cone target.

    // Calculate X and Y speeds

    return new ChassisSpeeds();
  }
}