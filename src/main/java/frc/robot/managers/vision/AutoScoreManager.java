// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.NodeHeight;
import frc.robot.managers.AutoRotate;
import frc.robot.managers.SuperstructureManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.LimelightSubsystem;
import frc.robot.vision.VisionTarget;
import org.littletonrobotics.junction.Logger;

public class AutoScoreManager extends LifecycleSubsystem {
  /* Aggregates vision based actions into commands for the robot. */
  private final LimelightSubsystem limelight;
  private final SwerveSubsystem swerve;
  private final SuperstructureManager superstructure;

  private double xP = -0.1;
  private double yP = 0.25;
  // tune setpoint value
  private double ySetpoint = 6;
  private double angleRange = 1.3;

  public AutoScoreManager(
      LimelightSubsystem limelight, SwerveSubsystem swerve, SuperstructureManager superstructure) {
    super(SubsystemPriority.VISION_MANAGER);

    this.limelight = limelight;
    this.swerve = swerve;
    this.superstructure = superstructure;
  }

  public Command getAutoScoreMidCone() {
    // Return full command to auto score cone mid.

    // TODO(Simon): Finish command.
    // Do we drive to position, raise up, then score? Do we raise up, drive to position, then raise
    // up?
    return limelight
        .setPipelineCommand(limelight.retroPipeline)
        .andThen(alignWithVisionTargetCommand().until(() -> atLocation()))
        .andThen(
            Commands.runOnce(
                () -> {
                  swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0), false);
                }))
        .andThen(superstructure.getScoreCommand(NodeHeight.MID, 0.3))
        .finallyDo(
            (boolean interrupted) -> {

              // Set drive speeds to 0.
              swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0), false);
              // Turn off LEDs.
              limelight.turnOffLights();
            });
  }

  private CommandBase alignWithVisionTargetCommand() {
    return Commands.run(
        () -> {
          // TODO(Simon): Fill in.
          swerve.setChassisSpeeds(calculateSwerveSpeeds(), false);
          swerve.setSnapToAngle(AutoRotate.getBackwardsAngle());
          // Set swerve speeds with calculateSwerveSpeeds()
        },
        limelight,
        swerve);
  }

  private boolean atLocation() {
    // Return true once at location
    VisionTarget closestNode = limelight.getClosestMiddleConeTarget();
    return closestNode.x >= -angleRange
        && closestNode.x <= angleRange
        && closestNode.y >= ySetpoint - angleRange
        && closestNode.y <= ySetpoint + angleRange;
  }

  private ChassisSpeeds calculateSwerveSpeeds() {
    // Get closest middle cone target.
    VisionTarget closestNode = limelight.getClosestMiddleConeTarget();
    // Calculate X and Y speeds
    double ySpeed = closestNode.x * xP;
    double xSpeed = (closestNode.y - ySetpoint) * yP;
    Logger.getInstance().recordOutput("Vision/LimelightX", closestNode.x);
    Logger.getInstance().recordOutput("Vision/LimelightY", closestNode.y);

    if (closestNode.valid) {
      return new ChassisSpeeds(xSpeed, ySpeed, 0);
    } else {
      return new ChassisSpeeds(0, 0, 0);
    }
  }
}
