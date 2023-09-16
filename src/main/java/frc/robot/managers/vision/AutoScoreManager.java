// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.NodeHeight;
import frc.robot.States;
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

  private static final double xP = -0.15;
  private static final double yP = 0.3;
  private double xOffset = 0;
  // tune setpoint value
  private static final double ySetpoint = 5.5;
  private static final double angleRange = .5;

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
    return storeConeOffsetCommand()
        .andThen(limelight.setPipelineCommand(limelight.retroPipeline))
        .andThen(alignWithVisionTargetCommand().until(() -> atLocation()))
        .andThen(
            Commands.runOnce(
                () -> {
                  swerve.setChassisSpeeds(new ChassisSpeeds(), false);
                }))
        .andThen(superstructure.getScoreCommand(NodeHeight.MID, 0.15))
        .andThen(Commands.runOnce(() -> superstructure.set(States.STOWED)))
        .finallyDo(
            (boolean interrupted) -> {
              // Set drive speeds to 0.
              swerve.setChassisSpeeds(new ChassisSpeeds(), false);
              swerve.disableSnapToAngle();
              // Turn off LEDs.
              limelight.turnOffLights();
              limelight.setPipeline(0);
            });
  }

  private Command storeConeOffsetCommand() {
    return limelight
        .setPipelineCommand(LimelightSubsystem.HELD_CONE_PIPELINE)
        // Short delay to ensure Limelight has the cone offset ready
        .andThen(Commands.waitSeconds(0.1))
        // Store cone offset for later
        .andThen(Commands.runOnce(() -> xOffset = limelight.getHeldConeXOffset(), limelight));
  }

  private CommandBase alignWithVisionTargetCommand() {
    return Commands.run(
        () -> {
          // TODO(Simon): Fill in.
          swerve.setSnapToAngle(AutoRotate.getBackwardsAngle());
          swerve.setChassisSpeeds(calculateSwerveSpeeds(), false);
          // Set swerve speeds with calculateSwerveSpeeds()
        },
        limelight,
        swerve);
  }

  private boolean atLocation() {
    // Return true once at location
    VisionTarget closestNode = limelight.getClosestMiddleConeTarget();
    return closestNode.x >= xOffset - angleRange
        && closestNode.x <= xOffset + angleRange
        && closestNode.y >= ySetpoint - angleRange
        && closestNode.y <= ySetpoint + angleRange;
  }

  private ChassisSpeeds calculateSwerveSpeeds() {
    // Get closest middle cone target.
    VisionTarget closestNode = limelight.getClosestMiddleConeTarget();
    // Calculate X and Y speeds
    double sidewaysSpeed = (closestNode.x + xOffset) * xP;
    double forwardSpeed = (closestNode.y - ySetpoint) * yP;
    Logger.getInstance().recordOutput("Vision/LimelightX", closestNode.x);
    Logger.getInstance().recordOutput("Vision/LimelightXOffset", xOffset);
    Logger.getInstance().recordOutput("Vision/LimelightXWithOffset", closestNode.x + xOffset);
    Logger.getInstance().recordOutput("Vision/LimelightY", closestNode.y);

    if (closestNode.valid) {
      return new ChassisSpeeds(forwardSpeed, sidewaysSpeed, 0);
    } else {
      return new ChassisSpeeds();
    }
  }

  private void logging() {
    VisionTarget closestNode = limelight.getClosestMiddleConeTarget();
    Logger.getInstance().recordOutput("Vision/LimelightX", closestNode.x);
    Logger.getInstance().recordOutput("Vision/LimelightY", closestNode.y);
    Logger.getInstance().recordOutput("Vision/Valid", closestNode.valid);
  }
}
