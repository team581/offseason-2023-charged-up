// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.States;
import frc.robot.intake.HeldGamePiece;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.managers.SuperstructureManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.LimelightSubsystem;
import frc.robot.vision.VisionTarget;
import org.littletonrobotics.junction.Logger;

public class GroundConeManager extends LifecycleSubsystem {
  /* Aggregates vision based actions into commands for the robot. */
  private final LimelightSubsystem limelight;
  private final SwerveSubsystem swerve;
  private final SuperstructureManager superstructure;
  private final IntakeSubsystem intake;

  private double yP = -0.1;

  public GroundConeManager(
      LimelightSubsystem limelight,
      SwerveSubsystem swerve,
      SuperstructureManager superstructure,
      IntakeSubsystem intake) {
    super(SubsystemPriority.VISION_MANAGER);

    this.limelight = limelight;
    this.swerve = swerve;
    this.superstructure = superstructure;
    this.intake = intake;
  }

  public Command getGroundCone() {
    // Return full command to auto score cone mid.

    // TODO(Simon): Finish command.
    // Do we drive to position, raise up, then score? Do we raise up, drive to position, then raise
    // up?
    return limelight
        .setPipelineCommand(0)
        .andThen(() -> superstructure.setIntakeMode(HeldGamePiece.CONE))
        .andThen(superstructure.getFloorIntakeSpinningCommand())
        .alongWith(
            Commands.run(() -> swerve.setChassisSpeeds(calculateSwerveSpeeds(), false), swerve)
                .until(() -> intake.getGamePiece() == HeldGamePiece.CONE))
        .andThen(
            superstructure
                .getCommand(States.STOWED)
                .alongWith(
                    Commands.runOnce(
                        () -> swerve.setChassisSpeeds(new ChassisSpeeds(), false), swerve)));
  }

  private ChassisSpeeds calculateSwerveSpeeds() {
    // Get closest middle cone target.
    VisionTarget closestCone = limelight.getClosestConeTarget();

    double thetaSpeed = closestCone.x * yP;
    Logger.getInstance().recordOutput("Vision/LimelightX", closestCone.x);
    Logger.getInstance().recordOutput("Vision/ThetaSpeed", thetaSpeed);
    return new ChassisSpeeds(1.0, 0, thetaSpeed);
  }
}