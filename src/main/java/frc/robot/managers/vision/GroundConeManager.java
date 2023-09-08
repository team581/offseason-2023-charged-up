// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers.vision;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.imu.ImuSubsystem;
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
  private final ImuSubsystem imu;

  private boolean firstConeDetected = false;
  private double goalAngle = 0;
  private double yP = -0.1;

  public GroundConeManager(
      LimelightSubsystem limelight,
      SwerveSubsystem swerve,
      SuperstructureManager superstructure,
      IntakeSubsystem intake,
      ImuSubsystem imu) {
    super(SubsystemPriority.VISION_MANAGER);

    this.limelight = limelight;
    this.swerve = swerve;
    this.superstructure = superstructure;
    this.intake = intake;
    this.imu = imu;
  }

  public Command getGroundCone() {
    // Return full command to auto score cone mid.

    // TODO(Simon): Finish command.
    // Do we drive to position, raise up, then score? Do we raise up, drive to position, then raise
    // up?
    return limelight
        .setPipelineCommand(0)
        .andThen(() -> firstConeDetected = false)
        .andThen(() -> superstructure.setIntakeMode(HeldGamePiece.CONE))
        .andThen(superstructure.getFloorIntakeIdleCommand())
        .andThen(
            superstructure
                .getFloorIntakeSpinningCommand()
                .alongWith(
                    Commands.run(
                            () -> swerve.setChassisSpeeds(calculateSwerveSpeeds(), false), swerve)
                        .until(() -> intake.getGamePiece() == HeldGamePiece.CONE)
                        .andThen(
                            Commands.runOnce(
                                () -> {
                                  swerve.setChassisSpeeds(new ChassisSpeeds(), false);
                                }))));
  }

  private ChassisSpeeds calculateSwerveSpeeds() {
    // Get closest middle cone target.

    if (!firstConeDetected) {
      // Get closest cone.
      VisionTarget closestCone = limelight.getClosestCone();

      Logger.getInstance().recordOutput("Vision/LimelightX", closestCone.x);
      // If valid, get angle
      if (closestCone.valid) {
        firstConeDetected = true;
        goalAngle = imu.getRobotHeading().getDegrees() - (closestCone.x * 1.0);
      }
      // Save angle + current robot angle
    }
    if (firstConeDetected) {
      double thetaSpeed = (imu.getRobotHeading().getDegrees() - goalAngle) * yP;
      // PID to the saved angle, using the current angle as your process
      Logger.getInstance().recordOutput("Vision/ThetaSpeed", thetaSpeed);
      return new ChassisSpeeds(2.0, 0, thetaSpeed);
    } else {
      return new ChassisSpeeds(2.0, 0, 0);
    }
  }

  private void logging() {
    VisionTarget closestCone = limelight.getClosestCone();

    Logger.getInstance().recordOutput("Vision/LimelightX", closestCone.x);
    Logger.getInstance().recordOutput("Vision/ThetaSpeed", closestCone.x * yP);
    Logger.getInstance()
        .recordOutput("Vision/RealThetaSpeed", swerve.getChassisSpeeds().omegaRadiansPerSecond);
    Logger.getInstance().recordOutput("Vision/Angle", imu.getRobotHeading().getDegrees());
    Logger.getInstance()
        .recordOutput("Vision/RealThetaSpeed", swerve.getChassisSpeeds().vxMetersPerSecond);
    Logger.getInstance()
        .recordOutput("Vision/RealThetaSpeed", swerve.getChassisSpeeds().vyMetersPerSecond);
    Logger.getInstance()
        .recordOutput("Vision/AngularError", swerve.getChassisSpeeds().vyMetersPerSecond);
  }
}
