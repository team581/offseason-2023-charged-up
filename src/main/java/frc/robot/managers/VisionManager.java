// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.HeldGamePiece;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class VisionManager extends LifecycleSubsystem {
  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private final IntakeSubsystem intake;
  private final ImuSubsystem imu;
  private final SuperstructureManager superstructure;
  private final PIDController thetaController = new PIDController(0.1, 0, 0);

  public VisionManager(
      LocalizationSubsystem localization,
      SwerveSubsystem swerve,
      IntakeSubsystem intake,
      ImuSubsystem imu,
      SuperstructureManager superstructure) {
    super(SubsystemPriority.VISION_MANAGER);
    this.localization = localization;
    this.swerve = swerve;
    this.intake = intake;
    this.imu = imu;
    this.superstructure = superstructure;
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance()
        .recordOutput("VisionManager/GampePiece/tx", localization.getGamePiecePosition().tx);
    Logger.getInstance()
        .recordOutput("VisionManager/GamePiece/ty", localization.getGamePiecePosition().ty);
  }

  public Command getIntakeConeCommand() {
    return superstructure
        .getFloorIntakeSpinningCommand()
        .alongWith(
            // turning to face cone
            Commands.run(
                    () -> {
                      ChassisSpeeds speeds =
                          new ChassisSpeeds(
                              0,
                              0,
                              thetaController.calculate(localization.getGamePiecePosition().tx, 0));
                      swerve.setChassisSpeeds(speeds, false);
                    },
                    swerve)
                .until(
                    () ->
                        localization.getGamePiecePosition().tx < 1
                            && localization.getGamePiecePosition().tx > -1)
                .andThen(
                    // driving to cone and rotate towards it
                    Commands.run(
                        () -> {
                          ChassisSpeeds speeds =
                              new ChassisSpeeds(
                                  1,
                                  0,
                                  thetaController.calculate(
                                      localization.getGamePiecePosition().tx, 0));
                          swerve.setChassisSpeeds(speeds, false);
                        },
                        swerve))
                // drive until ty is at a certain threshold when we almost lose vision of the cone
                .until(() -> localization.getGamePiecePosition().ty < -100))
        .andThen(
            // maintain heading
            Commands.run(
                    () -> {
                      ChassisSpeeds speeds = new ChassisSpeeds(1, 0, 0);
                      swerve.setSnapToAngle(imu.getRobotHeading());
                      swerve.setChassisSpeeds(speeds, false);
                    },
                    swerve)
                // until we intake cone
                .until((() -> intake.getGamePiece() == HeldGamePiece.CONE))
                .withTimeout(1));
  }
}
