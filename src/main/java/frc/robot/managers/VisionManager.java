// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.localization.GamePiecePosition;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class VisionManager extends LifecycleSubsystem {
  public final LocalizationSubsystem localization;
  public final SuperstructureManager superstructure;
  public final SwerveSubsystem swerve;

  public final PIDController thetaController = new PIDController(0.1, 0, 0);

  public VisionManager(
      LocalizationSubsystem localization,
      SuperstructureManager superstructure,
      SwerveSubsystem swerve) {
    super(SubsystemPriority.VISION_MANAGER);
    this.localization = localization;
    this.superstructure = superstructure;
    this.swerve = swerve;
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("VisionManager/tx", localization.getGamePiecePosition().tx);
    Logger.getInstance().recordOutput("VisionManager/ty", localization.getGamePiecePosition().ty);
  }

  public Command getConeIntakeCommand() {
    // starts spinning the intake
    return superstructure
        .getFloorIntakeSpinningCommand()
        // looks at the cone and positions the robot toward it
        .alongWith(
            Commands.run(
                () -> {
                  GamePiecePosition gamePiecePosition = localization.getGamePiecePosition();
                  ChassisSpeeds chassisSpeeds =
                      new ChassisSpeeds(0, 0, thetaController.calculate(gamePiecePosition.tx));
                  swerve.setChassisSpeeds(chassisSpeeds, false);
                },
                swerve))
                //while looking at cone robot will drive forward slowly, pickup and stow
        .andThen(
            Commands.run(
                () -> {
                  swerve.setChassisSpeeds(
                      new ChassisSpeeds(
                          0.75,
                          0,
                          thetaController.calculate(localization.getGamePiecePosition().tx)),
                      false);
                },
                swerve));
  }
}
