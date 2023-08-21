// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visionActions;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.vision.Limelight;

public class DriveToMiddleConeNode extends CommandBase {
  private final Limelight limelight;
  private final SwerveSubsystem swerve;
  private boolean doneDriving = false;

  public DriveToMiddleConeNode(Limelight limelight, SwerveSubsystem swerve) {
    this.limelight = limelight;
    this.swerve = swerve;

    this.doneDriving = false;

    // TODO(Simon): Add requirements for swerve and limelight.
  }

  @Override
  public void initialize() {
    /* Starts ScoreMiddleCone command up. */

    // TODO(Simon): Set proper limelight pipeline and turn on LEDs.

  }

  @Override
  public void execute() {
    // TODO(Simon): Fill in.

    // Get closest middle cone target.

    // Calculate X and Y speeds

    // Set swerve speeds.

    // Set doneDriving to True once at location
  }

  @Override
  public void end(boolean commandInterrupted) {
    // TODO(Simon): Fill in.

    // Set drive speeds to 0.

    // Turn off LEDs.
  }

  @Override
  public boolean isFinished() {
    return doneDriving;
  }
}
