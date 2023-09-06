// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.LimelightHelpers.LimelightResults;
import frc.robot.vision.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.vision.LimelightHelpers.LimelightTarget_Retro;
import org.littletonrobotics.junction.Logger;

public class LimelightSubsystem extends LifecycleSubsystem {
  public final String limelightName;
  public final int retroPipeline = 1;
  private SwerveSubsystem swerve;

  public LimelightSubsystem(String limelightName, SwerveSubsystem swerve) {
    super(SubsystemPriority.VISION, "LimelightSubsystem_" + limelightName);

    /* Constructor method. */
    this.limelightName = limelightName;
    this.swerve = swerve;
  }

  public void turnOnLights() {
    /* Turns on LEDs on the limelight. */
    LimelightHelpers.setLEDMode_ForceOn(limelightName);
  }

  public void turnOffLights() {
    /* Turns off LEDs on the limelight. */
    LimelightHelpers.setLEDMode_ForceOff(limelightName);
  }

  public void setPipeline(int index) {
    /* Set the Pipeline to use on the limelight. */
    LimelightHelpers.setPipelineIndex(limelightName, index);
  }

  public VisionTarget getClosestMiddleConeTarget() {
    /* Return the closest middle cone target, as detected using retroreflective tape.
     *
     * This is done by isolating the largest target on the screen.
     */

    // Get results from Limelight.
    LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
    LimelightTarget_Retro biggestTarget = new LimelightTarget_Retro();

    if (results.targetingResults.targets_Retro.length == 0) {
      VisionTarget.valid = false;
    } else {
      VisionTarget.valid = true;
    }
    // Iterate through the list of retro-reflective targets.
    for (int i = 0; i < results.targetingResults.targets_Retro.length; i++) {
      // Get the current target.
      LimelightTarget_Retro currentTarget = results.targetingResults.targets_Retro[i];
      // See if the current target is larger than the previous target.
      if (currentTarget.ta > biggestTarget.ta) {
        biggestTarget = currentTarget;
      }
    }

    return new VisionTarget(
        biggestTarget.tx,
        biggestTarget.ty,
        0,
        0,
        results.targetingResults.targets_Retro.length > 0);
  }

  public VisionTarget getClosestConeTarget() {
    /* Return the closest cone target, as detected with the Coral.
     *
     * This is done by isolating the largest target on the screen.
     */
    // TODO(Simon): Fill in the code to get the closest cone target.
    LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
    LimelightTarget_Detector biggestTarget = new LimelightTarget_Detector();

    if (results.targetingResults.targets_Detector.length == 0) {
      VisionTarget.valid = false;
    } else {
      VisionTarget.valid = true;
    }

    for (int i = 0; i < results.targetingResults.targets_Detector.length; i++) {
      LimelightTarget_Detector currentTarget = results.targetingResults.targets_Detector[i];

      if (currentTarget.ta > biggestTarget.ta) {
        biggestTarget = currentTarget;
      }
    }
    // Get results */
    return new VisionTarget(
        biggestTarget.tx,
        biggestTarget.ty,
        0,
        0,
        results.targetingResults.targets_Detector.length > 0);
  }

  public CommandBase setPipelineCommand(int pipeline) {
    return runOnce(
        () -> {
          // TODO(Simon): Set proper limelight pipeline and turn on LEDs.
          setPipeline(pipeline);

          if (pipeline == retroPipeline) {
            turnOnLights();
          } else {
            turnOffLights();
          }
        });
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Vision/LimelightX", getClosestMiddleConeTarget().x);
    Logger.getInstance().recordOutput("Vision/LimelightY", getClosestMiddleConeTarget().y);
    Logger.getInstance().recordOutput("Vision/XSpeed", swerve.getChassisSpeeds().vxMetersPerSecond);
    Logger.getInstance().recordOutput("Vision/YSpeed", swerve.getChassisSpeeds().vyMetersPerSecond);
  }
}
