// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
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
  // TODO: Change this to the correct pipeline index
  public static final int HELD_CONE_PIPELINE = 999;
  // TODO: Tune this value to convert held cone angle (relative to Limelight) to robot angle
  public static final double HELD_CONE_X_OFFSET_ANGLE_SCALAR = 1;
  private final SwerveSubsystem swerve;

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

  /** Get the X offset for scoring a held cone. */
  public double getHeldConeXOffset() {
    setPipeline(HELD_CONE_PIPELINE);

    LimelightResults results = LimelightHelpers.getLatestResults(limelightName);

    if (results.targetingResults.targets_Retro.length == 0) {
      return 0;
    }

    LimelightTarget_Retro rawTarget = results.targetingResults.targets_Retro[0];

    VisionTarget visionTarget = new VisionTarget(rawTarget, true);

    return visionTarget.x * HELD_CONE_X_OFFSET_ANGLE_SCALAR;
  }

  public VisionTarget getClosestMiddleConeTarget() {
    /* Return the closest middle cone target, as detected using retroreflective tape.
     *
     * This is done by isolating the closest target to (0°, -24.85°).
     */

    // Get results from Limelight.
    LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
    LimelightTarget_Retro closestTarget = new LimelightTarget_Retro();
    double closestDistance = 10000; // Some large number to evict once we have real results.

    // Iterate through the list of retro-reflective targets.
    for (int i = 0; i < results.targetingResults.targets_Retro.length; i++) {
      // Get the current target.
      LimelightTarget_Retro currentTarget = results.targetingResults.targets_Retro[i];
      // Calculate distance from currentTarget to  (0°, -24.85°)
      double distance = Math.pow(0 - currentTarget.tx, 2) + Math.pow(-24.85 - currentTarget.ty, 2);

      // If distance is less than the closestDistance, reassign closestTarget and closestDistance.
      if (distance < closestDistance) {
        closestDistance = distance;
        closestTarget = currentTarget;
      }
    }
    Logger.getInstance().recordOutput("Vision/NodeDistance", closestDistance);
    Logger.getInstance().recordOutput("Vision/NodeX", closestTarget.tx_pixels);
    Logger.getInstance().recordOutput("Vision/NodeY", closestTarget.ty_pixels);
    Logger.getInstance()
        .recordOutput("Vision/NodeLength", results.targetingResults.targets_Retro.length);
    return new VisionTarget(
        closestTarget.tx,
        closestTarget.ty,
        0,
        0,
        results.targetingResults.targets_Retro.length > 0);
  }

  public VisionTarget getClosestGroundCone() {
    /* Return the closest middle cone target, as detected using retroreflective tape.
     *
     * This is done by isolating the largest target on the screen.
     */
    // TODO(Simon): Fill in the code to get the closest cone target.
    LimelightResults results = LimelightHelpers.getLatestResults(limelightName);
    LimelightTarget_Detector biggestTarget = new LimelightTarget_Detector();

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
            })
        // Wait until Limelight has acknowleged the pipeline change
        .andThen(
            Commands.waitUntil(
                () -> LimelightHelpers.getCurrentPipelineIndex(limelightName) == pipeline))
        .withTimeout(0.25);
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Vision/LimelightX", getClosestMiddleConeTarget().x);
    Logger.getInstance().recordOutput("Vision/LimelightY", getClosestMiddleConeTarget().y);
    Logger.getInstance().recordOutput("Vision/XSpeed", swerve.getChassisSpeeds().vxMetersPerSecond);
    Logger.getInstance().recordOutput("Vision/YSpeed", swerve.getChassisSpeeds().vyMetersPerSecond);
    Logger.getInstance().recordOutput("Vision/NodeX", getClosestGroundCone().x);
    Logger.getInstance().recordOutput("Vision/NodeY", getClosestGroundCone().y);
  }
}
