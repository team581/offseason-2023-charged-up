// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.LimelightHelpers.LimelightResults;
import frc.robot.vision.LimelightHelpers.LimelightTarget_Retro;

public class LimelightSubsystem extends LifecycleSubsystem {
  public final String limelightName;
  public final int retroPipeline =2;

  public LimelightSubsystem(String limelightName) {
    super(SubsystemPriority.VISION, "LimelightSubsystem_" + limelightName);

    /* Constructor method. */
    this.limelightName = limelightName;
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
    // Iterate through the list of retro-reflective targets.
    for (int i = 0; i < results.targetingResults.targets_Retro.length; i++) {
      // Get the current target.
      LimelightTarget_Retro currentTarget = results.targetingResults.targets_Retro[i];
      // See if the current target is larger than the previous target.
      if (currentTarget.ta > biggestTarget.ta) {
        biggestTarget = currentTarget;
      }
    }

    // Return the largest (closest) target as object VisionTarget.
    return new VisionTarget(biggestTarget.tx_pixels, biggestTarget.ty_pixels, 0, 0);
  }

  public VisionTarget getClosestConeTarget() {
    /* Return the closest cone target, as detected with the Coral.
     *
     * This is done by isolating the largest target on the screen.
     */
    // TODO(Simon): Fill in the code to get the closest cone target.

    // Get results */
    return new VisionTarget(0, 0, 0, 0);
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
}
