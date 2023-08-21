// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.LimelightHelpers.LimelightResults;

public class Limelight extends SubsystemBase {

  public final String limelightName;

  public Limelight(String limelightName) {
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
    // TODO(Simon): Fill in the code to set the pipeline.

  }

  public VisionTarget getClosestMiddleConeTarget() {
    /* Return the closest middle cone target, as detected using retroreflective tape.
     *
     * This is done by isolating the largest target on the screen.
     */
    // TODO(Simon): Fill in the code to get the closest middle cone target.

    // Get results from Limelight.
    LimelightResults results = LimelightHelpers.getLatestResults(limelightName);

    // Iterate through the list of retro-reflective targets.
    for (int i = 0; i < results.targetingResults.targets_Retro.length; i++) {
      // Get the current target.

      // See if the current target is larger than the previous target.
    }

    // Return the largest (closest) target as object VisionTarget.
    return new VisionTarget(0, 0, 0, 0);
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
}
