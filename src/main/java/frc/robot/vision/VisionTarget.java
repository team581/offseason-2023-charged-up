// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

public class VisionTarget {
  public double x;
  public double y;
  public double width;
  public double height;
  public boolean valid;

  public VisionTarget(double x, double y, double width, double height, boolean valid) {
    this.x = x;
    this.y = y;
    this.width = width;
    this.height = height;
    this.valid = valid;
  }
}
