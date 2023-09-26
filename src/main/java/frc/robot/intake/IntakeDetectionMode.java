// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

/** Used to configure how a type of game piece should be detected. */
public enum IntakeDetectionMode {
  /** Use sensors on the intake for detection. */
  SENSOR,
  /** Use motor stator current for detection. */
  CURRENT,
  /** No automatic detection. */
  NONE;
}
