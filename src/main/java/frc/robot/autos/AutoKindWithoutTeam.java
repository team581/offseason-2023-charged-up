// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

public enum AutoKindWithoutTeam {
  DO_NOTHING(AutoKind.DO_NOTHING, AutoKind.DO_NOTHING),

  MID_1_BALANCE(AutoKind.RED_MID_1_BALANCE, AutoKind.BLUE_MID_1_BALANCE),
  MID_BUMP_1_5_BALANCE(AutoKind.RED_MID_BUMP_1_5_BALANCE, AutoKind.BLUE_MID_BUMP_1_5_BALANCE),
  MID_FLAT_1_5_BALANCE(AutoKind.RED_MID_FLAT_1_5_BALANCE, AutoKind.BLUE_MID_FLAT_1_5_BALANCE),

  MID_BUMP_2_BALANCE(AutoKind.RED_MID_BUMP_2_BALANCE, AutoKind.BLUE_MID_BUMP_2_BALANCE),
  MID_FLAT_2_BALANCE(AutoKind.RED_MID_FLAT_2_BALANCE, AutoKind.BLUE_MID_FLAT_2_BALANCE),

  FLAT_3(AutoKind.RED_SHORT_SIDE_3, AutoKind.BLUE_SHORT_SIDE_3),
  FLAT_3_YEET(AutoKind.RED_SHORT_SIDE_3_YEET, AutoKind.BLUE_SHORT_SIDE_3_YEET),
  BUMP_3(AutoKind.RED_LONG_SIDE_3, AutoKind.BLUE_LONG_SIDE_3),
  BUMP_3_BALANCE(AutoKind.RED_LONG_SIDE_3_BALANCE, AutoKind.BLUE_LONG_SIDE_3_BALANCE),
  BUMP_3_5(AutoKind.RED_LONG_SIDE_3_5, AutoKind.BLUE_LONG_SIDE_3_5);

  public final AutoKind redVersion;
  public final AutoKind blueVersion;

  private AutoKindWithoutTeam(AutoKind redVersion, AutoKind blueVersion) {
    this.redVersion = redVersion;
    this.blueVersion = blueVersion;
  }
}
