// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

public enum AutoKindWithoutTeam {
  DO_NOTHING(AutoKind.DO_NOTHING, AutoKind.DO_NOTHING),

  SHORT_SIDE_3(AutoKind.RED_SHORT_SIDE_3, AutoKind.BLUE_SHORT_SIDE_3),
  LONG_SIDE_3(AutoKind.RED_SHORT_SIDE_3, AutoKind.BLUE_LONG_SIDE_3),

  MID_1_5_BALANCE(AutoKind.RED_MID_1_5_BALANCE, AutoKind.BLUE_MID_1_5_BALANCE),
  MID_1_BALANCE(AutoKind.RED_MID_1_BALANCE, AutoKind.BLUE_MID_1_BALANCE),

  MID_RIGHT_2_BALANCE(AutoKind.RED_MID_RIGHT_2_BALANCE, AutoKind.BLUE_MID_RIGHT_2_BALANCE),
  MID_LEFT_2_BALANCE(AutoKind.RED_MID_LEFT_2_BALANCE, AutoKind.BLUE_MID_LEFT_2_BALANCE);

  // LONG_SIDE_3_WORN(AutoKind.RED_SHORT_SIDE_3_WORN, AutoKind.BLUE_LONG_SIDE_3_WORN),
  // MID_1_5_BALANCE_WORN(AutoKind.RED_MID_1_5_BALANCE_WORN, AutoKind.BLUE_MID_1_5_BALANCE_WORN),
  // MID_RIGHT_2_BALANCE_WORN(AutoKind.RED_MID_RIGHT_2_BALANCE_WORN,
  // AutoKind.BLUE_MID_RIGHT_2_BALANCE_WORN),
  // MID_LEFT_2_BALANCE_WORN(AutoKind.RED_MID_LEFT_2_BALANCE_WORN,
  // AutoKind.BLUE_MID_LEFT_2_BALANCE_WORN),

  public final AutoKind redVersion;
  public final AutoKind blueVersion;

  private AutoKindWithoutTeam(AutoKind redVersion, AutoKind blueVersion) {
    this.redVersion = redVersion;
    this.blueVersion = blueVersion;
  }
}
