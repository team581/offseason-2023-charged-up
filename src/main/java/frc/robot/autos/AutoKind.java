// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;

public enum AutoKind {
  DO_NOTHING("", 0.01, 0.01, false),

  BLUE_LONG_SIDE_3("BlueLongSide3", 4.5, 4, false),
  BLUE_MID_1_5_BALANCE("BlueMid1.5Balance", 2, 3, true),
  BLUE_MID_RIGHT_2_BALANCE("BlueMidRight2Balance", 3, 3, true),
  BLUE_MID_LEFT_2_BALANCE("BlueMidLeft2Balance", 3, 3, true),

  BLUE_SHORT_SIDE_3("BlueShortSide3", 4.5, 4, false),
  BLUE_SHORT_SIDE_3_WORN("BlueShortSide3Worn", 4.5, 4, false),

  RED_MID_1_5_BALANCE("RedMid1.5Balance", 2, 3, true),
  RED_MID_LEFT_2_BALANCE("RedMid2Balance", 3, 3, true),
  RED_MID_LEFT_2_BALANCE_WORN("RedMid2BalanceWorn", 3, 3, true),

  RED_MID_RIGHT_2_BALANCE("RedMid2Balance", 3, 3, true),
  RED_MID_RIGHT_2_BALANCE_WORN("RedMid2BalanceWorn", 3, 3, true);

  public final String pathName;
  public final PathConstraints constraints;
  public final boolean autoBalance;

  private AutoKind(
      String pathName, double maxVelocity, double maxAcceleration, boolean autoBalance) {
    this.pathName = pathName;
    this.constraints = new PathConstraints(maxVelocity, maxAcceleration);
    this.autoBalance = autoBalance;
  }
}
