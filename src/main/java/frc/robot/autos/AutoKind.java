// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;

public enum AutoKind {
  DO_NOTHING("", 0.01, 0.01, false),

  BLUE_SHORT_SIDE_3("BlueShortSide3", 4.5, 4, false),
  RED_SHORT_SIDE_3("RedShortSide3", 4.5, 4, false),

  BLUE_MID_BUMP_1_5_BALANCE("BlueMidBump1.5Balance", 2, 3, true),
  RED_MID_BUMP_1_5_BALANCE("RedMidBump1.5Balance", 2, 3, true),

  BLUE_MID_FLAT_1_5_BALANCE("BlueMidFlat1.5Balance", 2, 3, true),
  RED_MID_FLAT_1_5_BALANCE("RedMidFlat1.5Balance", 2, 3, true),

  BLUE_MID_FLAT_2_BALANCE("BlueMidFlat2Balance", 3, 3, true),
  BLUE_MID_BUMP_2_BALANCE("BlueMidBump2Balance", 3, 3, true),

  BLUE_LONG_SIDE_3("BlueLongSide3", 4, 4, false),
  RED_LONG_SIDE_3("RedLongSide3", 4, 4, false),

  BLUE_MID_1_BALANCE("BlueMid1Balance", 2, 3, true),
  RED_MID_1_BALANCE("RedMid1Balance", 2, 3, true),

  RED_MID_BUMP_2_BALANCE("RedMidBump2Balance", 3, 3, true),
  RED_MID_FLAT_2_BALANCE("RedMidFlat2Balance", 3, 3, true);

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
