// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.managers.SuperstructurePosition;

public class Positions {
  public static final SuperstructurePosition STOWED =
      new SuperstructurePosition(0.2, Rotation2d.fromDegrees(2));
  public static final SuperstructurePosition STOWED_UNSAFE =
      new SuperstructurePosition(0.1, Rotation2d.fromDegrees(2), true);

  public static final SuperstructurePosition INTAKING_CUBE_FLOOR =
      new SuperstructurePosition(1, Rotation2d.fromDegrees(80), true);
  public static final SuperstructurePosition INTAKING_CUBE_SHELF =
      new SuperstructurePosition(22, Rotation2d.fromDegrees(55));

  public static final SuperstructurePosition CUBE_NODE_LOW =
      new SuperstructurePosition(0, Rotation2d.fromDegrees(50));
  public static final SuperstructurePosition CUBE_NODE_MID =
      new SuperstructurePosition(12, Rotation2d.fromDegrees(50));
  public static final SuperstructurePosition CUBE_NODE_HIGH =
      new SuperstructurePosition(23, Rotation2d.fromDegrees(50));

  public static final SuperstructurePosition CUBE_SHOOT_MID =
      new SuperstructurePosition(10, Rotation2d.fromDegrees(2));
  public static final SuperstructurePosition CONE_SHOOT_MID =
      new SuperstructurePosition(12, Rotation2d.fromDegrees(2));

  public static final SuperstructurePosition INTAKING_CONE_FLOOR =
      new SuperstructurePosition(5, Rotation2d.fromDegrees(92), true);
  public static final SuperstructurePosition INTAKING_CONE_SHELF =
      new SuperstructurePosition(26.9, Rotation2d.fromDegrees(92));
  public static final SuperstructurePosition INTAKING_CONE_SHELF_EVIL =
      new SuperstructurePosition(9, Rotation2d.fromDegrees(35));
  public static final SuperstructurePosition INTAKING_CONE_SINGLE_SUBSTATION =
      new SuperstructurePosition(1, Rotation2d.fromDegrees(30));

  public static final SuperstructurePosition CONE_NODE_LOW =
      new SuperstructurePosition(0, Rotation2d.fromDegrees(65), true);
  public static final SuperstructurePosition CONE_NODE_MID =
      new SuperstructurePosition(12, Rotation2d.fromDegrees(52));
  public static final SuperstructurePosition CONE_NODE_HIGH =
      new SuperstructurePosition(24, Rotation2d.fromDegrees(72));
  public static final SuperstructurePosition AUTO_CONE_NODE_HIGH =
      new SuperstructurePosition(24, Rotation2d.fromDegrees(72), -1);

  private Positions() {}
}
