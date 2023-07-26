// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.config.Config;
import frc.robot.managers.SuperstructurePosition;

public class Positions {
  public static final SuperstructurePosition STOWED =
      Config.IS_SPIKE
          ? new SuperstructurePosition(0, Rotation2d.fromDegrees(2), -1)
          : new SuperstructurePosition(0, Rotation2d.fromDegrees(35), -1);
  public static final SuperstructurePosition STOWED_UNSAFE =
      Config.IS_SPIKE
          ? new SuperstructurePosition(0, Rotation2d.fromDegrees(2), -1, true)
          : new SuperstructurePosition(0, Rotation2d.fromDegrees(35), -1, true);
  public static final SuperstructurePosition FULL_EXTENSION =
      new SuperstructurePosition(20, Rotation2d.fromDegrees(10), -1);

  public static final SuperstructurePosition INTAKING_CUBE_FLOOR =
      Config.IS_SPIKE
          ? new SuperstructurePosition(0, Rotation2d.fromDegrees(68), -1)
          : new SuperstructurePosition(0, Rotation2d.fromDegrees(154), -1);
  public static final SuperstructurePosition INTAKING_CUBE_SHELF =
      Config.IS_SPIKE
          ? new SuperstructurePosition(22, Rotation2d.fromDegrees(55), -1)
          : new SuperstructurePosition(0.5, Rotation2d.fromDegrees(145), -1);
  public static final SuperstructurePosition CUBE_NODE_LOW =
      Config.IS_SPIKE
          ? new SuperstructurePosition(0, Rotation2d.fromDegrees(50), -1)
          : new SuperstructurePosition(0.5, Rotation2d.fromDegrees(125), -1);
  public static final SuperstructurePosition CUBE_NODE_MID =
      Config.IS_SPIKE
          ? new SuperstructurePosition(12, Rotation2d.fromDegrees(50), -1)
          : new SuperstructurePosition(17, Rotation2d.fromDegrees(146), -1);
  public static final SuperstructurePosition CUBE_NODE_HIGH =
      Config.IS_SPIKE
          ? new SuperstructurePosition(25, Rotation2d.fromDegrees(50), -1)
          : new SuperstructurePosition(28, Rotation2d.fromDegrees(157), -1);

  public static final SuperstructurePosition INTAKING_CONE_FLOOR =
      Config.IS_SPIKE
          ? new SuperstructurePosition(4.75, Rotation2d.fromDegrees(85), -1)
          : new SuperstructurePosition(3.39, Rotation2d.fromDegrees(161), -1);
  public static final SuperstructurePosition INTAKING_CONE_SHELF =
      Config.IS_SPIKE
          ? new SuperstructurePosition(24.5, Rotation2d.fromDegrees(75), -1)
          : new SuperstructurePosition(1, Rotation2d.fromDegrees(143), -1);
  public static final SuperstructurePosition INTAKING_CONE_SINGLE_SUBSTATION =
      Config.IS_SPIKE
          ? new SuperstructurePosition(1, Rotation2d.fromDegrees(30), -1)
          : new SuperstructurePosition(0, Rotation2d.fromDegrees(35), -1);
  public static final SuperstructurePosition CONE_NODE_LOW =
      Config.IS_SPIKE
          ? new SuperstructurePosition(0, Rotation2d.fromDegrees(65), -1)
          : new SuperstructurePosition(0.5, Rotation2d.fromDegrees(145), -1);
  public static final SuperstructurePosition CONE_NODE_MID =
      Config.IS_SPIKE
          ? new SuperstructurePosition(19, Rotation2d.fromDegrees(70), -1)
          : new SuperstructurePosition(25, Rotation2d.fromDegrees(175), -1);
  public static final SuperstructurePosition CONE_NODE_HIGH =
      Config.IS_SPIKE
          ? new SuperstructurePosition(25.6, Rotation2d.fromDegrees(65), -1)
          : new SuperstructurePosition(31, Rotation2d.fromDegrees(155), -1);

  private Positions() {}
}
