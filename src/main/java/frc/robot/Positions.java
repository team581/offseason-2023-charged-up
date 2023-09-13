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
          ? new SuperstructurePosition(0.2, Rotation2d.fromDegrees(2))
          : new SuperstructurePosition(0, Rotation2d.fromDegrees(35));
  public static final SuperstructurePosition STOWED_UNSAFE =
      Config.IS_SPIKE
          ? new SuperstructurePosition(0.1, Rotation2d.fromDegrees(2), true)
          : new SuperstructurePosition(0, Rotation2d.fromDegrees(35), true);

  public static final SuperstructurePosition INTAKING_CUBE_FLOOR =
      Config.IS_SPIKE
          ? new SuperstructurePosition(1, Rotation2d.fromDegrees(80), true)
          : new SuperstructurePosition(0, Rotation2d.fromDegrees(154));
  public static final SuperstructurePosition INTAKING_CUBE_SHELF =
      Config.IS_SPIKE
          ? new SuperstructurePosition(22, Rotation2d.fromDegrees(55))
          : new SuperstructurePosition(0.5, Rotation2d.fromDegrees(145));
  public static final SuperstructurePosition CUBE_NODE_LOW =
      Config.IS_SPIKE
          ? new SuperstructurePosition(0, Rotation2d.fromDegrees(50))
          : new SuperstructurePosition(0.5, Rotation2d.fromDegrees(125));
  public static final SuperstructurePosition CUBE_NODE_MID =
      Config.IS_SPIKE
          ? new SuperstructurePosition(12, Rotation2d.fromDegrees(50))
          : new SuperstructurePosition(17, Rotation2d.fromDegrees(146));
  public static final SuperstructurePosition CUBE_NODE_HIGH =
      Config.IS_SPIKE
          ? new SuperstructurePosition(23, Rotation2d.fromDegrees(50))
          : new SuperstructurePosition(28, Rotation2d.fromDegrees(157));
  public static final SuperstructurePosition YEET_CUBE_MID =
      new SuperstructurePosition(3, Rotation2d.fromDegrees(20));

  public static final SuperstructurePosition INTAKING_CONE_FLOOR =
      Config.IS_SPIKE
          ? new SuperstructurePosition(5, Rotation2d.fromDegrees(92), true)
          : new SuperstructurePosition(3.39, Rotation2d.fromDegrees(161));
  public static final SuperstructurePosition INTAKING_CONE_SHELF =
      Config.IS_SPIKE
          ? new SuperstructurePosition(26.9, Rotation2d.fromDegrees(92))
          : new SuperstructurePosition(1, Rotation2d.fromDegrees(143));
  public static final SuperstructurePosition INTAKING_CONE_SINGLE_SUBSTATION =
      Config.IS_SPIKE
          ? new SuperstructurePosition(1, Rotation2d.fromDegrees(30))
          : new SuperstructurePosition(0, Rotation2d.fromDegrees(35));
  public static final SuperstructurePosition CONE_NODE_LOW =
      Config.IS_SPIKE
          ? new SuperstructurePosition(0, Rotation2d.fromDegrees(65), true)
          : new SuperstructurePosition(0.5, Rotation2d.fromDegrees(145));
  public static final SuperstructurePosition CONE_NODE_MID =
      Config.IS_SPIKE
          ? new SuperstructurePosition(12.75, Rotation2d.fromDegrees(52))
          : new SuperstructurePosition(25, Rotation2d.fromDegrees(175));
  public static final SuperstructurePosition CONE_NODE_HIGH =
      Config.IS_SPIKE
          ? new SuperstructurePosition(26.3, Rotation2d.fromDegrees(72))
          : new SuperstructurePosition(31, Rotation2d.fromDegrees(155));

  private Positions() {}
}
