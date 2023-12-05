// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Objects;

public class SuperstructurePosition {
  public final double height;
  public final Rotation2d angle;
  public final double earlyTransitionHeight;
  public final boolean skipCollisionAvoidance;

  public SuperstructurePosition(double height, Rotation2d angle) {
    this(height, angle, -1, false);
  }

  public SuperstructurePosition(double height, Rotation2d angle, boolean skipCollisionAvoidance) {
    this(height, angle, -1, skipCollisionAvoidance);
  }

  public SuperstructurePosition(double height, Rotation2d angle, double earlyTransitionHeight) {
    this(height, angle, earlyTransitionHeight, false);
  }

  public SuperstructurePosition(
      double height,
      Rotation2d angle,
      double earlyTransitionHeight,
      boolean skipCollisionAvoidance) {
    this.height = height;
    this.angle = angle;
    this.earlyTransitionHeight = earlyTransitionHeight;
    this.skipCollisionAvoidance = skipCollisionAvoidance;
  }

  @Override
  public boolean equals(Object obj) {
    if (obj == this) {
      return true;
    }

    if (!(obj instanceof SuperstructurePosition)) {
      return false;
    }

    SuperstructurePosition other = (SuperstructurePosition) obj;

    return height == other.height
        && angle.equals(other.angle)
        && earlyTransitionHeight == other.earlyTransitionHeight
        && skipCollisionAvoidance == other.skipCollisionAvoidance;
  }

  @Override
  public int hashCode() {
    return Objects.hash(height, angle, earlyTransitionHeight, skipCollisionAvoidance);
  }
}
