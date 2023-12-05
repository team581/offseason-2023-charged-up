// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.managers;

import frc.robot.intake.IntakeMode;
import java.util.Objects;

public class SuperstructureState {
  public final SuperstructurePosition position;
  public final IntakeMode intakeMode;
  public final boolean intakeNow;

  public SuperstructureState(
      SuperstructurePosition position, IntakeMode intakeMode, boolean intakeNow) {
    this.position = position;
    this.intakeMode = intakeMode;
    this.intakeNow = intakeNow;
  }

  public SuperstructureState(SuperstructurePosition position, IntakeMode intakeMode) {
    this(position, intakeMode, false);
  }

  @Override
  public int hashCode() {
    return Objects.hash(position, intakeMode, intakeNow);
  }

  @Override
  public boolean equals(Object obj) {
    if (obj == this) {
      return true;
    }

    if (!(obj instanceof SuperstructureState)) {
      return false;
    }

    SuperstructureState other = (SuperstructureState) obj;

    return position.equals(other.position)
        && intakeMode == other.intakeMode
        && intakeNow == other.intakeNow;
  }
}
