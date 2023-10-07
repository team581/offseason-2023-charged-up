// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.Config;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends LifecycleSubsystem {
  private static final IntakeDetectionMode CONE_DETECTION_MODE = IntakeDetectionMode.SENSOR;
  private static final IntakeDetectionMode CUBE_DETECTION_MODE = IntakeDetectionMode.SENSOR;

  private static final SupplyCurrentLimitConfiguration CURRENT_LIMIT =
      new SupplyCurrentLimitConfiguration(true, 15, 25, 0.2);

  private HeldGamePiece gamePiece = HeldGamePiece.NOTHING;

  private IntakeMode mode = IntakeMode.STOPPED;

  private final TalonFX motor;

  private final Debouncer coneFilterSensor = new Debouncer(10 * 0.02, DebounceType.kBoth);
  private final Debouncer cubeFilterSensor = new Debouncer(10 * 0.02, DebounceType.kBoth);
  private boolean coneSensor = false;
  private boolean cubeSensor = false;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(10);
  private double filteredCurrent = -1;

  public IntakeSubsystem(TalonFX motor) {
    super(SubsystemPriority.INTAKE);

    this.motor = motor;
    motor.setInverted(Config.INVERTED_INTAKE);
    motor.configSupplyCurrentLimit(CURRENT_LIMIT);
    motor.overrideLimitSwitchesEnable(false);
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Intake/ConeDetectionMode", CONE_DETECTION_MODE.toString());
    Logger.getInstance().recordOutput("Intake/CubeDetectionMode", CUBE_DETECTION_MODE.toString());
    Logger.getInstance().recordOutput("Intake/Mode", mode.toString());

    Logger.getInstance().recordOutput("Intake/HeldGamePiece", gamePiece.toString());

    Logger.getInstance().recordOutput("Intake/StatorCurrent", motor.getStatorCurrent());
    Logger.getInstance().recordOutput("Intake/Voltage", motor.getMotorOutputVoltage());
    Logger.getInstance().recordOutput("Intake/ConeIntakeSensor", sensorHasCone());
    Logger.getInstance().recordOutput("Intake/CubeIntakeSensor", sensorHasCube());

    // This runs before enabledPeriodic(), so it's fine to update the values here
    filteredCurrent = currentFilter.calculate(motor.getStatorCurrent());
    coneSensor = coneFilterSensor.calculate(sensorHasCone());
    cubeSensor = cubeFilterSensor.calculate(sensorHasCube());

    Logger.getInstance().recordOutput("Intake/FilteredStatorCurrent", filteredCurrent);
    Logger.getInstance().recordOutput("Intake/FilteredConeIntakeSensor", coneSensor);
    Logger.getInstance().recordOutput("Intake/FilteredCubeIntakeSensor", cubeSensor);
  }

  @Override
  public void enabledPeriodic() {
    if (mode == IntakeMode.INTAKE_CUBE
        || mode == IntakeMode.OUTTAKE_CUBE_FAST
        || mode == IntakeMode.OUTTAKE_CUBE_SLOW
        || mode == IntakeMode.YEET_CUBE) {
      if (hasCube()) {
        gamePiece = HeldGamePiece.CUBE;
      } else {
        gamePiece = HeldGamePiece.NOTHING;
      }
    }

    if (mode == IntakeMode.INTAKE_CONE
        || mode == IntakeMode.OUTTAKE_CONE
        || mode == IntakeMode.SHOOT_CONE) {
      if (hasCone()) {
        gamePiece = HeldGamePiece.CONE;
      } else {
        gamePiece = HeldGamePiece.NOTHING;
      }
    }

    if (mode == IntakeMode.MANUAL_INTAKE) {
      motor.set(TalonFXControlMode.PercentOutput, 0.5);
    } else if (mode == IntakeMode.MANUAL_OUTTAKE) {
      motor.set(TalonFXControlMode.PercentOutput, -0.5);
    } else if (mode == IntakeMode.OUTTAKE_CUBE_SLOW) {
      motor.set(TalonFXControlMode.PercentOutput, -0.3);
    } else if (mode == IntakeMode.OUTTAKE_CUBE_FAST) {
      motor.set(TalonFXControlMode.PercentOutput, -0.5);
    } else if (mode == IntakeMode.YEET_CUBE) {
      motor.set(TalonFXControlMode.PercentOutput, -1);
    } else if (mode == IntakeMode.OUTTAKE_CONE) {
      motor.set(TalonFXControlMode.PercentOutput, 0.6);
    } else if (mode == IntakeMode.SHOOT_CONE) {
      motor.set(TalonFXControlMode.PercentOutput, 1);
    } else if (gamePiece == HeldGamePiece.CUBE) {
      motor.set(TalonFXControlMode.PercentOutput, 0.15);
    } else if (gamePiece == HeldGamePiece.CONE) {
      motor.set(TalonFXControlMode.PercentOutput, -0.1);
    } else if (mode == IntakeMode.INTAKE_CUBE) {
      motor.set(TalonFXControlMode.PercentOutput, 0.75);
    } else if (mode == IntakeMode.INTAKE_CONE) {
      motor.set(TalonFXControlMode.PercentOutput, -1);
    } else {
      motor.set(TalonFXControlMode.PercentOutput, 0);
    }
  }

  public void setMode(IntakeMode mode) {
    if (this.mode != mode && (mode == IntakeMode.INTAKE_CONE || mode == IntakeMode.INTAKE_CUBE)) {
      gamePiece = HeldGamePiece.NOTHING;
    }
    this.mode = mode;
  }

  public IntakeMode getMode() {
    return mode;
  }

  public boolean atGoal(IntakeMode goal) {
    if (mode != goal) {
      return false;
    }
    if (mode == IntakeMode.OUTTAKE_CONE
        || mode == IntakeMode.SHOOT_CONE
        || mode == IntakeMode.OUTTAKE_CUBE_FAST
        || mode == IntakeMode.OUTTAKE_CUBE_SLOW
        || mode == IntakeMode.YEET_CUBE) {
      return gamePiece == HeldGamePiece.NOTHING;
    }
    if (mode == IntakeMode.STOPPED) {
      return true;
    }
    if (mode == IntakeMode.INTAKE_CUBE) {
      return gamePiece == HeldGamePiece.CUBE;
    }
    if (mode == IntakeMode.INTAKE_CONE) {
      return gamePiece == HeldGamePiece.CONE;
    }
    if (mode == IntakeMode.MANUAL_INTAKE || mode == IntakeMode.MANUAL_OUTTAKE) {
      return false;
    }
    return false;
  }

  public HeldGamePiece getGamePiece() {
    return gamePiece;
  }

  public void setGamePiece(HeldGamePiece gamePiece) {
    this.gamePiece = gamePiece;
  }

  public Command getCommand(IntakeMode newGoal) {
    return runOnce(() -> setMode(newGoal))
        .andThen(Commands.waitUntil(() -> atGoal(newGoal)))
        .andThen(Commands.runOnce(() -> setMode(IntakeMode.STOPPED)));
  }

  public boolean hasCube() {
    if (CUBE_DETECTION_MODE == IntakeDetectionMode.SENSOR) {
      return cubeSensor;
    } else if (CUBE_DETECTION_MODE == IntakeDetectionMode.CURRENT) {
      if (mode == IntakeMode.INTAKE_CUBE) {
        return filteredCurrent > 15;
      }

      if (mode == IntakeMode.OUTTAKE_CUBE_FAST || mode == IntakeMode.OUTTAKE_CUBE_SLOW) {
        return filteredCurrent < 10;
      }

      if (mode == IntakeMode.YEET_CUBE) {
        return filteredCurrent < 15;
      }
    }

    return false;
  }

  public boolean hasCone() {
    if (CONE_DETECTION_MODE == IntakeDetectionMode.SENSOR) {
      return coneSensor;
    } else if (CONE_DETECTION_MODE == IntakeDetectionMode.CURRENT) {
      if (mode == IntakeMode.INTAKE_CONE) {
        return filteredCurrent > 25;
      }

      if (mode == IntakeMode.OUTTAKE_CONE) {
        return filteredCurrent < 10;
      }

      if (mode == IntakeMode.SHOOT_CONE) {
        return filteredCurrent < 15;
      }
    }

    return false;
  }

  private boolean sensorHasCube() {
    return motor.isFwdLimitSwitchClosed() == 1;
  }

  private boolean sensorHasCone() {
    return motor.isRevLimitSwitchClosed() == 1;
  }
}
