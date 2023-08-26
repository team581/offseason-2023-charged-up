// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.Config;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends LifecycleSubsystem {

  private HeldGamePiece gamePiece = HeldGamePiece.NOTHING;

  private IntakeMode mode = IntakeMode.STOPPED;

  private final TalonFX motor;

  private final Debouncer coneFilterSensor = new Debouncer(10 * 0.02, DebounceType.kBoth);
  private final Debouncer cubeFilterSensor = new Debouncer(10 * 0.02, DebounceType.kBoth);

  public IntakeSubsystem(TalonFX motor) {
    super(SubsystemPriority.INTAKE);

    this.motor = motor;

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.Inverted =
        Config.INVERTED_INTAKE
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    motorConfig.CurrentLimits.SupplyCurrentLimit = 15;
    motorConfig.CurrentLimits.SupplyCurrentThreshold = 25;
    motorConfig.CurrentLimits.SupplyTimeThreshold = 0.2;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfig.HardwareLimitSwitch.ReverseLimitEnable = false;
  }

  private boolean sensorHasCube() {
    return motor.getForwardLimit().getValue() == ForwardLimitValue.ClosedToGround;
  }

  private boolean sensorHasCone() {
    return motor.getReverseLimit().getValue() == ReverseLimitValue.Open;
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Intake/Mode", mode.toString());
    Logger.getInstance().recordOutput("Intake/HeldGamePiece", gamePiece.toString());
    Logger.getInstance().recordOutput("Intake/Current", motor.getStatorCurrent().getValue());
    Logger.getInstance().recordOutput("Intake/AppliedDutyCycle", motor.getDutyCycle().getValue());
    Logger.getInstance().recordOutput("Intake/ConeIntakeSensor", sensorHasCone());
    Logger.getInstance().recordOutput("Intake/CubeIntakeSensor", sensorHasCube());
  }

  @Override
  public void enabledPeriodic() {

    boolean coneSensor = coneFilterSensor.calculate(sensorHasCone());
    boolean cubeSensor = cubeFilterSensor.calculate(sensorHasCube());
    Logger.getInstance().recordOutput("Intake/FilteredConeIntakeSensor", coneSensor);
    Logger.getInstance().recordOutput("Intake/FilteredCubeIntakeSensor", cubeSensor);

    if (mode == IntakeMode.INTAKE_CUBE) {
      if (cubeSensor) {
        gamePiece = HeldGamePiece.CUBE;
      }
    } else if (mode == IntakeMode.INTAKE_CONE) {
      if (coneSensor) {
        gamePiece = HeldGamePiece.CONE;
      }
    } else if (mode == IntakeMode.OUTTAKE_CUBE_FAST || mode == IntakeMode.OUTTAKE_CUBE_SLOW) {
      if (!cubeSensor) {
        gamePiece = HeldGamePiece.NOTHING;
      }
    } else if (mode == IntakeMode.OUTTAKE_CONE || mode == IntakeMode.SHOOT_CONE) {
      if (!coneSensor) {
        gamePiece = HeldGamePiece.NOTHING;
      }
    }

    if (mode == IntakeMode.MANUAL_INTAKE) {
      motor.set(0.5);
    } else if (mode == IntakeMode.MANUAL_OUTTAKE) {
      motor.set(-0.5);
    } else if (mode == IntakeMode.OUTTAKE_CUBE_SLOW) {
      motor.set(-0.3);
    } else if (mode == IntakeMode.OUTTAKE_CUBE_FAST) {
      motor.set(-0.5);
    } else if (mode == IntakeMode.OUTTAKE_CONE) {
      motor.set(0.6);
    } else if (mode == IntakeMode.SHOOT_CONE) {
      motor.set(1);
    } else if (gamePiece == HeldGamePiece.CUBE) {
      motor.set(0.15);
    } else if (gamePiece == HeldGamePiece.CONE) {
      motor.set(-0.1);
    } else if (mode == IntakeMode.INTAKE_CUBE) {
      motor.set(0.75);
    } else if (mode == IntakeMode.INTAKE_CONE) {
      motor.set(-1);
    } else {
      motor.disable();
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
        || mode == IntakeMode.OUTTAKE_CUBE_SLOW) {
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
}
