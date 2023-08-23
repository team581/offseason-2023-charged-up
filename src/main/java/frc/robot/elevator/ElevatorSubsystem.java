// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Positions;
import frc.robot.config.Config;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.sensors.SensorUnitConverter;

import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends LifecycleSubsystem {
  private static final double TOLERANCE = 0.5;
  private static final double HOMING_CURRENT = 5;
  private static final double ROTATIONS_PER_ELEVATOR_INCH = 1.0/ (1.75 * Math.PI);

  private final TalonFX motor;
  private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

  private double goalPositionInInches = Positions.STOWED.height;
  private HomingState homingState = HomingState.NOT_HOMED;


  public ElevatorSubsystem(TalonFX motor) {
    super(SubsystemPriority.ELEVATOR);

    this.motor = motor;

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.Slot0.kV = Config.ELEVATOR_KV;
    motorConfig.Slot0.kP = Config.ELEVATOR_KP;
    motorConfig.Slot0.kI = Config.ELEVATOR_KI;
    motorConfig.Slot0.kD = Config.ELEVATOR_KD;
    motorConfig.Slot0.kS = Config.ELEVATOR_KS;

    motorConfig.MotorOutput.Inverted =Config.ELEVATOR_INVERTED? InvertedValue.Clockwise_Positive:InvertedValue.CounterClockwise_Positive;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = Config.ELEVATOR_CRUISE_VELOCITY;
    motorConfig.MotionMagic.MotionMagicAcceleration = Config.ELEVATOR_ACCELERATION;

    motorConfig.CurrentLimits.StatorCurrentLimit = 80;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    motorConfig.CurrentLimits.SupplyCurrentLimit = 20;
    motorConfig.CurrentLimits.SupplyCurrentThreshold = 30;
    motorConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConfig.Feedback.SensorToMechanismRatio = Config.ELEVATOR_GEARING;

    motor.getConfigurator().apply(motorConfig);
  }

  public void startHoming() {
    homingState = HomingState.HOMING;
  }

  public void resetHoming() {
    homingState = HomingState.NOT_HOMED;
    motor.disable();
  }

  public HomingState getHomingState() {
    return homingState;
  }

  public double getHeight() {
    double rotations = motor.getRotorPosition().getValue();
    double position = rotations / ROTATIONS_PER_ELEVATOR_INCH;
    return position;
  }

  public boolean atHeight(double height) {
    // Edit atHeight tolerance
    if (Math.abs(getHeight() - height) < TOLERANCE) {
      return true;
    } else {
      return false;
    }
  }

  public void setGoalPosition(double goal) {
    // Save goal position
    this.goalPositionInInches =
        MathUtil.clamp(goal, Config.ELEVATOR_MIN_HEIGHT, Config.ELEVATOR_MAX_HEIGHT);
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Elevator/Position", getHeight());
    Logger.getInstance().recordOutput("Elevator/StatorCurrent", motor.getStatorCurrent().getValue());
    Logger.getInstance().recordOutput("Elevator/SupplyCurrent", motor.getSupplyCurrent().getValue());
    Logger.getInstance().recordOutput("Elevator/GoalPosition", goalPositionInInches);
    Logger.getInstance().recordOutput("Elevator/Homing", homingState.toString());
    Logger.getInstance().recordOutput("Elevator/AppliedDutyCycle", motor.getDutyCycle().getValue());
    Logger.getInstance().recordOutput("Elevator/SensorVelocity", motor.getVelocity().getValue());
    Logger.getInstance().recordOutput("Elevator/TemperatureCelsius", motor.getDeviceTemp().getValue());
  }

  @Override
  public void enabledPeriodic() {
    // Add homing sequence
    // Convert goal in inches to sensor units, and set motor
    if (homingState == HomingState.HOMING) {
      motor.set(-0.15);
      double current = motor.getSupplyCurrent().getValue();
      if (current > HOMING_CURRENT) {
        motor.setRotorPosition(0);
        homingState = HomingState.HOMED;
        goalPositionInInches = Positions.STOWED.height;
      }
    } else if (homingState == HomingState.HOMED) {
      double goalPosition = goalPositionInInches * ROTATIONS_PER_ELEVATOR_INCH;
      motor.setControl(
            motionMagic.withPosition(goalPosition));
    } else {
      motor.set(0);
    }
  }

  public Command getHomeCommand() {
    return runOnce(() -> startHoming())
        .andThen(Commands.waitUntil(() -> homingState == HomingState.HOMED));
  }
}
