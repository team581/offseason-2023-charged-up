// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Positions;
import frc.robot.config.Config;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends LifecycleSubsystem {
  private static final double HOMING_CURRENT = 5;
  private final TalonFX motor;
  private double goalPositionInInches = Positions.STOWED.height;
  private double sensorUnitsPerElevatorInch = (Config.ELEVATOR_GEARING * 2048) / (1.75 * Math.PI);
  private HomingState homingState = HomingState.NOT_HOMED;
  private static final double TOLERANCE = 0.5;

  public ElevatorSubsystem(TalonFX motor) {
    super(SubsystemPriority.ELEVATOR);

    this.motor = motor;
    this.motor.setInverted(Config.ELEVATOR_INVERTED);

    // Set pid for slot 0
    Slot0Configs slot0configs = new Slot0Configs();

    slot0configs.kV = Config.ELEVATOR_KV;
    slot0configs.kP = Config.ELEVATOR_KP;
    slot0configs.kI = Config.ELEVATOR_KI;
    slot0configs.kD = Config.ELEVATOR_KD;
    slot0configs.kS = Config.ELEVATOR_KS;

    // Set motion magic
    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = Config.ELEVATOR_CRUISE_VELOCITY;
    motionMagicConfigs.MotionMagicAcceleration = Config.ELEVATOR_ACCELERATION;

    // Set current limiting
    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    TalonFXConfiguration toConfigure = new TalonFXConfiguration();
    //this.motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 80, 100, 1));
    currentLimits.StatorCurrentLimit = 80;
    currentLimits.StatorCurrentLimitEnable = true;


    //this.motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20, 30, 0.5));
    currentLimits.SupplyCurrentLimit = 20;
    currentLimits.SupplyCurrentThreshold = 30;
    currentLimits.SupplyTimeThreshold = 0.5;
    currentLimits.SupplyCurrentLimitEnable = true;

    toConfigure.
  }

  public void startHoming() {
    homingState = HomingState.HOMING;
  }

  public void resetHoming() {
    homingState = HomingState.NOT_HOMED;
    motor.set(TalonFXControlMode.PercentOutput, 0);
  }

  public HomingState getHomingState() {
    return homingState;
  }

  public double getHeight() {
    // Read talon sensor, convert to inches
    double sensorUnits = motor.getRotorPosition().getValue();
    double position = sensorUnits / sensorUnitsPerElevatorInch;
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
    Logger.getInstance().recordOutput("Elevator/StatorCurrent", motor.getStatorCurrent());
    Logger.getInstance().recordOutput("Elevator/SupplyCurrent", motor.getSupplyCurrent());
    Logger.getInstance().recordOutput("Elevator/GoalPosition", goalPositionInInches);
    Logger.getInstance().recordOutput("Elevator/Homing", homingState.toString());
    Logger.getInstance().recordOutput("Elevator/AppliedVoltage", motor.getMotorOutputVoltage());
    Logger.getInstance().recordOutput("Elevator/SensorVelocity", motor.getSelectedSensorVelocity());
    Logger.getInstance().recordOutput("Elevator/TemperatureCelsius", motor.getTemperature());
  }

  @Override
  public void enabledPeriodic() {
    // Add homing sequence
    // Convert goal in inches to sensor units, and set motor
    if (homingState == HomingState.HOMING) {
      motor.set(ControlMode.PercentOutput, -0.15);
      double current = motor.getSupplyCurrent();
      if (current > HOMING_CURRENT) {
        motor.setSelectedSensorPosition(0);
        homingState = HomingState.HOMED;
        goalPositionInInches = Positions.STOWED.height;
      }
    } else if (homingState == HomingState.HOMED) {
      double goalPositionInSensorUnits = goalPositionInInches * sensorUnitsPerElevatorInch;
      motor.set(
          ControlMode.MotionMagic,
          goalPositionInSensorUnits,
          DemandType.ArbitraryFeedForward,
          Config.ELEVATOR_ARB_F);
    } else {
      motor.set(ControlMode.PercentOutput, 0);
    }
  }

  public Command getHomeCommand() {
    return runOnce(() -> startHoming())
        .andThen(Commands.waitUntil(() -> homingState == HomingState.HOMED));
  }
}
