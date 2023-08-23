// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.wrist;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Positions;
import frc.robot.config.Config;
import frc.robot.util.HomingState;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends LifecycleSubsystem {
  private static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2);
  private final TalonFX motor;
  private Rotation2d goalAngle = Positions.STOWED.angle;
  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);

  private HomingState homingState = HomingState.NOT_HOMED;

  public WristSubsystem(TalonFX motor) {
    super(SubsystemPriority.WRIST);

    this.motor = motor;

    motor.setInverted(false);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();


    //slot0Configs
    motorConfig.Slot0.kV = Config.WRIST_KV;
    motorConfig.Slot0.kP = Config.WRIST_KP;
    motorConfig.Slot0.kI = Config.WRIST_KI;
    motorConfig.Slot0.kD = Config.WRIST_KD;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = Config.WRIST_MOTION_CRUISE_VELOCITY;
    motorConfig.MotionMagic.MotionMagicAcceleration = Config.WRIST_MOTION_ACCELERATION;
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorConfig.CurrentLimits.SupplyCurrentLimit = 25;
    motorConfig.CurrentLimits.SupplyCurrentThreshold = 25;
    motorConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConfig.Feedback.SensorToMechanismRatio = Config.WRIST_GEARING;

    motor.getConfigurator().apply(motorConfig);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(
        motor.getPosition().getValue());
  }

  public void setAngle(Rotation2d angle) {
    goalAngle = angle;
  }

  public boolean atAngle(Rotation2d angle) {
    var currentAngle = getAngle();
    return Math.abs(currentAngle.minus(angle).getDegrees()) < TOLERANCE.getDegrees();
  }

  public void startHoming() {
    homingState = HomingState.HOMING;
  }

  public void resetHoming() {
    homingState = HomingState.NOT_HOMED;
    motor.set(0);
  }

  public HomingState getHomingState() {
    return homingState;
  }

  @Override
  public void autonomousInit() {
    resetHoming();
  }

  @Override
  public void enabledPeriodic() {

    double rawCurrent = motor.getSupplyCurrent().getValue();
    double filteredCurrent = currentFilter.calculate(rawCurrent);

    Logger.getInstance().recordOutput("Wrist/FilteredCurrent", filteredCurrent);
    Logger.getInstance().recordOutput("Wrist/RawCurrent", rawCurrent);

    if (homingState == HomingState.HOMING) {
      motor.set(Config.WRIST_HOMING_VOLTAGE);

      if (filteredCurrent > Config.WRIST_HOMED_CURRENT) {
        motor.set(0);
        motor.setRotorPosition(Config.WRIST_HOMED_ANGLE.getRotations() * Config.WRIST_GEARING);
        setAngle(Positions.STOWED.angle);
        homingState = HomingState.HOMED;
      }
    } else if (homingState == HomingState.HOMED) {
      motor.set(goalAngle.getRotations() * 2048 * Config.WRIST_GEARING);
    } else {
      motor.set(0);
    }
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Wrist/Angle", getAngle().getDegrees());
    Logger.getInstance().recordOutput("Wrist/GoalAngle", goalAngle.getDegrees());
    Logger.getInstance().recordOutput("Wrist/Homing", homingState.toString());

    Logger.getInstance().recordOutput("Wrist/Voltage", motor.getDutyCycle().getValue());

    if (Config.IS_DEVELOPMENT) {
      Logger.getInstance().recordOutput("Wrist/RawAngle", motor.getPosition().getValue());
      Logger.getInstance().recordOutput("Wrist/ControlMode", motor.getControlMode().toString());
    }
  }

  public Command getHomeCommand() {
    return runOnce(() -> startHoming())
        .andThen(Commands.waitUntil(() -> getHomingState() == HomingState.HOMED));
  }
}
