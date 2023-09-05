// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.config.Config;
import frc.robot.util.CircleConverter;
import frc.robot.util.CtreModuleState;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
  private static final CircleConverter DRIVE_MOTOR_WHEEL_CONVERTER =
      CircleConverter.fromDiameter(Config.WHEEL_DIAMETER);

  private final SwerveModuleConstants constants;
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private CANcoder encoder;
  private final DutyCycleOut driveVoltageOpenLoopRequest =
      new DutyCycleOut(0, Config.SWERVE_USE_FOC, true);
  private final PositionVoltage steerMotorControl =
      new PositionVoltage(0, Config.SWERVE_USE_FOC, 0, 0, false);
  private final VelocityVoltage driveVoltageClosedLoopRequest =
      new VelocityVoltage(0, Config.SWERVE_USE_FOC, 0, 0, false);
  private Rotation2d previousAngle = new Rotation2d();

  private SwerveModuleState goalState = new SwerveModuleState();
  private StatusSignal<Double> driveMotorStatorCurrent;

  public SwerveModule(
      SwerveModuleConstants constants, TalonFX driveMotor, TalonFX steerMotor, CANcoder encoder) {
    this.constants = constants;
    this.driveMotor = driveMotor;
    this.steerMotor = steerMotor;
    this.encoder = encoder;

    CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

    cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = constants.angleOffset.getRotations();
    encoder.getConfigurator().apply(cancoderConfig);

    TalonFXConfiguration driveMotorConfigs = new TalonFXConfiguration();

    driveMotorConfigs.Slot0.kP = Config.SWERVE_DRIVE_KP;
    driveMotorConfigs.Slot0.kI = Config.SWERVE_DRIVE_KI;
    driveMotorConfigs.Slot0.kD = Config.SWERVE_DRIVE_KD;
    driveMotorConfigs.Slot0.kV = Config.SWERVE_DRIVE_KV;
    driveMotorConfigs.Slot0.kS = Config.SWERVE_DRIVE_KS;

    driveMotorConfigs.Voltage.PeakForwardVoltage = 12;
    driveMotorConfigs.Voltage.PeakReverseVoltage = -12;

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = 35;
    currentLimits.SupplyCurrentLimitEnable = true;

    if (constants.driveInversion) {
      driveMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      driveMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }

    StatusCode driveStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      driveStatus = driveMotor.getConfigurator().apply(driveMotorConfigs);
      if (driveStatus.isOK()) break;
    }
    if (!driveStatus.isOK()) {
      System.out.println("Could not apply configs, error code: " + driveStatus.toString());
    }

    TalonFXConfiguration steerMotorConfigs = new TalonFXConfiguration();
    steerMotorConfigs.ClosedLoopGeneral.ContinuousWrap = true;

    steerMotorConfigs.Slot0.kV = Config.SWERVE_STEER_KV;
    steerMotorConfigs.Slot0.kP = Config.SWERVE_STEER_KP;
    steerMotorConfigs.Slot0.kI = Config.SWERVE_STEER_KI;
    steerMotorConfigs.Slot0.kD = Config.SWERVE_STEER_KD;
    steerMotorConfigs.Slot0.kS = Config.SWERVE_STEER_KS;

    steerMotorConfigs.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    steerMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    steerMotorConfigs.Feedback.SensorToMechanismRatio = 1.0;
    steerMotorConfigs.Feedback.RotorToSensorRatio = Config.SWERVE_STEER_GEARING_REDUCTION;

    steerMotorConfigs.CurrentLimits.SupplyCurrentLimit = 35;
    steerMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    steerMotorConfigs.MotorOutput.DutyCycleNeutralDeadband = 0;

    if (constants.angleInversion) {
      steerMotorConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    } else {
      steerMotorConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    }

    StatusCode steerStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      steerStatus = steerMotor.getConfigurator().apply(steerMotorConfigs);
      if (steerStatus.isOK()) break;
    }
    if (!steerStatus.isOK()) {
      System.out.println("Could not apply configs, error code: " + steerStatus.toString());
    }

    driveMotorStatorCurrent = driveMotor.getStatorCurrent();
  }

  public void log() {
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + constants.corner.toString() + "/DriveMotorStatorCurrent",
            driveMotorStatorCurrent.refresh().getValue());
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + constants.corner.toString() + "/SteerMotorAngle",
            getSteerMotorPosition().getDegrees());
    Logger.getInstance()
        .recordOutput(
            "Swerve/" + constants.corner.toString() + "/CancoderAngle",
            Units.rotationsToDegrees(encoder.getAbsolutePosition().getValue()));
    Logger.getInstance().recordOutput("Swerve/" + constants.corner.toString() + "/GoalAngle", goalState.angle.getDegrees());
  }

  public void setDesiredState(
      SwerveModuleState state, boolean openLoop, boolean skipJitterOptimization) {
    final var steerMotorPosition = getSteerMotorPosition();
    state = SwerveModuleState.optimize(state, steerMotorPosition);
    goalState = state;

    steerMotor.setControl(steerMotorControl.withPosition(state.angle.getRotations()));

    boolean isStopped = Math.abs(state.speedMetersPerSecond) <= SwerveSubsystem.MAX_VELOCITY * 0.01;
    Rotation2d angle = isStopped && !skipJitterOptimization ? this.previousAngle : state.angle;
    this.previousAngle = angle;

    var wheelRotationsPerSecond =
        DRIVE_MOTOR_WHEEL_CONVERTER.distanceToRotations(state.speedMetersPerSecond);

    if (openLoop) {
      driveMotor.setControl(
          driveVoltageOpenLoopRequest.withOutput(
              state.speedMetersPerSecond / SwerveSubsystem.MAX_VELOCITY));
    } else {
      driveMotor.setControl(driveVoltageClosedLoopRequest.withVelocity(wheelRotationsPerSecond));
    }
  }

  public SwerveModuleState getState() {
    final var steerMotorPosition = getSteerMotorPosition();
    final var driveMotorVelocity = Units.inchesToMeters(getDriveMotorVelocity());

    return new SwerveModuleState(driveMotorVelocity, steerMotorPosition);
  }

  public SwerveModulePosition getPosition() {
    final var steerMotorPosition = getSteerMotorPosition();
    final var driveMotorPosition = getDriveMotorPosition();

    return new SwerveModulePosition(driveMotorPosition, steerMotorPosition);
  }

  private Rotation2d getSteerMotorPosition() {
    double rotations = steerMotor.getPosition().getValue();
    return Rotation2d.fromRotations(rotations);
  }

  private double getDriveMotorPosition() {
    final var rotations = driveMotor.getPosition().getValue();
    final var meters = DRIVE_MOTOR_WHEEL_CONVERTER.rotationsToDistance(rotations);
    return meters;
  }

  private double getDriveMotorVelocity() {
    final var rotationsPerSecond = driveMotor.getVelocity().getValue();
    final var metersPerSecond = DRIVE_MOTOR_WHEEL_CONVERTER.rotationsToDistance(rotationsPerSecond);
    final var inchesPerSecond = metersPerSecond * 39.37;
    return inchesPerSecond;
  }
}
