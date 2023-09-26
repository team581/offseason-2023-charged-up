// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import com.pathplanner.lib.auto.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.swerve.SwerveCorner;
import frc.robot.swerve.SwerveModuleConstants;
import frc.robot.vision.VisionMode;

public class Config {
  public static final String SERIAL_NUMBER = System.getenv("serialnum");

  // TODO: Change this to false during events
  public static final boolean IS_DEVELOPMENT = true;

  public static final double MATCH_DURATION_TELEOP = 135;

  public static final int DRIVE_CONTROLLER_PORT = 0;
  public static final int OPERATOR_CONTROLLER_PORT = 1;

  public static final String CANIVORE_ID = "581CANivore";

  public static final int PDP_ID = 1;
  public static final ModuleType PDP_TYPE = ModuleType.kRev;

  public static final VisionMode VISION_MODE = VisionMode.ENABLED_UNUSED;

  public static final int PIGEON_ID = 1;

  public static final double WHEEL_DIAMETER = Units.inchesToMeters(3.85);
  public static final double SWERVE_STEER_GEARING_REDUCTION = 150.0 / 7.0;
  public static final double SWERVE_DRIVE_GEARING_REDUCTION =
      50.0 * 16.0 * 45.0 / 14.0 / 28.0 / 15.0;

  public static final Translation2d SWERVE_FRONT_LEFT_LOCATION =
      new Translation2d(0.263525, 0.263525);
  public static final Translation2d SWERVE_FRONT_RIGHT_LOCATION =
      new Translation2d(0.263525, -0.263525);
  public static final Translation2d SWERVE_BACK_LEFT_LOCATION =
      new Translation2d(-0.263525, 0.263525);
  public static final Translation2d SWERVE_BACK_RIGHT_LOCATION =
      new Translation2d(-0.263525, -0.263525);

  public static final int SWERVE_FL_DRIVE_MOTOR_ID = 8;
  public static final int SWERVE_FL_STEER_MOTOR_ID = 9;
  public static final int SWERVE_FL_CANCODER_ID = 13;
  public static final SwerveModuleConstants SWERVE_FL_CONSTANTS =
      new SwerveModuleConstants(
          Rotation2d.fromDegrees(182.021484375), SwerveCorner.FRONT_LEFT, true, true);
  // -62.84
  public static final int SWERVE_FR_DRIVE_MOTOR_ID = 6;
  public static final int SWERVE_FR_STEER_MOTOR_ID = 7;
  public static final int SWERVE_FR_CANCODER_ID = 12;
  public static final SwerveModuleConstants SWERVE_FR_CONSTANTS =
      new SwerveModuleConstants(
          Rotation2d.fromDegrees(159.521484375), SwerveCorner.FRONT_RIGHT, true, true);
  // -147.8
  public static final int SWERVE_BL_DRIVE_MOTOR_ID = 4;
  public static final int SWERVE_BL_STEER_MOTOR_ID = 5;
  public static final int SWERVE_BL_CANCODER_ID = 11;
  public static final SwerveModuleConstants SWERVE_BL_CONSTANTS =
      new SwerveModuleConstants(
          Rotation2d.fromDegrees(2.548828125), SwerveCorner.BACK_LEFT, true, true);
  // 78.75
  public static final int SWERVE_BR_DRIVE_MOTOR_ID = 2;
  public static final int SWERVE_BR_STEER_MOTOR_ID = 3;
  public static final int SWERVE_BR_CANCODER_ID = 10;
  public static final SwerveModuleConstants SWERVE_BR_CONSTANTS =
      new SwerveModuleConstants(
          Rotation2d.fromDegrees(110.390625), SwerveCorner.BACK_RIGHT, true, true);
  // 104.58
  public static final int ELEVATOR_MOTOR_ID = 14;

  public static final double ELEVATOR_GEARING = 7.2;
  public static final double ELEVATOR_MIN_HEIGHT = 0;
  public static final double ELEVATOR_MAX_HEIGHT = 27;
  public static final double ELEVATOR_KF = 0;
  public static final double ELEVATOR_KP = 0.4;
  public static final double ELEVATOR_KI = 0;
  public static final double ELEVATOR_KD = 0.1;
  public static final double ELEVATOR_ARB_F = 0.08;
  public static final int ELEVATOR_CRUISE_VELOCITY = 25000;
  public static final int ELEVATOR_ACCELERATION = 30000;
  public static final boolean ELEVATOR_INVERTED = false;

  public static final int LIGHTS_CANDLE_ID = 15;
  public static final int LIGHTS_LED_COUNT = 0;

  public static final int WRIST_MOTOR_ID = 16;
  public static final double WRIST_GEARING = 25.0 * 2;
  public static final double WRIST_KF = 0;
  public static final double WRIST_KP = 0.15;
  public static final double WRIST_KI = 0;
  public static final double WRIST_KD = 0;
  public static final int WRIST_MOTION_CRUISE_VELOCITY = 7500;
  public static final int WRIST_MOTION_ACCELERATION = 60000;
  public static final double WRIST_HOMED_CURRENT = 1.3;
  public static final Rotation2d WRIST_HOMED_ANGLE = Rotation2d.fromDegrees(-1);
  public static final double WRIST_HOMING_VOLTAGE = -0.1;

  public static final int INTAKE_MOTOR_ID = 17;
  public static final boolean INVERTED_INTAKE = false;

  public static final double SWERVE_STEER_KV = 0.0;
  public static final double SWERVE_STEER_KP = 5.0;
  public static final double SWERVE_STEER_KI = 0.0;
  public static final double SWERVE_STEER_KD = 0.0;
  public static final double SWERVE_STEER_KS = 0.0;

  public static final int SWERVE_DRIVE_VOLTAGE_PEAK_FORWARD_VOLTAGE = 12;
  public static final int SWERVE_DRIVE_VOLTAGE_PEAK_REVERSE_VOLTAGE = -12;
  public static final double SWERVE_DRIVE_CURRENT_LIMIT = 55;
  public static final boolean SWERVE_DRIVE_LIMITS_ENABLE = true;

  public static final double SWERVE_DRIVE_KP = 0.24;
  public static final double SWERVE_DRIVE_KI = 0.0;
  public static final double SWERVE_DRIVE_KD = 0.0;
  public static final double SWERVE_DRIVE_KV = 0.1185;
  public static final double SWERVE_DRIVE_KS = 0.0;

  public static final double STEER_MOTOR_LIMITS = 35;
  public static final boolean SWERVE_MOTOR_LIMITS_ENABLED = true;
  public static final PIDConstants SWERVE_TRANSLATION_PID = new PIDConstants(2.5, 0, 0);
  public static final PIDConstants SWERVE_ROTATION_PID = new PIDConstants(4.5, 0, 0.1);
  public static final PIDConstants SWERVE_ROTATION_SNAP_PID = new PIDConstants(7.5, 0, 0.5);
  public static final boolean SWERVE_USE_FOC = true;

  public static final double SUPERSTRUCTURE_COLLISION_HEIGHT = 0.75;
  public static final Rotation2d SUPERSTRUCTURE_WRIST_RANGE = Rotation2d.fromDegrees(25);

  public static final double ROBOT_CENTER_TO_FRONT = Units.inchesToMeters(20);

  private Config() {}
}
