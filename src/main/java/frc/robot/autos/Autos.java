// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.NodeHeight;
import frc.robot.States;
import frc.robot.config.Config;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake.HeldGamePiece;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.managers.Autobalance;
import frc.robot.managers.SuperstructureManager;
import frc.robot.managers.vision.AutoScoreManager;
import frc.robot.managers.vision.GroundConeManager;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.vision.VisionMode;
import frc.robot.wrist.WristSubsystem;
import java.lang.ref.WeakReference;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos {
  private static Command wrapAutoEvent(String commandName, Command command) {
    return Commands.sequence(
            Commands.print("[COMMANDS] Starting auto event " + commandName),
            command.deadlineWith(
                Commands.waitSeconds(5)
                    .andThen(
                        Commands.print(
                            "[COMMANDS] Auto event "
                                + commandName
                                + " has been running for 5+ seconds!"))),
            Commands.print("[COMMANDS] Finished auto event " + commandName))
        .handleInterrupt(() -> System.out.println("[COMMANDS] Cancelled auto event " + commandName))
        .withName(commandName);
  }

  private static Map<String, Command> wrapAutoEventMap(Map<String, Command> eventMap) {
    Map<String, Command> wrappedMap = new HashMap<>();
    for (Map.Entry<String, Command> entry : eventMap.entrySet()) {
      wrappedMap.put(
          entry.getKey(), wrapAutoEvent("AutoEvent_" + entry.getKey(), entry.getValue()));
    }
    return wrappedMap;
  }

  private final SuperstructureManager superstructure;
  private final Autobalance autoBalance;

  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private final IntakeSubsystem intake;

  private final SwerveAutoBuilder autoBuilder;
  private final LoggedDashboardChooser<AutoKindWithoutTeam> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");
  private final Map<AutoKind, WeakReference<Command>> autosCache = new EnumMap<>(AutoKind.class);
  private final AutoScoreManager visionManager;
  private final GroundConeManager groundManager;

  public Autos(
      LocalizationSubsystem localization,
      SwerveSubsystem swerve,
      ImuSubsystem imu,
      SuperstructureManager superstructure,
      ElevatorSubsystem elevator,
      WristSubsystem wrist,
      IntakeSubsystem intake,
      Autobalance autoBalance,
      GroundConeManager groundManager,
      AutoScoreManager visionManager) {
    this.localization = localization;
    this.swerve = swerve;
    this.superstructure = superstructure;
    this.intake = intake;
    this.autoBalance = autoBalance;
    this.groundManager = groundManager;
    this.visionManager = visionManager;
    Map<String, Command> eventMap =
        Map.ofEntries(
            Map.entry(
                "coneWait", Commands.waitUntil(() -> intake.getGamePiece() == HeldGamePiece.CONE)),
            Map.entry(
                "cubeWait", Commands.waitUntil(() -> intake.getGamePiece() == HeldGamePiece.CUBE)),
            Map.entry(
                "preloadCube",
                superstructure
                    .setIntakeModeCommand(HeldGamePiece.CUBE)
                    .andThen(Commands.runOnce(() -> intake.setGamePiece(HeldGamePiece.CUBE)))),
            Map.entry("preloadCone", getPreloadConeCommand()),
            Map.entry(
                "scoreLow",
                superstructure
                    .getScoreCommand(NodeHeight.LOW, 0, false)
                    .withTimeout(3)
                    .andThen(Commands.runOnce(() -> intake.setGamePiece(HeldGamePiece.NOTHING)))),
            Map.entry(
                "scoreMid",
                superstructure
                    .getScoreCommand(Config.IS_SPIKE ? NodeHeight.MID : NodeHeight.LOW, 0, false)
                    .withTimeout(3)
                    .andThen(Commands.runOnce(() -> intake.setGamePiece(HeldGamePiece.NOTHING)))),
            Map.entry("scoreHigh", getScoreHighCommand()),
            Map.entry(
                "yeetMid",
                superstructure
                    .getCommand(States.YEET_CUBE_MID)
                    .withTimeout(1.5)
                    .andThen(Commands.runOnce(() -> intake.setGamePiece(HeldGamePiece.NOTHING)))),
            Map.entry("superstructureLow", superstructure.getManualScoreCommand(NodeHeight.LOW)),
            Map.entry(
                "superstructureMid",
                superstructure.getManualScoreCommand(
                    Config.IS_SPIKE ? NodeHeight.MID : NodeHeight.LOW)),
            Map.entry(
                "superstructureHigh",
                superstructure.getManualScoreCommand(
                    Config.IS_SPIKE ? NodeHeight.HIGH : NodeHeight.LOW)),
            Map.entry("home", superstructure.getHomeCommand()),
            Map.entry(
                "intakeCone",
                superstructure
                    .setIntakeModeCommand(HeldGamePiece.CONE)
                    .andThen(superstructure.getFloorIntakeSpinningCommand())),
            Map.entry(
                "intakeCube",
                superstructure
                    .setIntakeModeCommand(HeldGamePiece.CUBE)
                    .andThen(superstructure.getFloorIntakeSpinningCommand())),
            Map.entry("stow", superstructure.getCommand(States.STOWED)),
            Map.entry("stowFast", getStowFastCommand()));

    eventMap = wrapAutoEventMap(eventMap);

    autoBuilder =
        new SwerveAutoBuilder(
            Config.VISION_MODE == VisionMode.FULLY_ENABLED
                ? localization::getPose
                : localization::getOdometryPose,
            localization::resetPose,
            SwerveSubsystem.KINEMATICS,
            Config.SWERVE_TRANSLATION_PID,
            Config.SWERVE_ROTATION_PID,
            (states) -> swerve.setModuleStates(states, false, false),
            eventMap,
            false,
            swerve);

    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> System.out.println("[COMMANDS] Starting command " + command.getName()));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command -> System.out.println("[COMMANDS] Cancelled command " + command.getName()));
    CommandScheduler.getInstance()
        .onCommandFinish(
            command -> System.out.println("[COMMANDS] Finished command " + command.getName()));

    autoChooser.addOption("Do nothing", AutoKindWithoutTeam.DO_NOTHING);
    autoChooser.addOption("Test", AutoKindWithoutTeam.TEST);

    autoChooser.addDefaultOption("Mid 1.5 balance", AutoKindWithoutTeam.MID_1_5_BALANCE);
    autoChooser.addDefaultOption("Mid 2 balance", AutoKindWithoutTeam.MID_2_BALANCE);
    autoChooser.addDefaultOption("Mid 2 balance test", AutoKindWithoutTeam.MID_2_BALANCE_TEST);

    // autoChooser.addOption("Long side 2", AutoKindWithoutTeam.LONG_SIDE_2);
    autoChooser.addOption("Short side 2 balance", AutoKindWithoutTeam.SHORT_SIDE_2_BALANCE);
    autoChooser.addOption("Long side 2 balance", AutoKindWithoutTeam.LONG_SIDE_2_BALANCE);

    autoChooser.addOption("Short side 3", AutoKindWithoutTeam.SHORT_SIDE_3);
    autoChooser.addOption("Long side 3", AutoKindWithoutTeam.LONG_SIDE_3);
    autoChooser.addOption("Long side 3 worn", AutoKindWithoutTeam.LONG_SIDE_3_WARN);

    if (Config.IS_DEVELOPMENT) {
      PathPlannerServer.startServer(5811);
    }

    Logger.getInstance().recordOutput("Autos/CurrentTrajectory", new Trajectory());
    Logger.getInstance().recordOutput("Autos/TargetPose", new Pose2d());
    Logger.getInstance().recordOutput("Autos/SetpointSpeeds/X", 0);
    Logger.getInstance().recordOutput("Autos/SetpointSpeeds/Y", 0);
    Logger.getInstance().recordOutput("Autos/SetpointSpeeds/Omega", 0);
    Logger.getInstance().recordOutput("Autos/TranslationError", new Pose2d());
    Logger.getInstance().recordOutput("Autos/RotationError", 0);

    PPSwerveControllerCommand.setLoggingCallbacks(
        (PathPlannerTrajectory activeTrajectory) -> {
          Logger.getInstance().recordOutput("Autos/CurrentTrajectory", activeTrajectory);
        },
        (Pose2d targetPose) -> {
          Logger.getInstance().recordOutput("Autos/TargetPose", targetPose);
        },
        (ChassisSpeeds setpointSpeeds) -> {
          Logger.getInstance()
              .recordOutput("Autos/SetpointSpeeds/X", setpointSpeeds.vxMetersPerSecond);
          Logger.getInstance()
              .recordOutput("Autos/SetpointSpeeds/Y", setpointSpeeds.vyMetersPerSecond);
          Logger.getInstance()
              .recordOutput("Autos/SetpointSpeeds/Omega", setpointSpeeds.omegaRadiansPerSecond);
        },
        (Translation2d translationError, Rotation2d rotationError) -> {
          Logger.getInstance()
              .recordOutput(
                  "Autos/TranslationError", new Pose2d(translationError, new Rotation2d()));
          Logger.getInstance().recordOutput("Autos/RotationError", rotationError.getDegrees());
        });
  }

  private CommandBase getStowFastCommand() {
    return Commands.runOnce(() -> superstructure.set(States.STOWED));
  }

  private SequentialCommandGroup getScoreHighCommand() {
    return superstructure
        .getScoreCommand(Config.IS_SPIKE ? NodeHeight.HIGH : NodeHeight.LOW, 0, false)
        .withTimeout(3)
        .andThen(Commands.runOnce(() -> intake.setGamePiece(HeldGamePiece.NOTHING)));
  }

  private SequentialCommandGroup getScoreMidCommand() {
    return superstructure
        .getScoreCommand(Config.IS_SPIKE ? NodeHeight.MID : NodeHeight.LOW, 0, false)
        .withTimeout(3)
        .andThen(Commands.runOnce(() -> intake.setGamePiece(HeldGamePiece.NOTHING)));
  }

  private SequentialCommandGroup getScoreLowCommand() {
    return superstructure
        .getScoreCommand(NodeHeight.LOW, 0, false)
        .withTimeout(3)
        .andThen(Commands.runOnce(() -> intake.setGamePiece(HeldGamePiece.NOTHING)));
  }

  private Command getPreloadConeCommand() {
    return superstructure
        .setIntakeModeCommand(HeldGamePiece.CONE)
        .andThen(Commands.runOnce(() -> intake.setGamePiece(HeldGamePiece.CONE)));
  }

  public Command getAutoCommand() {
    AutoKindWithoutTeam rawAuto = autoChooser.get();

    if (rawAuto == null) {
      rawAuto = AutoKindWithoutTeam.MID_1_5_BALANCE;
    }

    AutoKind auto = FmsSubsystem.isRedAlliance() ? rawAuto.redVersion : rawAuto.blueVersion;

    return buildAutoCommand(auto);
  }

  private Command buildAutoCommand(AutoKind auto) {
    WeakReference<Command> ref = autosCache.get(auto);
    if (ref != null && ref.get() != null) {
      Command autoCommand = ref.get();

      if (autoCommand != null) {
        return autoCommand;
      }
    }

    String autoName = "Auto" + auto.toString();
    Command autoCommand = Commands.runOnce(() -> swerve.driveTeleop(0, 0, 0, true, true), swerve);

    if (auto == AutoKind.DO_NOTHING) {
      return autoCommand
          .andThen(localization.getZeroAwayCommand())
          .andThen(superstructure.getHomeCommand())
          .withName(autoName);
    }

    List<PathPlannerTrajectory> pathGroup = Paths.getInstance().getPath(auto);

    autoCommand =
        autoCommand.andThen(
            () -> localization.resetPose(pathGroup.get(0).getInitialHolonomicPose()));

    if (auto == AutoKind.BLUE_SHORT_SIDE_3 || auto == AutoKind.RED_LONG_SIDE_3) {
      autoCommand = autoCommand.andThen(getBlueShortSide3Auto(pathGroup));
    } else if (auto == AutoKind.BLUE_LONG_SIDE_3 || auto == AutoKind.RED_SHORT_SIDE_3) {
      autoCommand = autoCommand.andThen(getBlueLongSide3Auto(pathGroup));
    } else if (auto == AutoKind.BLUE_LONG_SIDE_3_WORN || auto == AutoKind.RED_LONG_SIDE_3_WORN) {
      autoCommand = autoCommand.andThen(getBlueLongSide3Auto(pathGroup));
    } else if (auto == AutoKind.BLUE_SHORT_SIDE_2_BALANCE|| auto == AutoKind.RED_LONG_SIDE_2_BALANCE) {
      autoCommand = autoCommand.andThen(getBlueShortSide2Balance(pathGroup));
    } else if (auto == AutoKind.BLUE_LONG_SIDE_2_BALANCE|| auto == AutoKind.RED_SHORT_SIDE_2_BALANCE) {
      autoCommand = autoCommand.andThen(getBlueLongSide2Balance(pathGroup));
    } else if (auto == AutoKind.BLUE_MID_2_BALANCE || auto == AutoKind.RED_MID_2_BALANCE) {
      autoCommand = autoCommand.andThen(getBlueMid15Balance(pathGroup));
    } else {
      autoCommand = autoCommand.andThen(autoBuilder.fullAuto(pathGroup));
    }

    if (auto.autoBalance) {
      autoCommand = autoCommand.andThen(this.autoBalance.getCommand());
    }

    autoCommand = autoCommand.withName(autoName);

    autosCache.put(auto, new WeakReference<>(autoCommand));

    return autoCommand;
  }

  private Command getBlueShortSide3Auto(List<PathPlannerTrajectory> pathGroup) {
    System.out.println("Markers array:");
    System.out.println(pathGroup.get(0).getMarkers());
    return Commands.sequence(
        superstructure.getHomeCommand(),
        getPreloadConeCommand(),
        getScoreHighCommand(),
        // getScoreMidCommand(),
        getStowFastCommand(),
        followPathWithEvents(pathGroup, 0),
        followPathWithEvents(pathGroup, 1),
        groundManager.getGroundCone(),
        followPathWithEvents(pathGroup, 2),
        visionManager.getAutoScoreMidCone()
        // followPathWithEvents(pathGroup, 3),
        // visionManager.getAutoScoreMidCone()
        );
  }

  private Command followPathWithEvents(List<PathPlannerTrajectory> pathGroup, int index) {
    Command command = Commands.none();

    PathPlannerTrajectory traj = pathGroup.get(index);
    if (index == 0) {
      command =
          command.andThen(
              () -> {
                localization.resetPose(traj.getInitialHolonomicPose());
              });
    }

    command =
        command
            .andThen(autoBuilder.stopEventGroup(traj.getStartStopEvent()))
            .andThen(autoBuilder.followPathWithEvents(traj));

    if (index == pathGroup.size() - 1) {
      command = command.andThen(autoBuilder.stopEventGroup(traj.getEndStopEvent()));
    }

    return command;
  }

  private Command getBlueLongSide3Auto(List<PathPlannerTrajectory> pathGroup) {
    return autoBuilder.fullAuto(pathGroup);

    // return Commands.sequence(
    //     followPathWithEvents(pathGroup, 0),
    //     followPathWithEvents(pathGroup, 1),
    //     // groundManager.getGroundCone(),
    //     followPathWithEvents(pathGroup, 2)
    //     // visionManager.getAutoScoreMidCone()
    //     // followPathWithEvents(pathGroup, 3),
    //     // visionManager.getAutoScoreMidCone()
    // );
  }

  private Command getBlueShortSide2Balance(List<PathPlannerTrajectory> pathGroup) {
    return Commands.sequence(
        followPathWithEvents(pathGroup, 0),
        groundManager.getGroundCone(),
        autoBuilder.followPathWithEvents(pathGroup.get(1)),
        visionManager.getAutoScoreMidCone(),
        autoBuilder.followPathWithEvents(pathGroup.get(2)));
  }

  private Command getBlueLongSide2Balance(List<PathPlannerTrajectory> pathGroup) {
    return Commands.sequence(
        followPathWithEvents(pathGroup, 0),
        groundManager.getGroundCone(),
        autoBuilder.followPathWithEvents(pathGroup.get(1)),
        visionManager.getAutoScoreMidCone());
  }

  private Command getBlueMid15Balance(List<PathPlannerTrajectory> pathGroup) {
    return Commands.sequence(
        followPathWithEvents(pathGroup, 0),
        groundManager.getGroundCone(),
        autoBuilder.followPathWithEvents(pathGroup.get(1)),
        visionManager.getAutoScoreMidCone(),
        autoBuilder.followPathWithEvents(pathGroup.get(2)));
  }

  public void clearCache() {
    autosCache.clear();
    Paths.getInstance().clearCache();
  }
}
