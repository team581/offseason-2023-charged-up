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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
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
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.VisionMode;
import frc.robot.wrist.WristSubsystem;
import java.lang.ref.WeakReference;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Autos extends LifecycleSubsystem {
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
  private final ImuSubsystem imu;

  private AutoKindWithoutTeam rawAuto = AutoKindWithoutTeam.MID_1_BALANCE;

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
    super(SubsystemPriority.AUTO);
    this.localization = localization;
    this.swerve = swerve;
    this.imu = imu;
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
                    .getScoreCommand(NodeHeight.LOW, 0, true)
                    .withTimeout(1)
                    .andThen(
                        Commands.runOnce(
                            () -> {
                              intake.setGamePiece(HeldGamePiece.NOTHING);
                              superstructure.set(States.STOWED);
                            }))),
            Map.entry(
                "scoreMid",
                superstructure
                    .getScoreCommand(NodeHeight.MID, 0, true)
                    .withTimeout(2.5)
                    .andThen(
                        Commands.runOnce(
                            () -> {
                              intake.setGamePiece(HeldGamePiece.NOTHING);
                              superstructure.set(States.STOWED);
                            }))),
            Map.entry(
                "scoreMidCube",
                superstructure
                    .getScoreCommand(NodeHeight.MID, 0, true)
                    .withTimeout(10)
                    .andThen(
                        Commands.runOnce(
                            () -> {
                              intake.setGamePiece(HeldGamePiece.NOTHING);
                              superstructure.set(States.STOWED);
                            }))),
            Map.entry(
                "scoreHigh",
                superstructure
                    .getScoreCommand(NodeHeight.HIGH, 0.5, true)
                    .withTimeout(3)
                    .andThen(
                        Commands.runOnce(
                            () -> {
                              intake.setGamePiece(HeldGamePiece.NOTHING);
                              superstructure.set(States.STOWED);
                            }))),
            Map.entry(
                "yeetCone",
                superstructure
                    .getCommand(States.YEET_CONE)
                    .withTimeout(0.5)
                    .andThen(Commands.runOnce(() -> intake.setGamePiece(HeldGamePiece.NOTHING)))
                    .andThen(Commands.runOnce(() -> superstructure.set(States.STOWED)))),
            Map.entry(
                "yeetConeMid",
                superstructure
                    .getCommand(States.YEET_CONE_MID)
                    .withTimeout(0.5)
                    .andThen(Commands.runOnce(() -> intake.setGamePiece(HeldGamePiece.NOTHING)))
                    .andThen(Commands.runOnce(() -> superstructure.set(States.STOWED)))),
            Map.entry("superstructureLow", superstructure.getManualScoreCommand(NodeHeight.LOW)),
            Map.entry("superstructureMid", superstructure.getManualScoreCommand(NodeHeight.MID)),
            Map.entry("superstructureHigh", superstructure.getManualScoreCommand(NodeHeight.HIGH)),
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

    for (AutoKindWithoutTeam autoKind : EnumSet.allOf(AutoKindWithoutTeam.class)) {
      autoChooser.addOption(autoKind.toString(), autoKind);
    }

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

  private Command getPreloadConeCommand() {
    return superstructure
        .setIntakeModeCommand(HeldGamePiece.CONE)
        .andThen(Commands.runOnce(() -> intake.setGamePiece(HeldGamePiece.CONE)));
  }

  public Command getAutoCommand() {
    AutoKindWithoutTeam rawAutoNullable = autoChooser.get();

    if (rawAutoNullable == null) {
      rawAuto = AutoKindWithoutTeam.MID_1_BALANCE;
    } else {
      rawAuto = rawAutoNullable;
    }

    AutoKind auto = FmsSubsystem.isRedAlliance() ? rawAuto.redVersion : rawAuto.blueVersion;

    return buildAutoCommand(auto);
  }

  @Override
  public void robotPeriodic() {
    Logger.getInstance().recordOutput("Autos/CurrentAuto", rawAuto.toString());
    SmartDashboard.putString("Autos/SelectedAuto", rawAuto.toString());
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

    if (pathGroup != null) {

      if (auto == AutoKind.RED_MID_1_BALANCE || auto == AutoKind.BLUE_MID_1_BALANCE) {

        autoCommand = autoCommand.andThen(getMid1Auto(pathGroup));

      } else {
        autoCommand = autoCommand.andThen(autoBuilder.fullAuto(pathGroup));
      }
    }

    if (auto.autoBalance) {
      autoCommand = autoCommand.andThen(this.autoBalance.getCommand());
    }

    autoCommand = autoCommand.withName(autoName);

    autosCache.put(auto, new WeakReference<>(autoCommand));

    return autoCommand;
  }

  private Command getMid1Auto(List<PathPlannerTrajectory> pathGroup) {
    return Commands.sequence(
        followPathWithEvents(pathGroup, 0),
        // Wait for charge station to level out before going on to balance
        Commands.waitSeconds(1),
        followPathWithEvents(pathGroup, 1));
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

  public void clearCache() {
    autosCache.clear();
    Paths.getInstance().clearCache();
  }
}
