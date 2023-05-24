package frc.robot.autoController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PurePursuitAutoBuilder {
  private final Supplier<Pose2d> poseSupplier;
  private final Consumer<Pose2d> resetPose;
  private final PIDConstants translationConstants;
  private final PIDConstants rotationConstants;
  private final Consumer<Twist2d> outputChassisState;
  private final Map<String, Command> eventMap;
  private final Subsystem[] driveRequirements;

  public PurePursuitAutoBuilder(
    Supplier<Pose2d> poseSupplier,
    Consumer<Pose2d> resetPose,
    PIDConstants translationConstants,
    PIDConstants rotationConstants,
    Consumer<Twist2d> outputChassisState,
    Map<String, Command> eventMap,
    Subsystem... driveRequirements
  ){
    this.poseSupplier = poseSupplier;
    this.resetPose = resetPose;
    this.translationConstants = translationConstants;
    this.rotationConstants = rotationConstants;
    this.outputChassisState = outputChassisState;
    this.eventMap = eventMap;
    this.driveRequirements = driveRequirements;
  }


}
