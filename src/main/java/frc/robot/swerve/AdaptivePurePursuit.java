// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;

public class AdaptivePurePursuit {
  private static Trajectory generateTrajectory(List<Pose2d> path, TrajectoryConfig config) {
    Pose2d start = path.get(0);
    List<Translation2d> interiorPoints =
        path.subList(1, path.size() - 1).stream()
            .map(Pose2d::getTranslation)
            .collect(Collectors.toList());
    Pose2d end = path.get(path.size() - 1);

    return TrajectoryGenerator.generateTrajectory(start, interiorPoints, end, config);
  }

  private final Trajectory trajectory;
  private final Supplier<Pose2d> poseSupplier;
  private final UnaryOperator<Double> calculateLookaheadDistance;

  private int indexOfLastClosestPoint = 0;
  private int indexOfLastLookAheadPoint = 0;
  private Trajectory.State lastClosestState;
  private Pose2d lastLookAheadPoint;

  public AdaptivePurePursuit(
      List<Pose2d> path,
      TrajectoryConfig config,
      Supplier<Pose2d> poseSupplier,
      UnaryOperator<Double> calculateLookaheadDistance) {
    this(generateTrajectory(path, config), poseSupplier, calculateLookaheadDistance);
  }

  public AdaptivePurePursuit(
      Trajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      UnaryOperator<Double> calculateLookaheadDistance) {
    this.trajectory = trajectory;
    this.poseSupplier = poseSupplier;
    this.calculateLookaheadDistance = calculateLookaheadDistance;

    lastClosestState = trajectory.getStates().get(0);
    lastLookAheadPoint = trajectory.getStates().get(1).poseMeters;
  }

  private Trajectory.State findClosestPoint() {
    Pose2d pose = poseSupplier.get();
    List<Trajectory.State> states = trajectory.getStates();
    // TODO: This variable seems like it can be removed, and we can just access lastClosestState
    // directly (as long as we start by looking at state 0)
    Trajectory.State closestState = states.get(0);

    for (int i = indexOfLastClosestPoint; i < states.size(); i++) {
      Trajectory.State state = states.get(i);

      double currentStateDistance = state.poseMeters.minus(pose).getTranslation().getNorm();
      double closestStateDistance = closestState.poseMeters.minus(pose).getTranslation().getNorm();

      if (closestStateDistance > currentStateDistance) {
        closestState = state;
        indexOfLastClosestPoint = i;
      }
    }

    lastClosestState = closestState;

    Logger.getInstance().recordOutput("PurePursuit/ClosestPoint", closestState.poseMeters);
    return closestState;
  }

  public Optional<Pose2d> findLookAheadPoint() {
    for (int i = indexOfLastLookAheadPoint; i < trajectory.getStates().size() - 1; i++) {
      var E = trajectory.getStates().get(i).poseMeters;
      var L = trajectory.getStates().get(i + 1).poseMeters;
      var C = poseSupplier.get();
      double errorDistance = C.getTranslation().getDistance(E.getTranslation());
      var r = calculateLookaheadDistance.apply(errorDistance);
      var d = L.minus(E);
      var f = E.minus(C);

      var a = dotProduct(d, d);
      var b = 2 * dotProduct(f, d);
      var c = dotProduct(f, f) - r * r;
      var discriminant = b * b - 4 * a * c;

      double t1 = 0, t2 = 0, tVal = 0;

      boolean intersects = true;

      if (discriminant < 0) {
        // no intersection
        intersects = false;
      } else {
        discriminant = Math.sqrt(discriminant);
        t1 = (-b - discriminant) / (2 * a);
        t2 = (-b + discriminant) / (2 * a);
        if (t1 >= 0 && t1 <= 1) {
          // return t1 intersection
          tVal = t1;
        }
        if (t2 >= 0 && t2 <= 1) {
          // return t2 intersection
          tVal = t2;
        }
        // otherwise, no intersection
        intersects = false;
      }

      // System.out.println(intersects);

      if (intersects) {
        var intersectionPoint = E.plus(d.times(tVal));

        if (tVal + i
            > indexOfLastLookAheadPoint) //  if the fractional index is greater than the index of
        // the last lookahead point
        {
          var lookAheadPoint = intersectionPoint;

          lastLookAheadPoint = lookAheadPoint;
          indexOfLastLookAheadPoint = i;
          return Optional.of(lookAheadPoint);
        }
      }
    }

    return Optional.empty();
  }

  private double dotProduct(Transform2d a, Transform2d b) {
    return (a.getX() * b.getX()) + (a.getY() * b.getY());
  }
}
