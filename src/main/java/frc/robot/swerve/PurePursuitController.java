// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Java implementation of
 * https://www.chiefdelphi.com/t/paper-implementation-of-the-adaptive-pure-pursuit-controller/166552
 * using some built-in WPILib classes for paths
 */
public class PurePursuitController {
  private final Trajectory traj;
  private final Supplier<Pose2d> poseSupplier;
  private final double lookAheadDistanceMeters;

  private Trajectory.State lastClosestState;
  private int indexOfLastClosestPoint = 1;
  private Pose2d lastLookAheadPoint;
  private int indexOfLastLookAheadPoint = 0;

  public PurePursuitController(
      Trajectory traj, Supplier<Pose2d> poseSupplier, double lookAheadDistanceMeters) {
    this.traj = traj;
    this.poseSupplier = poseSupplier;

    this.lookAheadDistanceMeters = lookAheadDistanceMeters;

    lastClosestState = traj.getStates().get(0);
    lastLookAheadPoint = traj.getStates().get(1).poseMeters;
  }

  private Trajectory.State findClosestPoint() {
    var robotPose = poseSupplier.get();

    var states = traj.getStates();

    Trajectory.State closestState = states.get(0);

    for (int i = indexOfLastClosestPoint; i < states.size(); i++) {
      var curState = states.get(i);

      var distBtwnCurStateAndRobot =
          curState.poseMeters.minus(robotPose).getTranslation().getNorm();

      var distBtwnClosestStateAndRobot =
          closestState.poseMeters.minus(robotPose).getTranslation().getNorm();

      if (distBtwnCurStateAndRobot < distBtwnClosestStateAndRobot) {
        closestState = curState;
        indexOfLastClosestPoint = i;
      }
    }

    lastClosestState = closestState;

    // System.out.println(closestState);

    return closestState;
  }

  public Optional<Pose2d> findLookAheadPoint() {
    for (int i = indexOfLastLookAheadPoint; i < traj.getStates().size() - 1; i++) {
      var E = traj.getStates().get(i).poseMeters;
      var L = traj.getStates().get(i + 1).poseMeters;
      var C = poseSupplier.get();
      var r = lookAheadDistanceMeters;
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

    if (lastLookAheadPoint != null) {
      return Optional.of(findClosestPoint().poseMeters);
    }

    return Optional.empty();
  }

  private double dotProduct(Transform2d a, Transform2d b) {
    return (a.getX() * b.getX()) + (a.getY() * b.getY());
  }

  public boolean isFinished() {
    return findClosestPoint().equals(traj.getStates().get(traj.getStates().size() - 1));
  }

  public void reset() {
    lastClosestState = traj.getStates().get(0);
    lastLookAheadPoint = traj.getStates().get(1).poseMeters;
  }
}
