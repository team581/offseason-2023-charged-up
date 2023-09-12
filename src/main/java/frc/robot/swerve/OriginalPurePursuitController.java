// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.Trajectory;
import java.util.function.*;

/**
 * Java implementation of
 * https://www.chiefdelphi.com/t/paper-implementation-of-the-adaptive-pure-pursuit-controller/166552
 * using some built-in WPILib classes for paths
 */
public class OriginalPurePursuitController {
  Trajectory traj;
  Supplier<Pose2d> poseSupplier;
  double trackWidthMeters;
  BiConsumer<Double, Double> outputMetersPerSecond;
  double lookAheadDistanceMeters;

  Trajectory.State lastClosestState;
  int indexOfLastClosestPoint = 1;
  Pose2d lastLookAheadPoint;
  int indexOfLastLookAheadPoint = 0;

  public OriginalPurePursuitController(
      Trajectory traj,
      Supplier<Pose2d> poseSupplier,
      double trackWidthMeters,
      BiConsumer<Double, Double> outputMetersPerSecond,
      double lookAheadDistanceMeters) {
    this.traj = traj;
    this.poseSupplier = poseSupplier;
    this.trackWidthMeters = trackWidthMeters;
    this.outputMetersPerSecond = outputMetersPerSecond;
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

  private Pose2d findLookAheadPoint() {
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
          return lookAheadPoint;
        }
      }
    }

    return null;
  }

  private double dotProduct(Transform2d a, Transform2d b) {
    return (a.getX() * b.getX()) + (a.getY() * b.getY());
  }

  double lastCurvature;

  private double calculateCurvature() {
    var robotPose = poseSupplier.get();

    double a = -robotPose.getRotation().getTan();
    double b = 1;
    double c = robotPose.getRotation().getTan() * robotPose.getX() - robotPose.getY();
    var lookAheadPoint = findLookAheadPoint();

    if (lookAheadPoint == null) {
      return lastCurvature;
    }

    double x =
        Math.abs(a * lookAheadPoint.getX() + b * lookAheadPoint.getY() + c)
            / Math.sqrt(a * a + b * b);

    double curvature = (2 * x) / Math.pow(lookAheadDistanceMeters, 2);

    double side =
        Math.signum(
            robotPose.getRotation().getSin() * (lookAheadPoint.getX() - robotPose.getX())
                - robotPose.getRotation().getCos() * (lookAheadPoint.getY() - robotPose.getY()));

    lastCurvature = curvature * side;
    return curvature * side;
  }

  public void setWheelSpeeds() {
    double v = findClosestPoint().velocityMetersPerSecond;

    // System.out.print(findClosestPoint());
    // System.out.print(" ");

    double c = calculateCurvature();

    // System.out.print(c);
    // System.out.print(" ");

    double t = trackWidthMeters;

    double l = v * ((2 + c * t) / 2);
    double r = v * ((2 - c * t) / 2);

    // System.out.print(l);
    // System.out.print(" ");
    // System.out.println(r);

    outputMetersPerSecond.accept(l, r);
  }

  public boolean isFinished() {
    return findClosestPoint().equals(traj.getStates().get(traj.getStates().size() - 1));
  }

  public void reset() {
    lastClosestState = traj.getStates().get(0);
    lastLookAheadPoint = traj.getStates().get(1).poseMeters;
  }
}
