// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.math;

import edu.wpi.first.math.geometry.Twist2d;

public class EqualsUtil {
  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, 1e-9);
  }

  /** Extension methods for wpi geometry objects */
  public static class GeomExtensions {
    public static boolean epsilonEquals(Twist2d twist, Twist2d other) {
      return EqualsUtil.epsilonEquals(twist.dx, other.dx)
          && EqualsUtil.epsilonEquals(twist.dy, other.dy)
          && EqualsUtil.epsilonEquals(twist.dtheta, other.dtheta);
    }
  }
}
