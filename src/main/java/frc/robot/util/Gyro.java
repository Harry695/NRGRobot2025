/*
 * Copyright (c) 2025 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

public interface Gyro {
  /** Returns the gyro angle with positive values in the counter-clockwise direction. */
  Rotation2d getAngle();

  /** Resets the gyro to 0. */
  void reset();
}