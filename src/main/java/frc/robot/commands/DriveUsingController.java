/*
 * Copyright (c) 2024 Newport Robotics Group. All Rights Reserved.
 *
 * Open Source Software; you can modify and/or share it under the terms of
 * the license file in the root directory of this project.
 */
 
package frc.robot.commands;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.Swerve;

/** A command that enables the driver to drive the robot using an Xbox controller. */
public class DriveUsingController extends Command {
  private static final double RUMBLE_MIN_G = 1.0;
  private static final double RUMBLE_MAX_G = 8.0;

  private static final double DEADBAND = 0.08;

  private final Swerve drivetrain;
  private final CommandXboxController xboxController;

  private static final double LOCK_ORIENTATION_DELAY = 0.5; // Seconds
  private static final double AUTO_ORIENTATION_SPEED_THRESHOLD = 0.05; // Meters per second

  @RobotPreferencesValue(column = 0, row = 1)
  public static final RobotPreferences.DoubleValue AUTO_ORIENT_KP =
      new RobotPreferences.DoubleValue("Drive", "Auto Orient kP", 1.0);

  @RobotPreferencesValue(column = 1, row = 1)
  public static final RobotPreferences.DoubleValue AUTO_ORIENT_KI =
      new RobotPreferences.DoubleValue("Drive", "Auto Orient kI", 0);

  @RobotPreferencesValue(column = 2, row = 1)
  public static final RobotPreferences.DoubleValue AUTO_ORIENT_KD =
      new RobotPreferences.DoubleValue("Drive", "Auto Orient kD", 0);

  private ProfiledPIDController controller;
  private double previousRInput = 0;
  private boolean isRotationLocked = false;
  private Rotation2d lockedRotation = new Rotation2d();
  private final Timer lockOrientationTimer = new Timer();

  /** Creates a new DriveUsingController. */
  public DriveUsingController(Subsystems subsystems, CommandXboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    drivetrain = subsystems.drivetrain;
    this.xboxController = xboxController;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller =
        new ProfiledPIDController(
            AUTO_ORIENT_KP.getValue(),
            AUTO_ORIENT_KI.getValue(),
            AUTO_ORIENT_KD.getValue(),
            Swerve.getRotationalConstraints());
    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setIZone(Math.toRadians(5));
    controller.reset(drivetrain.getOrientation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rInput = -xboxController.getRightX();
    double xInput = -xboxController.getLeftY();
    double yInput = -xboxController.getLeftX();
    double inputScalar = Math.max(1.0 - xboxController.getRightTriggerAxis(), 0.15);

    // Applies deadbands to x and y joystick values and multiples all
    // values with inputScalar which allows finer driving control.
    rInput = MathUtil.applyDeadband(rInput, DEADBAND) * inputScalar;
    xInput = MathUtil.applyDeadband(xInput, DEADBAND) * inputScalar;
    yInput = MathUtil.applyDeadband(yInput, DEADBAND) * inputScalar;

    if (rInput == 0) {
      if (previousRInput != 0) {
        lockOrientationTimer.reset();
        lockOrientationTimer.start();
      } else if (lockOrientationTimer.get() > LOCK_ORIENTATION_DELAY) {
        isRotationLocked = true;
        lockOrientationTimer.stop();
        lockOrientationTimer.reset();
        lockedRotation = drivetrain.getOrientation();
      }
    } else {
      if (isRotationLocked) {
        isRotationLocked = false;
      }
    }

    previousRInput = rInput;
    double rSpeed;

    // If the orientation is locked, maintain the orientation using PID.
    if (isRotationLocked) {
      double feedback =
          controller.calculate(
              drivetrain.getOrientation().getRadians(), lockedRotation.getRadians());

      rSpeed =
          feedback
              + (controller.getSetpoint().velocity / Swerve.getRotationalConstraints().maxVelocity);
    } else {
      rSpeed = rInput;
    }

    drivetrain.drive(xInput, yInput, rSpeed, true);

    if (Swerve.ENABLE_RUMBLE.getValue()) {
      // Rumbles the driver controller based on a exponential scale based on
      // acceleration between
      // min and max.
      double rumblePower =
          MathUtil.inverseInterpolate(RUMBLE_MIN_G, RUMBLE_MAX_G, drivetrain.getAcceleration());
      xboxController.setRumble(RumbleType.kBothRumble, rumblePower * rumblePower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
