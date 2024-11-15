// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticSubsystem;
import frc.robot.subsystems.Shootersubsystem;

/** Container for auto command factories. */
public final class Grab {
  
  public static Command grab(DriveSubsystem drive, IntakeSubsystem intake, PneumaticSubsystem pneumatics, Shootersubsystem shooter) {

    // Grab and lift the Note
    return  
        new InstantCommand (
            ()-> intake.setIntake(IntakeConstants.kIntakeSpeed), intake)
            .andThen(new InstantCommand (
                () -> intake.setGripper(IntakeConstants.kFingersInAngle), intake))
            .andThen(new InstantCommand (
                () -> pneumatics.setArmDown(), pneumatics))
            .andThen(
                new WaitCommand(1))
            .andThen(new InstantCommand (
                () -> intake.setGripper(IntakeConstants.kFingersOutAngle), intake))
            .andThen(
                new WaitCommand(3))
            .andThen(new InstantCommand (
                () -> pneumatics.setArmUp(), pneumatics));


    }

   public void robotInit() {

  }

  private Grab() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
