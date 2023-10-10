// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import frc.robot.Constants.Auton;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {
    /** Example static factory for an autonomous command. */
    public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
        return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
    }

    public static CommandBase eAuto(SwerveSubsystem swerve) {
        PathPlannerTrajectory e = PathPlanner.loadPath("e", new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCEL));

        return Commands.sequence(new FollowTrajectory(swerve, e, true));
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
