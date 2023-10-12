// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import frc.robot.Constants.Auton;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class Autos {

    public static CommandBase eAuto(SwerveSubsystem swerve) {
        PathPlannerTrajectory e = PathPlanner.loadPath("e", new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCEL));

        return Commands.sequence(new FollowTrajectory(swerve, e, true));
    }

    public static CommandBase notGayAuto(SwerveSubsystem swerve) {
        PathPlannerTrajectory notGay = PathPlanner.loadPath("straight just like me",
                new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCEL));

        return Commands.sequence(new FollowTrajectory(swerve, notGay, true));
    }

    public static CommandBase notGayEventMapAuto(SwerveSubsystem swerve) {
        PathPlannerTrajectory notGayMap = PathPlanner.loadPath("straight map", new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCEL));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("print", new InstantCommand(() -> swerve.thingY = true));
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            swerve::getRobotPose,
            swerve::resetOdometry,
            new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d),
            new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d),
            swerve::setChassisSpeeds,
            eventMap,
            swerve);
        return Commands.sequence(autoBuilder.fullAuto(notGayMap));
    }

    public static CommandBase e2Path(SwerveSubsystem swerve) {
        PathPlannerTrajectory e2Traj = PathPlanner.loadPath("e2", new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCEL));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("e2Event", new InstantCommand(System.out::println));
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            swerve::getRobotPose,
            swerve::resetOdometry,
            new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d),
            new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d),
            swerve::setChassisSpeeds,
            eventMap,
            swerve);
        return Commands.sequence(autoBuilder.fullAuto(e2Traj));
    }
    
    public static CommandBase nicksArc(SwerveSubsystem swerve) {
        PathPlannerTrajectory arcTraj = PathPlanner.loadPath("Nick's Crazy Arc", new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCEL));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("Halfish Way Through Path 1", new InstantCommand(() -> swerve.halfPath1 = true));
        eventMap.put("Halfish Way Through Path 2", new InstantCommand(() -> swerve.halfPath2 = true));
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            swerve::getRobotPose,
            swerve::resetOdometry,
            new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d), 
            new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d), 
            swerve::setChassisSpeeds, 
            eventMap, 
            swerve);
        return Commands.sequence(autoBuilder.fullAuto(arcTraj));

    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
