// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import frc.robot.OneMechanism;
import frc.robot.Constants.Auton;
import frc.robot.OneMechanism.ScoringPositions;
import frc.robot.commands.vision.LimelightSquare;
import frc.robot.subsystems.PhotonCam;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.manipulator.Gripper;

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
        PathPlannerTrajectory notGayMap = PathPlanner.loadPath("straight map",
                new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCEL));
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

    public static CommandBase e2Path(SwerveSubsystem swerve, PhotonCam limelight) {
        PathPlannerTrajectory e2Traj = PathPlanner.loadPath("e2",
                new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCEL));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("e2Event", new InstantCommand(() -> {
            var pose = limelight.getEstimatedPos();
            if (pose.isPresent()) {
                var poseUnwrapped = pose.get();
                swerve.addVisionMeasurement(poseUnwrapped.estimatedPose.toPose2d(), poseUnwrapped.timestampSeconds, false, .5);
            }

        }));
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

    public static CommandBase ifOfFedPath(SwerveSubsystem swerveSubsystem, Gripper gripper) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("Trying a in of feed",
                new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCEL));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("infed", OneMechanism.runArms(ScoringPositions.ACQUIRE_FLOOR_CUBE)
                .andThen(gripper.runMotorIn().withTimeout(0.5)));
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            swerveSubsystem::getRobotPose, 
            swerveSubsystem::resetOdometry, 
            new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d), 
            new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d), 
            swerveSubsystem::setChassisSpeeds, 
            eventMap, 
            swerveSubsystem, gripper);
        return Commands.sequence(autoBuilder.fullAuto(traj));
    }

    public static CommandBase trackPath(SwerveSubsystem swerveSubsystem, Gripper gripper) {
        PathPlannerTrajectory traj = PathPlanner.loadPath("GamePieceTrack",
                new PathConstraints(Auton.MAX_SPEED, Auton.MAX_ACCEL));
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("StartTrack", new LimelightSquare(
            () -> false,
            true,
        () -> (Auton.MAX_SPEED / 4.),
        () -> 0., swerveSubsystem).withTimeout(2));
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            swerveSubsystem::getRobotPose,
            swerveSubsystem::resetOdometry,
            new PIDConstants(Auton.yAutoPID.p, Auton.yAutoPID.i, Auton.yAutoPID.d),
            new PIDConstants(Auton.angleAutoPID.p, Auton.angleAutoPID.i, Auton.angleAutoPID.d),
            swerveSubsystem::setChassisSpeeds,
            eventMap,
            swerveSubsystem,
            gripper);
        return Commands.sequence(autoBuilder.fullAuto(traj));
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
