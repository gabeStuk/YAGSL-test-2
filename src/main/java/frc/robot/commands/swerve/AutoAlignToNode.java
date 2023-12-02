// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swerve;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.auton.FollowTrajectory;
import frc.robot.subsystems.PhotonCam;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAlignToNode extends CommandBase {

    SwerveSubsystem drivetrain;
    PhotonCam camera;
    Alliance isBlue = DriverStation.getAlliance();
    RobotContainer.Nodes.TheNodes node;

    /** Creates a new AutoAlignToNode. */
    public AutoAlignToNode(SwerveSubsystem drivetrain, PhotonCam camera, String nodeName) {

        for (RobotContainer.Nodes.TheNodes node : RobotContainer.Nodes.TheNodes.values())
            this.node = node.name().equalsIgnoreCase(nodeName) ? node : this.node;

        this.drivetrain = drivetrain;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(this.camera = camera);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        var pose = camera.getEstimatedPos();
        if (pose.isPresent()) {
            var poseUnwrapped = pose.get();
            drivetrain.addVisionMeasurement(poseUnwrapped.estimatedPose.toPose2d(), poseUnwrapped.timestampSeconds, false, .5);
        }

        Pose2d nodePose;
        switch (node) {
            case ZERO:
                nodePose = new Pose2d(2.05, 4.94, new Rotation2d(Math.PI));
                break;
            case ONE:
                nodePose = new Pose2d(2.05, 4.45, new Rotation2d(Math.PI));
                break;
            case TWO:
                nodePose = new Pose2d(2.05, 3.86, new Rotation2d(Math.PI));
                break;
            case THREE:
                nodePose = new Pose2d(2.05, 3.30, new Rotation2d(Math.PI));
                break;
            case FOUR:
                nodePose = new Pose2d(2.05, 2.75, new Rotation2d(Math.PI));
                break;
            case FIVE:
                nodePose = new Pose2d(2.05, 2.21, new Rotation2d(Math.PI));
                break;
            case SIX:
                nodePose = new Pose2d(2.05, 1.63, new Rotation2d(Math.PI));
                break;
            case SEVEN:
                nodePose = new Pose2d(2.05, 1.08, new Rotation2d(Math.PI));
                break;
            case EIGHT:
                nodePose = new Pose2d(2.05, 0.42, new Rotation2d(Math.PI));
                break;
            default:
                nodePose = null;
                throw new RuntimeException("Node inputted into AutoAlignToNode command was invalid.");
        }

        if (isBlue == Alliance.Red) {
            Translation2d transformedTranslation = new Translation2d(
                    nodePose.getX(),
                    FieldConstants.FIELD_WIDTH - nodePose.getY());

            Rotation2d transformedHeading = nodePose.getRotation().times(-1);

            nodePose = new Pose2d(transformedTranslation, transformedHeading);
        }

        new FollowTrajectory(
                drivetrain,
                PathPlannerTrajectory.transformTrajectoryForAlliance(generateTrajToPose(nodePose),
                        isBlue),
                true).schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public PathPlannerTrajectory generateTrajToPose(Pose2d desiredPose) {
        Pose2d roboPose = drivetrain.getRobotPose();

        if (roboPose.equals(desiredPose))
            return new PathPlannerTrajectory();

        double driveMaxVel = drivetrain.getConfig().maxSpeed * 0.25;

        PathConstraints constraints = new PathConstraints(
                driveMaxVel,
                driveMaxVel);

        List<PathPoint> points = new ArrayList<>();

        Translation2d midTranslation = desiredPose.getTranslation().plus(roboPose.getTranslation()).div(2)
                .plus(new Translation2d(0.15, 0.));

        Rotation2d midHeading = new Rotation2d(Math.atan2(
                desiredPose.getTranslation().getY() - midTranslation.getY(),
                desiredPose.getTranslation().getX() - midTranslation.getX()));

        PathPoint startPoint = new PathPoint(
                roboPose.getTranslation(),
                new Rotation2d(Math.atan2(
                        midTranslation.getY() - roboPose.getTranslation().getY(),
                        midTranslation.getX() - roboPose.getTranslation().getX()),
                        roboPose.getRotation().getRadians()));

        PathPoint midPoint = new PathPoint(
                midTranslation,
                midHeading,
                desiredPose.getRotation(),
                driveMaxVel);

        PathPoint endPoint = new PathPoint(
                desiredPose.getTranslation(),
                new Rotation2d(Math.PI).minus(midHeading),
                desiredPose.getRotation());

        points.add(startPoint);
        points.add(midPoint);
        points.add(endPoint);

        points.add(new PathPoint(desiredPose.getTranslation(), desiredPose.getRotation(), desiredPose.getRotation()));

        return PathPlanner.generatePath(constraints, points);
    }
}
