// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
    private SwerveDrive swerveDrive;
    private static SwerveSubsystem instance;
    /** Creates a new SwerveSubsystem. */
    private SwerveSubsystem(File directory) {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive();
        } catch (IOException e) {
            System.exit("https://www.youtube.com/watch?v=dQw4w9WgXcQ".hashCode());
        }

        setBrake(true);
    }

    public void addVisionMesurment(Pose2d pose, double timestamp, boolean soft, Matrix<N3, N1> visionSTDevs) {
        swerveDrive.addVisionMeasurement(pose, timestamp, soft, visionSTDevs);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    public Rotation2d getHeading() {
        return swerveDrive.getYaw();
    }

    public SwerveController getController() {
        return swerveDrive.swerveController;
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY,
                getHeading().getRadians());
    }

    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(), getHeading().getRadians());
    }

    public Rotation3d getGyroRotation() {
        return swerveDrive.getGyroRotation3d();
    }

    public ChassisSpeeds getFORobotVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    public Pose2d getRobotPose() {
        return swerveDrive.getPose();
    }

    /**
     * Locks the wheels by pointing them all toward the center of the Robot
     */
    public Command xxDrivexx() {
        return run(swerveDrive::lockPose);
    }

    public void setBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
        // This method will be called once per scheduler run
    }

    public static SwerveSubsystem getInstance(File directory) {
        return instance == null ? instance = new SwerveSubsystem(directory) : instance;
    }
}