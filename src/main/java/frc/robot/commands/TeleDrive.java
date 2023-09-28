// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveController;

public class TeleDrive extends CommandBase {

    private final SwerveSubsystem swerve;
    private final Supplier<Double> vX;
    private final Supplier<Double> vY;
    private final Supplier<Double> omega;
    private final Supplier<Boolean> driveMode;
    private final boolean isOpenLoop;
    private final SwerveController controller;
    private final Timer timer = new Timer();
    private final boolean headingCorrection;
    private double angle = 0;
    private double lastTime = 0;
    private final Supplier<Double> scalar;

    /** Creates a new TeleDrive. */
    public TeleDrive(SwerveSubsystem swerve, Supplier<Double> vX, Supplier<Double> vY, Supplier<Double> omega,
            Supplier<Boolean> driveMode, boolean isOpenLoop, boolean headingCorrection, Supplier<Double> scalar) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.omega = omega;
        this.driveMode = driveMode;
        this.isOpenLoop = isOpenLoop;
        this.controller = swerve.getController();
        this.headingCorrection = headingCorrection;
        this.scalar = scalar;
        if (headingCorrection) {
            timer.start();
        }

        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        lastTime = headingCorrection ? timer.get() : lastTime;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // double xVel = Math.pow(vX.get(), 3);
        // double yVel = Math.pow(vY.get(), 3);
        // double angVel = Math.pow(omega.get(), 3);
        double xVel = vX.get();
        double yVel = vY.get();
        double angVel = omega.get();

        if (headingCorrection) {
            angle += (angVel * (timer.get() - lastTime)) * controller.config.maxAngularVelocity;
            // angle += angVel * controller.config.maxAngularVelocity;

            ChassisSpeeds correctedChassisSpeeds = controller.getTargetSpeeds(xVel, yVel, angle,
                    swerve.getHeading().getRadians());
            swerve.drive(SwerveController.getTranslation2d(correctedChassisSpeeds).times(scalar.get()),
                    correctedChassisSpeeds.omegaRadiansPerSecond * scalar.get(),
                    driveMode.get(), isOpenLoop);
            lastTime = timer.get();
        } else {
            swerve.drive(new Translation2d(xVel * controller.config.maxSpeed, yVel * controller.config.maxSpeed).times(scalar.get()),
                    angVel * scalar.get(), headingCorrection, isOpenLoop);
        }
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
}
