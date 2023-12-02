// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OneMechanism;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;

public class LimelightSquare extends CommandBase {
    private static final double CUBE_PICKUP_THRESH = -12;
    private static final double CONE_PICKUP_THRESH = -1;

    private final BooleanSupplier cone;
    private final boolean continuous;
    private boolean hasReachedThresh = false;
    private final DoubleSupplier xSupplier, ySupplier;
    private final SwerveSubsystem drive;
    private PIDController pid;

    /** Creates a new LimelightSquare. */
    public LimelightSquare(BooleanSupplier cone, boolean continuous, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            SwerveSubsystem drive) {
        this.cone = cone;
        this.continuous = continuous;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.drive = drive;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pid = new PIDController(DriverStation.isAutonomousEnabled() ? 7.5 : 5., 0., 0.);
        pid.reset();

        OneMechanism.setLocked(true);

        LimelightHelpers.setPipelineIndex("", (cone.getAsBoolean() ? 1 : 0));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println(cone + ", " + OneMechanism.getGamePieceMode());

        if (!hasReachedThresh)
            hasReachedThresh = LimelightHelpers.getTY("") < (cone.getAsBoolean() ? CONE_PICKUP_THRESH : CUBE_PICKUP_THRESH);

        double rotationOutput = pid.calculate(Units.degreesToRadians(LimelightHelpers.getTX("")));

        drive.drive(new ChassisSpeeds(
            xSupplier.getAsDouble(),
            ySupplier.getAsDouble(),
            (continuous || !hasReachedThresh) ? -rotationOutput : 0.
        ), false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        OneMechanism.setLocked(false);

        drive.drive(new ChassisSpeeds());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
