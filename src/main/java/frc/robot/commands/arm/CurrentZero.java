// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arms.Arm;

/** An example command that uses an example subsystem. */
public class CurrentZero extends CommandBase {
    private Arm m_arm;

    private final double m_zeroOffset;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem
     *            The subsystem used by this command.
     * @return
     */
    public CurrentZero(double zeroOffset, Arm arm) {
        m_zeroOffset = zeroOffset;
        m_arm = arm;
        addRequirements(arm);
        // Use addRequirements() here to declare subsystem dependencies.

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_arm.runArmVBus(m_arm.getZeroVBus());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_arm.runArmVBus(m_arm.getZeroVBus());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_arm.setEncoderPosition(m_zeroOffset);
        m_arm.runArmVBus(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_arm.getMotorCurrent() >= m_arm.getZeroCurrentThresh();
    }
}
