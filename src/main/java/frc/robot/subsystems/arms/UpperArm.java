// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants.UpperArmConstants;

public class UpperArm extends Arm {
    private static UpperArm instance;

    /** Creates a new UpperArm. */
    private UpperArm() {
        FFModel = new ElevatorFeedforward(UpperArmConstants.kS, UpperArmConstants.kG, UpperArmConstants.kV);

        motor = new CANSparkMax(UpperArmConstants.CAN_ID, MotorType.kBrushless);
        motor.setInverted(true);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 17);

        pid = motor.getPIDController();
        pid.setP(UpperArmConstants.kP);
        pid.setI(UpperArmConstants.kI);
        pid.setD(UpperArmConstants.kD);
        pid.setIZone(UpperArmConstants.kIz);
        pid.setFF(UpperArmConstants.kFF);
        pid.setOutputRange(UpperArmConstants.kMinOutput, UpperArmConstants.kMaxOutput);

        super.initArm();
    }

    public static UpperArm getInstance() {
        return instance == null ? instance = new UpperArm() : instance;
    }

    @Override
    public double getZeroCurrentThresh() {
        return UpperArmConstants.ZEROING_CURRENT_THRESH;
    }

    @Override
    public double getZeroVBus() {
        return UpperArmConstants.ZEROING_VBUS;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Up Arm Pos", getEncoderPosition());
        SmartDashboard.putNumber("Up Arm Target", targetPos);
        SmartDashboard.putNumber("Up Arm Err", getError());
        SmartDashboard.putNumber("Up Arm Amps", getMotorCurrent());
    }
}
