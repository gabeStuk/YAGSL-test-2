// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants.LowerArmConstants;

public class LowerArm extends Arm {
    private static LowerArm instance;
    /** Creates a new LowerArm. */
    private LowerArm() {
        FFModel = new ElevatorFeedforward(LowerArmConstants.kS, LowerArmConstants.kG, LowerArmConstants.kV);
        
        motor = new CANSparkMax(LowerArmConstants.CAN_ID, MotorType.kBrushless);
        motor.setInverted(true);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 19);

        pid = motor.getPIDController();
        pid.setP(LowerArmConstants.kP);
        pid.setI(LowerArmConstants.kI);
        pid.setD(LowerArmConstants.kD);
        pid.setIZone(LowerArmConstants.kIz);
        pid.setFF(LowerArmConstants.kFF);
        pid.setOutputRange(LowerArmConstants.kMinOutput, LowerArmConstants.kMaxOutput);

        super.initArm();
    }

    @Override
    public double getZeroCurrentThresh() {
        return LowerArmConstants.ZEROING_CURRENT_THRESH;
    }

    @Override
    public double getZeroVBus() {
        return LowerArmConstants.ZEROING_VBUS;
    }

    public static LowerArm getInstance() {
        return instance == null ? instance = new LowerArm() : instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Low Arm Pos", getEncoderPosition());
        SmartDashboard.putNumber("Low Arm Targ", targetPos);
        SmartDashboard.putNumber("Low Arm Err", getError());
        SmartDashboard.putNumber("Low Arm Amps", getMotorCurrent());
    }
}
