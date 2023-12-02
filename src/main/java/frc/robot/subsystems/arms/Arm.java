// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arms;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public abstract class Arm extends SubsystemBase {
    protected SparkMaxPIDController pid;

    protected CANSparkMax motor;
    protected RelativeEncoder encoder;
    protected double targetPos, distToTravel;
    protected ElevatorFeedforward FFModel;

    protected void initArm() {
        encoder = motor.getEncoder();
        motor.setSmartCurrentLimit(40);
        motor.setIdleMode(IdleMode.kBrake);
        motor.setOpenLoopRampRate(ArmConstants.OPEN_LOOP_RAMP_RATE);
        motor.setClosedLoopRampRate(ArmConstants.RAMP_RATE);

        motor.burnFlash();
        targetPos = 3.;
    }

    public void runArmVBus(double speed) {
        motor.set(speed);
    }

    public double getMotorCurrent() {
        return motor.getOutputCurrent();
    }

    public void runToPosition(double position) {
        pid.setReference(position, ControlType.kPosition);
    }

    public void runToPosition(double position, double feedForward) {
        pid.setReference(position, ControlType.kPosition, 0, feedForward);
    }

    public boolean atTargetPos() {
        return getError() < 0.2;
    }

    public BooleanSupplier atTargetPosSupplier() {
        return this::atTargetPos;
    }

    public BooleanSupplier isReady() {
        return () -> distToTravel > 1. ? getError() < 0.75 * distToTravel : getError() <= 1.;
    }

    public double getError() {
        return Math.abs(getEncoderPosition() - targetPos);
    }

    public void zeroEncoder() {
        encoder.setPosition(0.);
    }

    public double getEncoderPosition() {
        return encoder.getPosition();
    }

    public void setEncoderPosition(double pos) {
        encoder.setPosition(pos);
    }

    public double getTargetPos() {
        return targetPos;
    }

    public void setTargetPos(double rotations) {
        targetPos = rotations;
    }

    public double getDistanceToTravel() {
        return distToTravel;
    }

    public void setDistanceToTravel(double dist) {
        distToTravel = dist;
    }

    public static Arm getInstance() { return null; }

    public double getZeroVBus() {
        return 0.;
    }

    public double getZeroCurrentThresh() {
        return 0.;
    }

    public Command holdArmPosition() {
        return runOnce(() -> {
            motor.stopMotor();
            runToPosition(encoder.getPosition());
        });
    }

    public Command changePositionCommand(double delta) {
        return runOnce(() -> {
            setTargetPos(targetPos + delta);
            runToPosition(targetPos);
        });
    }

    public ElevatorFeedforward getFF() {
        return FFModel;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
