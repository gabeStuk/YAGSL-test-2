// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {

    public static Wrist instance;

    private CANSparkMax motor;
    private SparkMaxAbsoluteEncoder absoluteEncoder;
    private SparkMaxPIDController pid;

    private double targetAngle;

    /** Creates a new Wrist. */
    private Wrist() {
        motor = new CANSparkMax(WristConstants.WRIST_CANID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(25);
        motor.setInverted(true);
        motor.setIdleMode(IdleMode.kCoast);
        motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 18);

        absoluteEncoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(360.);

        absoluteEncoder.setZeroOffset(141.2);
        absoluteEncoder.setInverted(false);

        pid = motor.getPIDController();
        pid.setFeedbackDevice(absoluteEncoder);

        pid.setP(WristConstants.kP);
        pid.setI(WristConstants.kI);
        pid.setD(WristConstants.kD);
        pid.setIZone(WristConstants.kIz);
        pid.setFF(WristConstants.kFF);

        pid.setOutputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput);

        motor.setClosedLoopRampRate(WristConstants.RAMP_RATE);
        motor.burnFlash();
    }

    public double getAbsoluteEncoderPosition() {
        return absoluteEncoder.getPosition();
    }

    public Command runMotor(double vbus) {
        return startEnd(
                () -> motor.set(vbus), () -> {
                    motor.set(vbus);
                    pid.setReference(getAbsoluteEncoderPosition(), ControlType.kPosition);
                });
    }

    public Command stopMotor() {
        return runOnce(() -> motor.set(0.));
    }

    public Command runToAngle(double angle) {
        return run(() -> {
            pid.setReference(angle, ControlType.kPosition);
            targetAngle = angle;
        }).until(() -> Math.abs(getAbsoluteEncoderPosition() - angle) < 0.9);
    }

    public Command holdWristAngle() {
        return runOnce(() -> {
            motor.set(0.);
            targetAngle = getAbsoluteEncoderPosition();
            pid.setReference(targetAngle, ControlType.kPosition);
        });
    }

    public Command changeAngleCommand(double delta) {
        return runOnce(() -> {
            targetAngle += delta;
            pid.setReference(targetAngle, ControlType.kPosition);
        });
    }

    public void buckeyeConstants() {
        pid.setOutputRange(-.85, .85);
    }

    public void worldsConsants() {
        pid.setOutputRange(WristConstants.kMinOutput, WristConstants.kMaxOutput);
    }

    public static Wrist getInstance() {
        return instance == null ? instance = new Wrist() : instance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
