// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.manipulator;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OneMechanism;
import frc.robot.Constants.GripperConstants;
import frc.robot.OneMechanism.GamePieceMode;

public class Gripper extends SubsystemBase {
    private static Gripper instance;
    private final TalonSRX motor;
    private boolean hasGamepiece = false;

    private enum GripState {
        INFEED, //
        HOLD, //
        BEGIN_OUTFEED, //
        OUTFEED, //
        IDLE, //
    }

    private GripState currentState;

    /** Creates a new Gripper. */
    private Gripper() {
        motor = new TalonSRX(GripperConstants.GRIPPER_CANID);

        motor.configFactoryDefault();

        motor.config_kP(0, 0.2);
        motor.config_kI(0, 0);
        motor.config_kD(0, 0);

        motor.configPeakCurrentLimit(40);
        motor.configPeakCurrentDuration(100);
        motor.configContinuousCurrentLimit(20);
        motor.enableCurrentLimit(true);
        currentState = GripState.IDLE;
    }

    public Command runMotorIn() {
        return runOnce(() -> hasGamepiece = false)
                .andThen(runMotorInWithoutReset());
    }

    public Command runMotorInWithoutReset() {
        return run(() -> {
            motor.set(ControlMode.PercentOutput, GripperConstants.RUN_SPEED);
            currentState = GripState.HOLD;
        });
    }

    public Command stopMotor() {
        return runOnce(() -> motor
                .set(ControlMode.PercentOutput, 0.));
    }

    public Command runMotorOut() {
        return startEnd(() -> {
            currentState = GripState.IDLE;
            motor.set(ControlMode.PercentOutput, -GripperConstants.RUN_SPEED);
        }, () -> motor.set(ControlMode.PercentOutput, 0.));
    }

    public Command runMotorOutSoft() {
        return startEnd(() -> {
            currentState = GripState.IDLE;
            motor.set(ControlMode.PercentOutput, -GripperConstants.SOFT_RUN_SPEED);
        }, () -> motor.set(ControlMode.PercentOutput, 0.));
    }

    public Command modeSensitiveOutfeedCommand() {
        return new ConditionalCommand(
            runMotorOut(),
            runMotorOutSoft(),
            () -> OneMechanism.getGamePieceMode() == GamePieceMode.ORANGE_CONE);
    }

    public void beIdleMode() {
        switch (currentState) {
            case HOLD:
                motor.set(ControlMode.PercentOutput, GripperConstants.HOLD_SPEED);
                break;
            case IDLE:
            default:
                motor.set(ControlMode.PercentOutput, GripperConstants.IDLE_SPEED);
                break;
        }
    }

    public boolean hasGamepiece() {
        return hasGamepiece;
    }

    public BooleanSupplier hasGamepieceSupplier() {
        return () -> hasGamepiece();
    }

    public boolean atCurrentThreshold() {
        if (motor.getSupplyCurrent() > GripperConstants.HOLD_THRESH && motor.getBusVoltage() > 0.) {
            // TODO: Signal aquisition
            return hasGamepiece = true;
        } else
            return false;
    }

    public BooleanSupplier atCurrentThresholdSupplier() {
        return () -> atCurrentThreshold();
    }

    public static Gripper getInstance() {
        return instance == null ? instance = new Gripper() : instance;
    }

    public GripState getGripState() {
        return currentState;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
