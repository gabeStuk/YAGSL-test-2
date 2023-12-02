// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final double LEFT_X_DEADBAND = 0.01;
        public static final double LEFT_Y_DEADBAND = 1e-2;
    }

    public static final class Auton {
        public static final PIDFConfig xAutoPID = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig yAutoPID = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig angleAutoPID = new PIDFConfig(0.7, 0, 0.01);

        public static final double MAX_SPEED = 2;
        public static final double MAX_ACCEL = 2;
    }

    public static final class FieldConstants {
        /** in metres */
        public static final double FIELD_WIDTH = 8.0137;
    }

    public static final class GripperConstants {
        public static final int GRIPPER_CANID = 11;
        public static final double RUN_SPEED = 0.95;
        public static final double SOFT_RUN_SPEED = 0.4;
        public static final double HOLD_SPEED = 0.2;
        public static final double IDLE_SPEED = 0.0;
        public static final double HOLD_THRESH = 45.;
    }

    public static final class WristConstants {
        public static final int WRIST_CANID = 12;

        public static final double kP = 0.06;
        public static final double kI = 0.;
        public static final double kD = 0.4;
        public static final double kIz = 0.;
        public static final double kFF = 0.;

        public static final double kMaxOutput = 0.9;
        public static final double kMinOutput = -0.9;
        public static final double RAMP_RATE = 0.15;
    }

    public static final class ArmConstants {
        public static final double EXTEND_COEFFICIENT = 116.1;
        public static final double RETRACT_COEFFICIENT = 156.58;
        public static final double EXTEND_WAIT_INTERVAL = .2;
        public static final double RETRACT_WAIT_INERVAL = .4;
        public static final double OPEN_LOOP_RAMP_RATE = .1;
        public static final double RAMP_RATE = .25;

        public static final class LowerArmConstants {
            public static final int CAN_ID = 10;

            public static final double kP = .125;
            public static final double kI = 0.;
            public static final double kD = 1.6;
            public static final double kIz = 0.;
            public static final double kFF = 0.;
            public static final double kMaxOutput = .95;
            public static final double kMinOutput = -.5;
            
            public static final double kS = -.23303;
            public static final double kG = .8;
            public static final double kV = .11691;

            public static final double ZEROING_VBUS = -.1;
            public static final double ZEROING_CURRENT_THRESH = 20.;

            public static final double MAX_VEL = 90.;
            public static final double MAX_ACCEL = 180.;
        }

        public static final class UpperArmConstants {
            public static final int CAN_ID = 9;

            public static final double kP = .11;
            public static final double kI = 0.;
            public static final double kD = 0.;
            public static final double kIz = 0.;
            public static final double kFF = 0.;
            public static final double kMaxOutput = .9;
            public static final double kMinOutput = -.9;

            public static final double kS = .089792;
            public static final double kG = 0.;
            public static final double kV = .13158;

            public static final double ZEROING_VBUS = -.15;
            public static final double ZEROING_CURRENT_THRESH = 28.;
            
            public static final double MAX_VEL = 80.;
            public static final double MAX_ACCEL = 160.;
        }
    }
}
