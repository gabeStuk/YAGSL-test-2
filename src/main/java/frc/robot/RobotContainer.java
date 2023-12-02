// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.OneMechanism.GamePieceMode;
import frc.robot.OneMechanism.ScoringPositions;
import frc.robot.RobotContainer.Nodes.TheNodes;
import frc.robot.commands.TeleDrive;
import frc.robot.commands.arm.CurrentZero;
import frc.robot.commands.auton.Autos;
import frc.robot.commands.swerve.AutoAlignToNode;
import frc.robot.commands.vision.LimelightSquare;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.PhotonCam;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.arms.LowerArm;
import frc.robot.subsystems.arms.UpperArm;
import frc.robot.subsystems.kickstand.Kickstand;
import frc.robot.subsystems.manipulator.Gripper;
import frc.robot.subsystems.manipulator.Wrist;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    private final String FRONT_CAM_NAME = "Front_AprilTag_Camera";
    private final String BACK_CAM_NAME = "Rear_AprilTag_Camera";
    private Nodes.TheNodes nodeOn = TheNodes.ZERO;
    private final Pose3d FRONT_CAM_TO_ROBOT = new Pose3d(Units.inchesToMeters(-2.75),
            Units.inchesToMeters(-5.5625), Units.inchesToMeters(43.125),
            new Rotation3d(0., Units.degreesToRadians(-15.),
                    Units.degreesToRadians(90.)));

    private final Pose3d REAR_CAM_TO_ROBOT = new Pose3d(Units.inchesToMeters(-5.875),
            Units.inchesToMeters(5.5625), 0,
            new Rotation3d(0., Units.degreesToRadians(0.),
                    Units.degreesToRadians(270.)));
    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

    private final PhotonCam frontCam = new PhotonCam(FRONT_CAM_NAME, FRONT_CAM_TO_ROBOT.minus(new Pose3d()));
    private final LEDs candle = LEDs.getInstance();

    private final Gripper gripper = Gripper.getInstance();
    private final Wrist wrist = Wrist.getInstance();
    private final UpperArm upperArm = UpperArm.getInstance();
    private final LowerArm lowerArm = LowerArm.getInstance();
    private final Kickstand kickstand = Kickstand.getInstance();

    private final SendableChooser<CommandBase> autonChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        OneMechanism.addSubsystems(candle, swerveSubsystem, lowerArm, upperArm, wrist);

        // Configure the trigger bindings
        configureBindings();
        initAutonChooser();
    }

    private void initAutonChooser() {
        autonChooser.setDefaultOption("e ^ i * pi = -1", Autos.eAuto(swerveSubsystem));
        autonChooser.addOption("e2", Autos.e2Path(swerveSubsystem, frontCam));
        SmartDashboard.putData("Auto Choices", autonChooser);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link
     * CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        swerveSubsystem.setDefaultCommand(new TeleDrive(swerveSubsystem,
                () -> MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> MathUtil.applyDeadband(m_driverController.getRightX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> true, false, false/* DON'T USE TRUE!! */, this::getScalar));

        gripper.setDefaultCommand(new RunCommand(gripper::beIdleMode, gripper));

        m_driverController.start().onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

        m_driverController.a().onTrue(new InstantCommand(() -> {
            var pose = frontCam.getEstimatedPos();
            if (pose.isPresent()) {
                var poseUnwrapped = pose.get();
                swerveSubsystem.addVisionMeasurement(poseUnwrapped.estimatedPose.toPose2d(),
                        poseUnwrapped.timestampSeconds);
            }
        }));

        m_driverController.b().toggleOnTrue(new LimelightSquare(
                () -> OneMechanism.getGamePieceMode().equals(GamePieceMode.ORANGE_CONE),
                true,
                () -> applyScalarAndDeadbandY() * DriveConstants.MAX_SPD_MPS / 2,
                () -> applyScalarAndDeadbandX() * DriveConstants.MAX_SPD_MPS / 2,
                swerveSubsystem));

        m_driverController.back().onTrue(wrist.runToAngle(ScoringPositions.STOWED.wristAngle)
                .andThen(new CurrentZero(.65, upperArm))
                .andThen(new CurrentZero(0., lowerArm))
                .andThen(new WaitCommand(.5))
                .andThen(upperArm.holdArmPosition())
                .andThen(lowerArm.holdArmPosition()));

        m_driverController.leftStick().onTrue(OneMechanism.runArms(ScoringPositions.SCORE_MID_CUBE));
        m_driverController.rightStick().onTrue(OneMechanism.runArms(ScoringPositions.STOWED));
        m_driverController.povDown()
                .onTrue(OneMechanism.runArms(OneMechanism.getGamePieceMode() == GamePieceMode.ORANGE_CONE
                        ? ScoringPositions.ACQUIRE_FLOOR_CONE_TIPPED
                        : ScoringPositions.ACQUIRE_FLOOR_CUBE));
        m_driverController.povUp().onTrue(new InstantCommand(() -> OneMechanism.setScoreMode(!OneMechanism.getScoreMode())));

        m_driverController.leftTrigger().whileTrue(new ConditionalCommand(
                gripper.runMotorIn(),
                gripper.modeSensitiveOutfeedCommand(),
                () -> !OneMechanism.getScoreMode())
                .andThen(new InstantCommand(gripper::beIdleMode)));

        m_driverController.x().whileTrue(swerveSubsystem.xxDrivexx());
        m_driverController.povLeft().onTrue(new InstantCommand(() -> nodeOn = Nodes.decrementNode(nodeOn)));
        m_driverController.povRight().onTrue(new InstantCommand(() -> nodeOn = Nodes.incrementNode(nodeOn)));
        m_driverController.y().onTrue(new AutoAlignToNode(swerveSubsystem, frontCam, nodeOn.name()));

        m_driverController.leftBumper().onTrue(new InstantCommand(OneMechanism::becomePurpleMode));
        m_driverController.rightBumper().onTrue(new InstantCommand(OneMechanism::becomeOrangeMode));
    }

    private double getScalar() {
        return m_driverController.getRightTriggerAxis() * .6 + .4;
    }

    private double applyScalarAndDeadbandY() {
        return MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND) * getScalar();
    }

    private double applyScalarAndDeadbandX() {
        return MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND) * getScalar();
    }

    private double applyScalarAndDeadbandOmega() {
        return MathUtil.applyDeadband(m_driverController.getRightX(), OperatorConstants.LEFT_X_DEADBAND) * getScalar();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autonChooser.getSelected();
    }

    public static class Nodes {
        public enum TheNodes {
            ZERO(6, -1), ONE(6, 0), TWO(6, 1),
            THREE(7, -1), FOUR(7, 0), FIVE(7, 1),
            SIX(8, -1), SEVEN(8, 0), EIGHT(8, 1);

            public int closestTag;
            public int tagOffset;

            TheNodes(int closestTag, int tagOffset) {
                this.closestTag = closestTag;
                this.tagOffset = tagOffset;
            }
        }

        public static TheNodes incrementNode(TheNodes node) {
            switch (node) {
                case ZERO:
                    return TheNodes.ONE;
                case ONE:
                    return TheNodes.TWO;
                case TWO:
                    return TheNodes.THREE;
                case THREE:
                    return TheNodes.FOUR;
                case FOUR:
                    return TheNodes.FIVE;
                case FIVE:
                    return TheNodes.SIX;
                case SIX:
                    return TheNodes.SEVEN;
                case SEVEN:
                case EIGHT:
                default:
                    return TheNodes.EIGHT;
            }
        }

        public static TheNodes decrementNode(TheNodes node) {
            switch (node) {
                case EIGHT:
                    return TheNodes.SEVEN;
                case SEVEN:
                    return TheNodes.SIX;
                case SIX:
                    return TheNodes.FIVE;
                case FIVE:
                    return TheNodes.FOUR;
                case FOUR:
                    return TheNodes.THREE;
                case THREE:
                    return TheNodes.TWO;
                case TWO:
                    return TheNodes.ONE;
                case ONE:
                case ZERO:
                default:
                    return TheNodes.ZERO;
            }
        }
    }

    public TheNodes getCurrentNode() {
        return nodeOn;
    }
}
