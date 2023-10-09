package frc.robot.commands.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Auton;
import frc.robot.subsystems.SwerveSubsystem;

public class FollowTrajectory extends SequentialCommandGroup{
    public FollowTrajectory(SwerveSubsystem drive, PathPlannerTrajectory traj, boolean resetOdometry) {
        addRequirements(drive);

        if(resetOdometry) drive.resetOdometry(traj.getInitialHolonomicPose());

        addCommands(
            new PPSwerveControllerCommand(traj,
            drive::getRobotPose,
            Auton.xAutoPID.createPIDController(),
            Auton.yAutoPID.createPIDController(),
            Auton.angleAutoPID.createPIDController(),
            drive::setChassisSpeeds, drive),
            drive.xxDrivexx()
        );
    }
}
