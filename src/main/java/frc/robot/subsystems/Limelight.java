// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    private PhotonCamera limelight;
    private AprilTagFieldLayout fieldLayout;
    private PhotonPoseEstimator poseEstimator;

    /** Creates a new Limelight. */
    public Limelight(String camName, Transform3d robotToCam) {
        limelight = new PhotonCamera(camName);
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP, limelight, robotToCam);
        } catch (IOException e) {
            throw new RuntimeException("There was a camera error.");
        }
    }

    public PhotonPipelineResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public PhotonTrackedTarget getBestTarget() {
        return limelight.getLatestResult().getBestTarget();
    }

    public Optional<EstimatedRobotPose> getEstimatedPos() {
        return poseEstimator.update();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
