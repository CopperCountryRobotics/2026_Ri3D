package frc.robot;

import java.lang.annotation.Target;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
    private PhotonCamera camera = new PhotonCamera("Camera");

    public double latestShooterPos = 0;
    public boolean seesShooter = false;
    public double latestShooterYaw = 0;

    public Vision() {
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    public double getYaw() {
        try {
            return camera.getLatestResult().getBestTarget().getYaw();
        } catch (Exception ex) {
            return 0;
        }
    }

    public double getSkew() {
        try {
            return camera.getLatestResult().getBestTarget().getSkew();
        } catch (Exception ex) {
            return 0;
        }
    }

    public double getPitch() {
        try {
            return camera.getLatestResult().getBestTarget().getPitch();
        } catch (Exception ex) {
            return 0;
        }
    }

    public double getFiducialId() {
        try {
            return camera.getLatestResult().getBestTarget().getFiducialId();
        } catch (Exception ex) {
            return 0;
        }
    }

    public double getTagPoseX() {
        try {
            return camera.getLatestResult().getBestTarget().getBestCameraToTarget().getX();
        } catch (Exception ex) {
            return 0;
        }
    }

    public double getShooterPose() {
        double shooterPose = 0;
        try {
            if (camera.getLatestResult().hasTargets()) {

                for (PhotonTrackedTarget target : camera.getLatestResult().getTargets()) {
                    if (target.getFiducialId() == 10 | target.getFiducialId() == 26) {
                        shooterPose = target.getBestCameraToTarget().getX();
                        break;
                    }
                }
            }
            return shooterPose;
        } catch (Exception ex) {
            return 0;
        }
    }

    public double getShooterYaw(){
        double shooterYaw = 0;
        try {
            if (camera.getLatestResult().hasTargets()) {

                for (PhotonTrackedTarget target : camera.getLatestResult().getTargets()) {
                    if (target.getFiducialId() == 10 | target.getFiducialId() == 26) {
                        shooterYaw = target.getYaw();
                        break;
                    }
                }
            }
            return shooterYaw;
        } catch (Exception ex) {
            return 0;
        }
    }

    public double getTagPoseY() {
        try {
            return camera.getLatestResult().getBestTarget().getBestCameraToTarget().getY();
        } catch (Exception ex) {
            return 0;
        }
    }

    public double getTagPoseZ() {
        try {
            return camera.getLatestResult().getBestTarget().getBestCameraToTarget().getZ();
        } catch (Exception ex) {
            return 0;
        }
    }



    public void updateDashboard() {
        if(getShooterPose() == 0){
            seesShooter = false;
        } else {
            seesShooter = true;
            latestShooterPos = getShooterPose();
            latestShooterYaw = getShooterYaw();
        }

        SmartDashboard.putNumber("yaw", getYaw());
        SmartDashboard.putNumber("tag pose x", getTagPoseX());
        SmartDashboard.putNumber("tag pose y", getTagPoseY());
        SmartDashboard.putNumber("pitch", getPitch());
        SmartDashboard.putNumber("fiducial id", getFiducialId());
        SmartDashboard.putNumber("Vision Shooter Pose", getShooterPose());
        SmartDashboard.putNumber("Vision Shooter Yaw", getShooterYaw());


    }
}
