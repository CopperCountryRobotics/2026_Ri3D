package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {
    private PhotonCamera camera = new PhotonCamera("PC_Camera");

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

    public double getYawByTag(int tagId) {
        try {
            boolean targetFound = false;
            var yaw = 0.0;
            List<PhotonTrackedTarget> targets = camera.getLatestResult().getTargets();
            for (int i = 0; i > targets.size() - 1; i++) {
                if (targetFound) {
                    break;
                } else {
                    if (targets.get(i).getFiducialId() == tagId) {
                        yaw = targets.get(i).getYaw();
                    }
                }
            }
            return yaw;
        } catch (Exception ex) {
            return 0;
        }
    }

    public double getPitchByTag(int tagId) {
        try {
            boolean targetFound = false;
            var yaw = 0.0;
            List<PhotonTrackedTarget> targets = camera.getLatestResult().getTargets();
            for (int i = 0; i > targets.size() - 1; i++) {
                if (targetFound) {
                    break;
                } else {
                    if (targets.get(i).getFiducialId() == tagId) {
                        yaw = targets.get(i).getPitch();
                    }
                }
            }
            return yaw;
        } catch (Exception ex) {
            return 0;
        }
    }

    public void updateDashboard() {
        SmartDashboard.putNumber("yaw", getYaw());
        SmartDashboard.putNumber("tag pose x", getTagPoseX());
        SmartDashboard.putNumber("tag pose y", getTagPoseY());
        SmartDashboard.putNumber("tag pose z", getTagPoseZ());
        SmartDashboard.putNumber("pitch", getPitch());
        SmartDashboard.putNumber("fiducial id", getFiducialId());
        SmartDashboard.putNumber("Tag 10 pitch", getPitchByTag(10));
        SmartDashboard.putNumber("Tag 10 yaw", getYawByTag(10));
    }
}
