package frc.robot.vision;

import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionHardware{

    @AutoLog
    public static class VisionInputs{
        public String cameraName = "";
        public boolean isConnected = false;
        public double yaw = 0.0;
        public double pitch = 0.0;
        public double area = 0.0;
        public double latencySeconds = 0.0;
        public boolean hasTarget = false;
        public int numberOfTargets = 0;

        public Transform3d cameraToAprilTag = new Transform3d();
        public double poseAmbiguity = 0.0;
        public int aprilTagID = 0;
        public Transform3d robotToAprilTag = new Transform3d();
        public double latestTimestap = 0.0;
        public Pose3d estimatedRobotPose = new Pose3d();

        // This array would change to be shorter depending on how many april tags are detected //
        public Transform3d[] tagTransforms = new Transform3d[] {
            new Transform3d(), new Transform3d(), new Transform3d(), new Transform3d(), 
            new Transform3d(), new Transform3d(), new Transform3d(), new Transform3d(), 
            new Transform3d(), new Transform3d(), new Transform3d(), new Transform3d(), 
            new Transform3d(), new Transform3d()};

        // Same as this one //
        // The ambiguity refers to how certain the camera is able to determine the posistion of the tag //
        // Higher value -> bad , Lower value -> good //
        public double[] tagAmbiguities = new double[] {
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }

    private String cameraName;
    private PhotonCamera camera;
    private PhotonPoseEstimator poseEstimator;
    private Transform3d cameraTransform;

    public VisionHardware(String cameraName, Transform3d cameraTransform){
        this.cameraName = cameraName;
        camera = new PhotonCamera(cameraName);
        this.cameraTransform = cameraTransform;
        PhotonCamera.setVersionCheckEnabled(false);
        
        poseEstimator = new PhotonPoseEstimator(
            AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo),
            //TODO: If we hook up cameras to orange pi5 change this 
            PoseStrategy.MULTI_TAG_PNP_ON_RIO, 
            cameraTransform);
        // If the camera can't detect multiple tags then it would use this strategy insteadd //
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public void updateInputs(VisionInputs inputs, Pose2d lastRobotPose){
        inputs.cameraName = cameraName;
        
        try{
            PhotonPipelineResult result = camera.getLatestResult();
            poseEstimator.setLastPose(lastRobotPose);
            Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update(result);
            inputs.isConnected = camera.isConnected();
            inputs.hasTarget = result.hasTargets();

            if(result.hasTargets()){
                PhotonTrackedTarget target = result.getBestTarget();
                inputs.yaw = target.getYaw();
                inputs.pitch = target.getPitch();
                inputs.area = target.getArea();
                inputs.latencySeconds = result.getLatencyMillis() / 1000.0;
                inputs.numberOfTargets = estimatedRobotPose.get().targetsUsed.size();
                inputs.cameraToAprilTag = target.getBestCameraToTarget();
                inputs.poseAmbiguity = target.getPoseAmbiguity();
                inputs.aprilTagID = target.getFiducialId();
                inputs.robotToAprilTag = target.getBestCameraToTarget().plus(cameraTransform);
                inputs.latestTimestap = result.getTimestampSeconds();
                
                estimatedRobotPose.ifPresent(pose -> {
                    // If the camera is facing the back then 180* needs to be added // 
                    inputs.estimatedRobotPose = estimatedRobotPose.get().estimatedPose.transformBy(new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)));

                    ArrayList<Transform3d> tags = new ArrayList<>();
                    double[] ambiguities = new double[estimatedRobotPose.get().targetsUsed.size()];

                    for(int i = 0; i < ambiguities.length; i++){
                        tags.add(estimatedRobotPose.get().targetsUsed.get(i).getBestCameraToTarget());
                        ambiguities[i] = estimatedRobotPose.get().targetsUsed.get(i).getPoseAmbiguity();
                    }

                    inputs.tagAmbiguities = ambiguities;
                    // Transfters the list into an array to be accepted by the input //
                    inputs.tagTransforms = tags.toArray(Transform3d[]::new);
                });
            }
        }

        // This would happen if the camera was disconnected :( //
        catch(Exception error){
            // Print out error to rio logs //
            error.printStackTrace();
            inputs.isConnected = false; // Camera is not connected
            inputs.yaw = 0.0;
            inputs.pitch = 0.0;
            inputs.area = 0.0;
            inputs.latencySeconds = 0.0;
            inputs.hasTarget = false;
            inputs.numberOfTargets = 0;

            inputs.cameraToAprilTag = new Transform3d();
            inputs.poseAmbiguity = 0.0;
            inputs.aprilTagID = 0;
            inputs.robotToAprilTag = new Transform3d();
            inputs.latestTimestap = 0.0;
            inputs.estimatedRobotPose = new Pose3d();
            inputs.tagTransforms = new Transform3d[14];
            inputs.tagAmbiguities = new double[14];
        }
    }
}
