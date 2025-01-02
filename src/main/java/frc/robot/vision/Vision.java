package frc.robot.vision;

import java.util.Vector;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

public class Vision {

    // Holds observation from each of the cameras //
    // The data from the vision observation can be fed into the pose estimator used for the robot //
    public record VisionObservation(boolean hasObserved, Pose2d pose, Vector<N3> stdDevs, double timeStamp, String camName) {}
    
    private VisionHardware[] cameras;
    private VisionInputsAutoLogged[] camerasData;

    public Vision(VisionHardware[] cameras){
        this.cameras = cameras;
        this.camerasData = new VisionInputsAutoLogged[cameras.length];

        for(int i = 0; i < cameras.length; i++){
            camerasData[i] = new VisionInputsAutoLogged();
        }
    }

    public void periodic(Pose2d lastRobotPose){
        for(int i = 0; i < cameras.length; i++){
            // Process inputs for each of the cameras //
            cameras[i].updateInputs(camerasData[i], lastRobotPose);
            Logger.processInputs("Vision/" + camerasData[i].cameraName , camerasData[i]);

            // Log pose estiamte along with the rotation estimate //
            Logger.recordOutput("Vision/"+camerasData[i].cameraName+"/Pose", camerasData[i].estimatedRobotPose.toPose2d());
            Logger.recordOutput("Vision/"+camerasData[i].cameraName+"/X (roll)", camerasData[i].estimatedRobotPose.getRotation().getX());
            Logger.recordOutput("Vision/"+camerasData[i].cameraName+"/Y (pitch)", camerasData[i].estimatedRobotPose.getRotation().getY());
            Logger.recordOutput("Vision/"+camerasData[i].cameraName+"/Z (yaw)", camerasData[i].estimatedRobotPose.getRotation().getZ());
        }
    }

    public VisionObservation[] getVisionObservations(){
        VisionObservation[] observations = new VisionObservation[cameras.length];
        int j = 0;

        // To calculate the standard deviation dynamically from multiple variables //
        // Variables -> Timestamp, standard deviations, ambiguities //
        // If the ambiguity is too high then the target is not going to be used //
        // If the signle target is too far (3.5m) it shouldn't be used //
        // Std Deviation = (avg distance) ^ 2 / # of tags //

        for(VisionInputsAutoLogged data : camerasData){
            if(data.hasTarget){
                double numTargets = data.numberOfTargets;
                double avgDistance = 0.0;

                for(int i = 0; i < data.tagTransforms.length; i++){
                    if(data.tagTransforms != null){
                        if(data.tagAmbiguities[i] < VisionConstants.kAmbiguityThreshold){
                            // Gets Distance from the origin //
                            avgDistance += data.tagTransforms[i].getTranslation().getNorm();
                        }

                        else{
                            numTargets -= 1;
                        }
                    }
                }

                if(numTargets == 0) {
                    observations[j] = new VisionObservation(
                        true, 
                        data.estimatedRobotPose.toPose2d(), 
                        VecBuilder.fill(
                            Double.MAX_VALUE, 
                            Double.MAX_VALUE, 
                            Double.MAX_VALUE), 
                        data.latestTimestap, 
                        data.cameraName);
                }
            }
        }
    }
}
