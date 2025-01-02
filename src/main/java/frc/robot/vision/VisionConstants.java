package frc.robot.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {

    public static final String kLeftCamName = "Example Camera";
    // The camera transform is the 3d distances between the center of the robot and the camera //
    // The camera rotation gets the different oreientations of the camera //
    public static final Transform3d kLeftCamTransform = new Transform3d(
        new Translation3d(0, 0, 0),
        new Rotation3d(0.0, 0.0, 0.0)
    );

    public static final Vector<N3> kSingleStdDevs = VecBuilder.fill(0.01, 0.01, 5.0);
    public static final Vector<N3> kMultiStdDevs = VecBuilder.fill(0.01, 0.01, 5.0);

    // Any data with an ambiguity value above 0.3 does not have enough certainity and will not be used // 
    public static final double kAmbiguityThreshold = 0.3;
}
