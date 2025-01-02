package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
    // FALSE IT BEFORE COMPETITION
    public static final boolean kTuningMode = true;

    public static final Alliance kAlliance = DriverStation.getAlliance().isPresent() && 
        DriverStation.getAlliance().get() == Alliance.Red ? Alliance.Red : Alliance.Blue;

    // ROBOT SEPCIFIC
    public static final String kCanbusName = "TBD";

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final double kFieldLength = 16.54;

    public static final double kLoopPeriod = 0.02;
}