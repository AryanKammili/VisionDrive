package frc.robot.drive.SwerveModule;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drive.SwerveConstants;
import frc.robot.drive.SwerveConstants.SwerveModuleHardwareConfig;
import frc.robot.util.debugging.LoggedTunableNumber;

public class SwerveModule extends SubsystemBase {

    public static final LoggedTunableNumber driveP = new LoggedTunableNumber("Module/Drive/kP", SwerveConstants.driveControllerConfig.kP());
    public static final LoggedTunableNumber driveD = new LoggedTunableNumber("Module/Drive/kD", SwerveConstants.driveControllerConfig.kD());
    public static final LoggedTunableNumber driveS = new LoggedTunableNumber("Module/Drive/kS", SwerveConstants.driveControllerConfig.kS());
    public static final LoggedTunableNumber driveV = new LoggedTunableNumber("Module/Drive/kV", SwerveConstants.driveControllerConfig.kV());
    public static final LoggedTunableNumber driveA = new LoggedTunableNumber("Module/Drive/kA", SwerveConstants.driveControllerConfig.kA());

    public static final LoggedTunableNumber azimuthP = new LoggedTunableNumber("Module/Drive/kP", SwerveConstants.azimuthControllerConfig.kP());
    public static final LoggedTunableNumber azimuthD = new LoggedTunableNumber("Module/Drive/kD", SwerveConstants.azimuthControllerConfig.kD());
    public static final LoggedTunableNumber azimuthS = new LoggedTunableNumber("Module/Drive/kS", SwerveConstants.azimuthControllerConfig.kS());
    public static final LoggedTunableNumber azimuthV = new LoggedTunableNumber("Module/Drive/kV", SwerveConstants.azimuthControllerConfig.kV());
    public static final LoggedTunableNumber azimuthA = new LoggedTunableNumber("Module/Drive/kA", SwerveConstants.azimuthControllerConfig.kA());

    private Double velocitySetpointMPS = null;
    private Rotation2d azimuthSetpointAngle = null;

    private SwerveModuleState currentState = new SwerveModuleState();
    private SwerveModulePosition currentPosition = new SwerveModulePosition();

    private SwerveModuleHardware io;
    private SwerveModuleInputsAutoLogged inputs = new SwerveModuleInputsAutoLogged();


    private String nameKey;

    public SwerveModule(SwerveModuleHardwareConfig config){
        this.io = new SwerveModuleHardware(config);
        nameKey = "Module/" + io.getName();
    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Drive/"+ nameKey, inputs);

        currentState = new SwerveModuleState(inputs.driveVelocityMPS, inputs.cancoderPosistion);
        currentPosition = new SwerveModulePosition(inputs.drivePosistionM, inputs.cancoderPosistion);

        if(velocitySetpointMPS != null){
            io.setDriveVelocity(velocitySetpointMPS);
        }

        if(azimuthSetpointAngle != null){
            io.setAzimuthPosistion(azimuthSetpointAngle.getRotations());
        }

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            io.setDriveConstants(driveP.get(), driveD.get(), driveS.get(), driveV.get(), driveA.get());
        }, driveP, driveD, driveS, driveV, driveA);

        LoggedTunableNumber.ifChanged(hashCode(), () -> {
            io.setAzimuthConstants(azimuthP.get(), azimuthD.get(), azimuthS.get(), azimuthV.get(), azimuthA.get());
        }, azimuthP, azimuthD, azimuthS, azimuthV, azimuthA);

    }

    public void resetAzimuthEncoder(){
        io.resetAzimuthPosistion();
    }

    public void runLinearCharacterization(double inputVolts) {
        setAzimuthPosistion(new Rotation2d());
        setDriveVelocity(null);
        setDriveVolts(inputVolts);
    }

    public SwerveModuleState setSwerveState(SwerveModuleState state){
        setDriveVelocity(state.speedMetersPerSecond);
        setAzimuthPosistion(state.angle);
        return new SwerveModuleState(velocitySetpointMPS, azimuthSetpointAngle);
    }

    public SwerveModuleState getCurrentState(){
        return currentState;
    }

    public SwerveModulePosition getCurrentPosistion(){
        return currentPosition;
    }

    public void setDriveVelocity(Double velocityDemand){
        velocitySetpointMPS = velocityDemand;
    }

    public void setDriveVolts(Double volts){
        io.setDriveVolts(volts);
    }

    public void setAzimuthPosistion(Rotation2d posistionDemand){
        azimuthSetpointAngle = posistionDemand;
    }

    public void setAzimuthVolts(Double volts){
        io.setAzimuthVolts(volts);
    }

    public SwerveModuleInputsAutoLogged getInputs(){
        return inputs;
    }

    public void stop(){
        io.setAzimuthVolts(0);
        io.setDriveVolts(0);
    }

}