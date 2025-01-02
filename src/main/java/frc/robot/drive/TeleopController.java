package frc.robot.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.debugging.LoggedTunableNumber;

public class TeleopController {

    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private DoubleSupplier omegaSupplier;

    private boolean fieldRelative = true;

    public static final LoggedTunableNumber linearDeadband = new LoggedTunableNumber("Drive/Teleop/LinearDeadband", 1);
    public static final LoggedTunableNumber omegaDeadband = new LoggedTunableNumber("Drive/Teleop/RotationalDeadband", 0.075);

    public static final LoggedTunableNumber linearScalar = new LoggedTunableNumber("Drive/Teleop/LinearScalar", 2);
    public static final LoggedTunableNumber omegaScalar = new LoggedTunableNumber("Drive/Teleop/RotationScalar", 0.5);

    public static final LoggedTunableNumber rotationInputsExponent = new LoggedTunableNumber("Drive/Teleop/RotationInputExponent", 1.0);
    public static final LoggedTunableNumber linearInputsExponent = new LoggedTunableNumber("Drive/Teleop/LinearInputExponent", 0.1);



    public TeleopController(){}

    public void acceptJoystickInputs(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier){
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.omegaSupplier = omegaSupplier;
    }

    public ChassisSpeeds computeChassisSpeeds(Rotation2d robotAngle, ChassisSpeeds currentRelativeSpeeds){
        double xChangedDemand = linearScalar.get() * applyDeadband(xSupplier.getAsDouble(), linearDeadband.get());
        double yChangedDemand = linearScalar.get() * applyDeadband(ySupplier.getAsDouble(), linearDeadband.get());
        double omegaChangedDemand = omegaScalar.get() * applyDeadband(omegaSupplier.getAsDouble(), omegaDeadband.get());

        int linearExp = (int) Math.round(linearInputsExponent.get());
        int rotationExp = (int) Math.round(rotationInputsExponent.get());

        double xVelocityMPS = SwerveConstants.maxLinearSpped * Math.pow(xChangedDemand, linearExp);
        double yVelocityMPS = SwerveConstants.maxLinearSpped * Math.pow(yChangedDemand, linearExp);
        double rotationVelocityRPS = SwerveConstants.maxRadiansPS * Math.pow(omegaChangedDemand, rotationExp);

        if (linearExp % 2 == 0) {
            xVelocityMPS *= Math.signum(xChangedDemand);
            yVelocityMPS *= Math.signum(yChangedDemand);
        }

        if (rotationExp % 2 == 0) {
            rotationVelocityRPS *= Math.signum(omegaChangedDemand);
        }

        ChassisSpeeds desiredSpeeds = new ChassisSpeeds( 
            xVelocityMPS, 
            yVelocityMPS, 
            rotationVelocityRPS);

        if(fieldRelative){
            desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(desiredSpeeds, robotAngle);
        }

        return desiredSpeeds;
    }

    public void toggleFieldOrientation(){
        fieldRelative = !fieldRelative;
    }

    private double applyDeadband(double number, double deadband){
        if(-deadband < number && number < deadband){
            return number;
        }
        return 0;
    }
}