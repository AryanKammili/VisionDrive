package frc.robot.drive;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.drive.Gyro.GyroHardware;
import frc.robot.drive.Gyro.GyroInputsAutoLogged;
import frc.robot.drive.SwerveModule.SwerveModule;
import frc.robot.util.debugging.SysIDCharacterization;

public class Drive extends SubsystemBase{

    public static enum DriveState {
        TELEOP,
        SYS_ID,
        SNIPER_UP,
        SNIPER_DOWN,
        SNIPER_RIGHT,
        SNIPER_LEFT,
        STOP,
        AUTON
    }

    private SwerveModule[] modules; 
    private GyroHardware gyro;
    private final GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();
    
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    private ChassisSpeeds autonDesiredSpeeds = new ChassisSpeeds();

    private Rotation2d robotRotation;
    private SwerveDriveOdometry swerveOdometry;
    private SwerveDrivePoseEstimator swervePoseEstimator;
    private Field2d field = new Field2d();

    private TeleopController teleopController = new TeleopController();

    private PIDConstants translationPathplannerConstants = new PIDConstants(0.0, 0.0, 0.0);
    private PIDConstants rotationPathplannerConstants = new PIDConstants(0.0, 0.0, 0.0);

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.ModuleTranslations);

    @AutoLogOutput(key="Drive/CurrentState")
    private DriveState driveState = DriveState.TELEOP;
    
    public Drive(SwerveModule[] modules, GyroHardware gyro){
        this.modules = modules;
        this.gyro = gyro;
        robotRotation = gyroInputs.yaw;

        swerveOdometry = new SwerveDriveOdometry(kinematics, getRobotRotation(), getModulePositions());
        swervePoseEstimator = new SwerveDrivePoseEstimator(kinematics, getRobotRotation(), getModulePositions(), new Pose2d());

       AutoBuilder.configureHolonomic(
            this::getEstimatedPose,
            this::setPose,
            () -> SwerveConstants.kinematics.toChassisSpeeds(getModuleStates()),
            (speeds) -> autonDesiredSpeeds = speeds,
            new HolonomicPathFollowerConfig(
                translationPathplannerConstants,
                rotationPathplannerConstants,
                SwerveConstants.maxLinearSpped,
                SwerveConstants.kDrivebaseRadiusMeters,
                new ReplanningConfig(true, false)),
            () -> DriverStation.getAlliance().isPresent() && 
                DriverStation.getAlliance().get() == Alliance.Red,
            this);

    }  
    
    @Override
    public void periodic(){
        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        for(SwerveModule module: modules){
            module.periodic();

            if(DriverStation.isDisabled()){
                module.stop();
            }
        }

        robotRotation = gyroInputs.yaw;

        swervePoseEstimator.update(robotRotation, getModulePositions());
        swerveOdometry.update(robotRotation, getModulePositions());

        field.setRobotPose(getEstimatedPose());

        ChassisSpeeds teleopSpeeds = teleopController.computeChassisSpeeds((getEstimatedPose().getRotation()), getChassisSpeeds());

        switch (driveState){
            case TELEOP:
                desiredSpeeds = teleopSpeeds;
                break;

            case SYS_ID:
                break;

            case SNIPER_UP:
                desiredSpeeds = new ChassisSpeeds(0.5, 0, 0);
                break;

            case SNIPER_DOWN:
                desiredSpeeds = new ChassisSpeeds(-0.5, 0, 0);
                break;

            case SNIPER_RIGHT:
                desiredSpeeds = new ChassisSpeeds(0, 0.5, 0);
                break;

            case SNIPER_LEFT:
                desiredSpeeds = new ChassisSpeeds(0, -0.5, 0);
                break;

            case AUTON:
                desiredSpeeds = new ChassisSpeeds(
                    autonDesiredSpeeds.vxMetersPerSecond, 
                    autonDesiredSpeeds.vyMetersPerSecond,
                    autonDesiredSpeeds.omegaRadiansPerSecond);
                break;

            case STOP:
                desiredSpeeds = new ChassisSpeeds();
                break;

        }

    }  

    public void setDriveEnum(DriveState state){
        driveState = state;
    }

    public Command setDriveStateCommand(DriveState state){
        return Commands.runOnce(() -> setDriveEnum(state), this);
    }

    public Command setDriveStateCommandContinued(DriveState state){
        return new FunctionalCommand(
            () -> setDriveEnum(state), 
            () -> {}, 
            (interrupted) -> {}, 
            () -> false, 
            this);
    }

    public Command characterizeDriveMotors() {
        return setDriveStateCommand(DriveState.SYS_ID).andThen(
            SysIDCharacterization.runDriveSysIDTests( (voltage) -> {
                for (var module : modules) module.runLinearCharacterization(voltage);
        }, this));
    }

    public void setChassisSpeeds(ChassisSpeeds speeds){
        desiredSpeeds = speeds;
    }

    public void setSwerve(ChassisSpeeds speeds){
        SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxLinearSpped);

        for(int i = 0; i < 4; i++){
            modules[i].setSwerveState(SwerveModuleState.optimize(desiredStates[i], modules[i].getCurrentState().angle));
        }
    }

    public void stop(){
        setSwerve(new ChassisSpeeds());
    } 

    public void resetGyro() {
        robotRotation = Constants.kAlliance == Alliance.Blue ? Rotation2d.fromDegrees(0.0) : Rotation2d.fromDegrees(180.0);
        gyro.setYaw(robotRotation.getDegrees());
        setPose(new Pose2d(getEstimatedPose().getTranslation(), robotRotation));
    }

    public void resetPose(){
        setPose(new Pose2d());
    }

    public void setPose(Pose2d pose){
        robotRotation = pose.getRotation();
        gyro.setYaw(pose.getRotation().getDegrees());
        swerveOdometry.resetPosition(robotRotation, getModulePositions(), pose);
        swervePoseEstimator.resetPosition(robotRotation, getModulePositions(), pose);
    }

    @AutoLogOutput(key = "Drive/Odometry/RobotRotation")
    public Rotation2d getRobotRotation(){
        return robotRotation;
    }

    @AutoLogOutput(key = "Drive/Odometry/GyroRotation")
    public Rotation2d getGyroRotation(){
        return gyroInputs.yaw;
    }

    @AutoLogOutput(key = "Drive/Odometry/PoseEstimate")
    public Pose2d getEstimatedPose(){
        return swervePoseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput(key = "Drive/Odometry/DrivePose")
    public Pose2d getOdometryPose(){
        return swerveOdometry.getPoseMeters();
    }

    @AutoLogOutput(key = "Drive/Swerve/MeasuredPosistion")
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] posistions = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++){
            posistions[i] = modules[i].getCurrentPosistion();
        }

        return posistions;
    }

    @AutoLogOutput(key = "Drive/Swerve/MeasuredStates")
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++){
            states[i] = modules[i].getCurrentState();
        }

        return states;
    }

    @AutoLogOutput(key = "Drive/Odometry/CurrentChassisSpeeds")
    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.kinematics.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "Drive/Odometry/DesiredChassisSpeeds")
    public ChassisSpeeds getDesiredChassisSpeeds() {
        return desiredSpeeds;
    }

    public void resetAllEncoders(){
        for(int i = 0; i < 4; i++){
            modules[i].resetAzimuthEncoder();
        }
    }

    public void acceptJoystickInputs(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier){
        teleopController.acceptJoystickInputs(xSupplier, ySupplier, thetaSupplier);
    }

}