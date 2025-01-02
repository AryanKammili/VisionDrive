package frc.robot.drive.SwerveModule;

import org.littletonrobotics.junction.AutoLog;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.drive.SwerveConstants;
import frc.robot.drive.SwerveConstants.SwerveModuleHardwareConfig;

public class SwerveModuleHardware {

  // Robot Specific info //
  private String name;
  private Rotation2d angleOffset;

  // Hardware //
  private TalonFX azimuth;
  private TalonFX drive;
  private CANcoder cancoder;

  private SwerveConfigs config = new SwerveConfigs();

  // Status Signals //
  private StatusSignal<Double> driveVelocity;
  private StatusSignal<Double> driveVoltage;
  private StatusSignal<Double> drivePosistion;
  private StatusSignal<Double> driveStatorCurrent;
  private StatusSignal<Double> driveSupplyCurrent;
  private StatusSignal<Double> driveTempC;

  private StatusSignal<Double> azimuthPosistion;
  private StatusSignal<Double> azimuthVoltage;
  private StatusSignal<Double> azimuthStatorCurrent;
  private StatusSignal<Double> azimuthSupplyCurrent;
  private StatusSignal<Double> azimuthTempC;

  private StatusSignal<Double> cancoderPosistion;

  // Control //
  private VoltageOut azimuthVoltageControl;
  private VoltageOut driveVoltageControl;
  private PositionDutyCycle azimuthVoltagePosistion;
  private VelocityVoltage driveVelocityControl;

  // AutLog these values //
  @AutoLog
  public static class SwerveModuleInputs {
    public boolean driveConnected = true;
    public double driveVelocityMPS = 0.0;
    public double drivePosistionM = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveMotorVolts = 0.0;
    public double[] driveStator = {0.0};
    public double[] driveSupply = {0.0};
    public double[] driveTempC = {0.0};

    public boolean azimuthConnected = true;
    public Rotation2d azimuthPosistion = new Rotation2d();
    public double azimuthAppliedVolts = 0.0;
    public double azimuthMotorVolts = 0.0;
    public double[] azimuthStator = {0.0};
    public double[] azimuthSupply = {0.0};
    public double[] azimuthTempC = {0.0};

    public boolean cancoderConnected = true;
    public Rotation2d cancoderPosistion = new Rotation2d();
  }

  public SwerveModuleHardware(SwerveModuleHardwareConfig modConstants){

    name = modConstants.name();
    angleOffset = modConstants.offset();
    
    azimuth = new TalonFX(modConstants.azimuthPort());
    drive = new TalonFX(modConstants.drivePort());
    cancoder = new CANcoder(modConstants.cancoderPort());

    config.configAzimuth(azimuth, (false));
    config.configDrive(drive, (false));
    config.configCANcoder(cancoder);

    azimuthVoltageControl = new VoltageOut(0);
    driveVoltageControl = new VoltageOut(0);
    azimuthVoltagePosistion = new PositionDutyCycle(0);
    driveVelocityControl = new VelocityVoltage(0);

    driveVelocity = drive.getVelocity();
    drivePosistion = drive.getPosition();
    driveVoltage = drive.getMotorVoltage();
    driveStatorCurrent = drive.getStatorCurrent();
    driveSupplyCurrent = drive.getSupplyCurrent();
    driveTempC = drive.getDeviceTemp();

    azimuthPosistion = azimuth.getPosition();
    azimuthVoltage = azimuth.getMotorVoltage();
    azimuthStatorCurrent = azimuth.getStatorCurrent();
    azimuthSupplyCurrent = azimuth.getSupplyCurrent();
    azimuthTempC = azimuth.getDeviceTemp();

    cancoderPosistion = cancoder.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
      50.0, 
      driveVelocity, 
      driveVoltage, 
      driveStatorCurrent, 
      driveSupplyCurrent,
      driveTempC,
      azimuthPosistion, 
      azimuthVoltage, 
      azimuthStatorCurrent, 
      azimuthSupplyCurrent,
      azimuthTempC,
      cancoderPosistion);

    azimuth.optimizeBusUtilization();
    drive.optimizeBusUtilization();
    cancoder.optimizeBusUtilization();

    resetAzimuthPosistion();
  }

  public void updateInputs(SwerveModuleInputs inputs){
    inputs.driveConnected = BaseStatusSignal.refreshAll(
      driveVelocity,
      driveVoltage,
      driveStatorCurrent,
      driveSupplyCurrent,
      driveTempC).isOK();

    inputs.driveVelocityMPS = (driveVelocity.getValueAsDouble());
    inputs.drivePosistionM = drivePosistion.getValueAsDouble();
    inputs.driveMotorVolts = driveVoltage.getValueAsDouble();
    inputs.driveStator = new double[] {driveStatorCurrent.getValueAsDouble()};
    inputs.driveSupply = new double[] {driveSupplyCurrent.getValueAsDouble()};
    inputs.driveTempC = new double[] {driveTempC.getValueAsDouble()};

    inputs.azimuthConnected = BaseStatusSignal.refreshAll(
      azimuthPosistion,
      azimuthVoltage,
      azimuthStatorCurrent,
      azimuthSupplyCurrent,
      azimuthTempC).isOK();

    inputs.azimuthPosistion = new Rotation2d(azimuthPosistion.getValueAsDouble());
    inputs.azimuthMotorVolts = azimuthVoltage.getValueAsDouble();
    inputs.azimuthStator = new double[] {azimuthStatorCurrent.getValueAsDouble()};
    inputs.azimuthSupply = new double[] {azimuthSupplyCurrent.getValueAsDouble()};
    inputs.azimuthTempC = new double[] {azimuthTempC.getValueAsDouble()};

    inputs.cancoderConnected = BaseStatusSignal.refreshAll(cancoderPosistion).isOK();
    inputs.cancoderPosistion = new Rotation2d(cancoderPosistion.getValueAsDouble());
  }

  public String getName(){
    return name;
  }

  public void setAzimuthVolts(double volts) {
    azimuth.setControl(azimuthVoltageControl.withOutput(volts));
  }

  public void setAzimuthPosistion(double posistion) {
    drive.setControl(azimuthVoltagePosistion.withPosition(posistion));
  }

  public void setAzimuthConstants(double kP, double kD, double kS, double kV, double kA){
    Slot0Configs configs = new Slot0Configs();
    configs.kP = kP;
    configs.kD = kD;
    configs.kS = kS;
    configs.kV = kV;
    configs.kA = kA;
    azimuth.getConfigurator().apply(configs);
  }

  public void setDriveVolts(double volts) {
    drive.setControl(driveVoltageControl.withOutput(volts));
  }

  public void setDriveVelocity(double velocityMPS) {
    drive.setControl(driveVelocityControl.withVelocity(velocityMPS));
  }

  public void setDriveConstants(double kP, double kD, double kS, double kV, double kA){
    Slot0Configs configs = new Slot0Configs();
    configs.kP = kP;
    configs.kD = kD;
    configs.kS = kS;
    configs.kV = kV;
    configs.kA = kA;
    drive.getConfigurator().apply(configs);
  }

  public void resetAzimuthPosistion() {
    azimuth.setPosition(Rotation2d.fromRotations(
        cancoderPosistion.getValueAsDouble())
        .minus(angleOffset).getRotations() * SwerveConstants.azimuthGearing);
  }
}