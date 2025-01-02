package frc.robot.drive.SwerveModule;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;

public class SwerveConfigs {
    public SwerveConfigs(){}

    public void configAzimuth(TalonFX motor, boolean invert){
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.ClosedLoopGeneral.ContinuousWrap = true;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 0;

        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;

        config.MotorOutput.Inverted = (invert) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.Slot0.kP = 0;
        config.Slot0.kD = 0;

        motor.getConfigurator().apply(config);
    }

    public void configDrive(TalonFX motor, boolean invert){
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.ClosedLoopGeneral.ContinuousWrap = true;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 0;

        config.Voltage.PeakForwardVoltage = 12;
        config.Voltage.PeakReverseVoltage = -12;

        config.MotorOutput.Inverted = (invert) ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        config.Slot0.kP = 0;
        config.Slot0.kD = 0;

        motor.getConfigurator().apply(config);
    }

    public void configCANcoder(CANcoder encoder){
        CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        
        encoder.getConfigurator().apply((config));
    }
}