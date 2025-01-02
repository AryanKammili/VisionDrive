package frc.robot.drive.Gyro;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.drive.SwerveConstants;

public class GyroHardware {

    private Pigeon2 gyroscope;
    private StatusSignal<Double> yaw;

    @AutoLog
    public static class GyroInputs {
        public boolean gyroConnected = true;
        public Rotation2d yaw = new Rotation2d();
    }
    
    public GyroHardware(){
        gyroscope = new Pigeon2(SwerveConstants.gyroPort);
        gyroscope.getConfigurator().apply(new Pigeon2Configuration());
        gyroscope.setYaw(0);

        yaw = gyroscope.getYaw();
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, yaw);
        gyroscope.optimizeBusUtilization();
    }

    public void updateInputs(GyroInputs inputs){
        inputs.gyroConnected = BaseStatusSignal.refreshAll(yaw).isOK();
        inputs.yaw = new Rotation2d(yaw.getValueAsDouble());
    }

    public void setYaw(double yaw){
        gyroscope.setYaw(yaw);
    }
}