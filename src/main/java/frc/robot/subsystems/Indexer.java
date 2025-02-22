package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
        
    private VictorSPX fingerMotor;
    private VictorSPX cannonMotor;

    private ColorSensorV3 colorSensor;

    public Indexer() {
        fingerMotor = new VictorSPX(19);
        cannonMotor = new VictorSPX(20);
        colorSensor = new ColorSensorV3(Port.kOnboard);
    }

    public void setFingerAndCannon(boolean up, boolean down, double blast, double retract) {
        if(up) {
            fingerMotor.set(ControlMode.PercentOutput, 0.5);
        } else if(down) {
            fingerMotor.set(ControlMode.PercentOutput, -0.5); 
        } else {
            fingerMotor.set(ControlMode.PercentOutput, 0.0);
        }

        if(blast > 0.05) {
            cannonMotor.set(ControlMode.PercentOutput, blast);
        } else if(retract > 0.05) {
            cannonMotor.set(ControlMode.PercentOutput, -retract);
        } else {
            cannonMotor.set(ControlMode.PercentOutput, 0.0);
        }
    }

    public void turnOffAllMotors() {
        fingerMotor.set(ControlMode.PercentOutput, 0.0);
        cannonMotor.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Color Sensor 1", colorSensor.getProximity());
    }
}
