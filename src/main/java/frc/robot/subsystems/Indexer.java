package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
        
    private VictorSPX fingerMotor;
    private VictorSPX cannonMotor;
    private DigitalInput bottomFingerSwitch;
    private DigitalInput topFingerSwitch;

    private ColorSensorV3 colorSensor;

    public Indexer() {
        fingerMotor = new VictorSPX(19);
        cannonMotor = new VictorSPX(20);
        bottomFingerSwitch = new DigitalInput(6);
        topFingerSwitch = new DigitalInput(7);
        colorSensor = new ColorSensorV3(Port.kOnboard);
    }

    public void setFingerAndCannon(boolean up, boolean down, double blast, double retract) {
        if(up) {
            if(topFingerSwitch.get()) {
               fingerMotor.set(ControlMode.PercentOutput, 0.1); 
            } else {
                fingerMotor.set(ControlMode.PercentOutput, 0.5);
            }

            
        } else if(down) {
            if(bottomFingerSwitch.get()) {
                fingerMotor.set(ControlMode.PercentOutput, 0.0);
            } else {
                fingerMotor.set(ControlMode.PercentOutput, -0.5);
            }
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

    public BooleanSupplier getTopFingerSwitch() {
        return () -> topFingerSwitch.get();
    }

    public BooleanSupplier getBottomFingerSwitch() {
        return () -> bottomFingerSwitch.get();
    }

    public BooleanSupplier isThereNoCoral() {
        if(colorSensor.getProximity() < 250) {
            return () -> true;
        } else { 
            return () -> false;
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Color Sensor 1", colorSensor.getProximity());
        SmartDashboard.putBoolean("Top Finger", topFingerSwitch.get());
        SmartDashboard.putBoolean("Bottom Finger", bottomFingerSwitch.get());
    }
}
