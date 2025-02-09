package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
        
    private VictorSPX fingerMotor;
    private VictorSPX cannonMotor;

    public Indexer() {
        fingerMotor = new VictorSPX(19);
        cannonMotor = new VictorSPX(20);
    }

    public void setFingerAndCannon(boolean up, boolean down, double blast, double retract) {
        if(up) {
            fingerMotor.set(ControlMode.PercentOutput, 0.1);
        } else if(down) {
            fingerMotor.set(ControlMode.PercentOutput, -0.1);
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
}
