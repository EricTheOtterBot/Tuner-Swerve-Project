package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpaceGun extends SubsystemBase {

    private TalonSRX spaceGunMotor;

    public SpaceGun() {
        spaceGunMotor = new TalonSRX(40);
    }

    public void shootMotor(double speed) {
            spaceGunMotor.set(TalonSRXControlMode.PercentOutput, speed);
        
    }
    
}
