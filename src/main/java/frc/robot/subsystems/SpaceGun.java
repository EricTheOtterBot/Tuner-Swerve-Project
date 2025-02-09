package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpaceGun extends SubsystemBase {

    private SparkMax spaceGunMotor;

    public SpaceGun() {
        spaceGunMotor = new SparkMax(12, MotorType.kBrushless);
    }

    public void shootMotor(double speed) {
            spaceGunMotor.set(speed);
        
    }
    
}
