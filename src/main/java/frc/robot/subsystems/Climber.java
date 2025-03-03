package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Climber extends SubsystemBase {

    private SparkMax climberMotor;

    private RelativeEncoder encoder;

    private DigitalInput limitSwitch;

    public Climber() {
        climberMotor = new SparkMax(18, MotorType.kBrushless);

        encoder = climberMotor.getEncoder();
    
        limitSwitch = new DigitalInput(7);
    }

    public void setMotor(boolean attach, boolean release) {
        if(attach) {
            if(limitSwitch.get()) {
                climberMotor.set(0.0);
            } else {
                climberMotor.set(1.0);
            }
        } else if(release) {
            climberMotor.set(-1.0);
        } else {
            climberMotor.set(0.0);
        }
    }

    public Trigger getSwitch() {
        return new Trigger(() -> limitSwitch.get());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ClimberEncoder", encoder.getPosition());
        SmartDashboard.putBoolean("Switch Climber", limitSwitch.get());
        SmartDashboard.putBoolean("Trigger Limit Switch", getSwitch().getAsBoolean());
    }
}
