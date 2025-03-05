package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

    private static final int kPort = 9;
    private static final int kLength = 100;

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_buffer;
    private LEDPattern base;
    private LEDPattern isDeployedPattern;

    public LED() {
        m_led = new AddressableLED(kPort);
        m_buffer = new AddressableLEDBuffer(kLength);
        m_led.setLength(kLength);
        // isDeployedPattern = LEDPattern.kOff;
        //isDeployedPattern.applyTo((m_buffer));

        base = LEDPattern.kOff;
        
        isDeployedPattern  = base; //.scrollAtRelativeSpeed(Percent.per(Second).of(25)).atBrightness(Percent.of(0))


        isDeployedPattern.applyTo(m_buffer);

        m_led.setData(m_buffer);
        m_led.start();
        
    }

    public void runPattern(LEDPattern pattern) {
        pattern.applyTo(m_buffer);
        m_led.setData(m_buffer);
    }

    @Override
    public void periodic() {
        //isDeployedPattern.applyTo(m_buffer);
        m_led.setData(m_buffer);
        
    }
    
}
