package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
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

        base = LEDPattern.solid(getAllianceColor()).atBrightness(Percent.of(50))
        .breathe(Seconds.of(5));
        
        isDeployedPattern = base;


        isDeployedPattern.applyTo(m_buffer);

        m_led.setData(m_buffer);
        m_led.start();
        
    }

        public Color getAllianceColor() {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            return Color.kRed;
        } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
            return Color.kBlue;
        } else {
            return Color.kMaroon;
        }
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
