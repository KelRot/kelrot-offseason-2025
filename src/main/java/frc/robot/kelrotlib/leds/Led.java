package frc.robot.kelrotlib.leds;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;

public class Led extends SubsystemBase{ //a Java inheritance example
    private AddressableLED m_led;
    private AddressableLEDBuffer m_buffer;
    private AddressableLEDBufferView[] m_groupList;
    private LEDPattern[] m_patternList;
    private LEDPattern k_defaultPattern;

    public Led(int[][] ledGroups) {
        m_led = new AddressableLED(LedConstants.kLedPort); //every variable you might change later (ports, length etc.) should be added to Constants.java*
        m_buffer = new AddressableLEDBuffer(LedConstants.kLedLength); // * this way every port and important variable is in one place.
        m_led.setLength(LedConstants.kLedLength); //setting the length takes a lot of load so do it only one time when possible
        m_led.start();
        
        k_defaultPattern = LEDPattern.solid(Color.kBlack);
        
        m_groupList = new AddressableLEDBufferView[ledGroups.length]; //create a new groupList with the same length as the ledGroups array
        m_patternList = new LEDPattern[ledGroups.length]; //create a new patternList with the same length as the ledGroups array
        for (int i = 0; i < ledGroups.length; i++) {
            createGroup(ledGroups[i][0], ledGroups[i][1], i);
        }

        setDefaultCommand(runPattern(k_defaultPattern).withName("Off")); //set all leds to off/black on start
    }

    public void createGroup(int startingIndex, int endingIndex, int groupID) {
        AddressableLEDBufferView group = m_buffer.createView(startingIndex, endingIndex); //create new group(view)
        m_groupList[groupID] = group; //add the group to the groupList
        m_patternList[groupID] = k_defaultPattern; //add the default pattern to the patternList
    }

    public void setSolidColor(Color color, int[] groupIDs){
        LEDPattern solidColorPattern = LEDPattern.solid(color);
        for (int i = 0; i < groupIDs.length; i++) { //update the latest pattern for every LED group given
            m_patternList[groupIDs[i]] = solidColorPattern; //replace the pattern for the groupID with the new solid color pattern
        }
        runPattern().schedule();
    }

    public void setBlinkColor(Color color, int[] groupIDs){
        LEDPattern base = LEDPattern.solid(color);
        LEDPattern blinkPattern = base.blink(Milliseconds.of(LedConstants.kLedBlinkInterval)); //synchronised blink, wpilib also has support for asynchronised blink
        for (int i = 0; i < groupIDs.length; i++) {
            m_patternList[groupIDs[i]] =  blinkPattern;
        }
        runPattern().schedule();
    }

    public void rainbow(int[] groupIDs){
        LEDPattern rainbow = LEDPattern.rainbow(255, 128); //all hues at maximum saturation and *half* brightness
        LEDPattern scrollingRainbow = rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LedConstants.kLedSpacing); //moves/scrolls the effect at a speed of 1 meter per second
        for (int i = 0; i < groupIDs.length; i++) {
            m_patternList[groupIDs[i]] = scrollingRainbow;
        }
        runPattern().schedule();
    }

    public void turnOff(){
        setSolidColor(Color.kBlack, new int[] {0, 1, 2}); //turn off all LEDs by setting them to black
    }

    public Command runPattern(){  //A command is used as it doesn't allow actions to run simultaneously, for this usage it is crucial, because we need to stop the previous patterns and start the new ones.
        return run(() -> {
            for (int i = 0; i < m_groupList.length; i++) {
                m_patternList[i].applyTo(m_groupList[i]); //apply the pattern to the group
            }
        }).ignoringDisable(true);
    }

    private Command runPattern(LEDPattern pattern) {
        return run(() -> {
            pattern.applyTo(m_buffer);
        }).ignoringDisable(true);
    }

    @Override
    public void periodic() { 
        m_led.setData(m_buffer);
    }
  
    @Override
    public void simulationPeriodic() {
    }
}