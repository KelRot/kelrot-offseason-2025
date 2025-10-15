package frc.robot.kelrotlib.utils;

import edu.wpi.first.networktables.*;
import frc.robot.Constants;
import dev.doglog.DogLog;

import java.util.EnumSet;
import java.util.function.Consumer;

public class TunableNumber {
    private final NetworkTableEntry entry;
    public double lastValue;
    private NetworkTableInstance inst = NetworkTableInstance.getDefault();


    /**
     * Constructor with onChange callback
     * @param key          The name of the tunable value
     * @param defaultValue The default value if not set on the dashboard
     * @param onChange     A callback that runs whenever the value changes
     */
    public TunableNumber(String key, double defaultValue, Consumer<Double> onChange) {
        this.lastValue = defaultValue;

        // Get or create the NetworkTable entry for this tunable number
        entry = NetworkTableInstance.getDefault()
                .getTable("SmartDashboard")
                .getEntry(key);

        // Set the initial value on the dashboard
        entry.setDouble(defaultValue);

        // Add a listener to detect changes in the entry
        inst.addListener(
                entry,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                    double newValue = entry.getDouble(lastValue);

                    // If the value changed and tuningMode is ON, accept the change
                    if(this.lastValue != newValue && Constants.tuningMode) {
                        DogLog.log(key, entry.getDouble(lastValue));
                        System.out.println(newValue);
                        this.lastValue = newValue;
                        onChange.accept(lastValue); // call the callback
                    } else { 
                        // Otherwise, reset the value to the last known value
                        entry.setDouble(this.lastValue);
                    }
                });
    }

    /** 
     * Constructor without onChange callback
     * @param key          The name of the tunable value
     * @param defaultValue The default value if not set on the dashboard
     */
    public TunableNumber(String key, double defaultValue) {
        this.lastValue = defaultValue;

        // Get or create the NetworkTable entry for this tunable number
        entry = NetworkTableInstance.getDefault()
                .getTable("SmartDashboard")
                .getEntry(key);

        // Set the initial value on the dashboard
        entry.setDouble(defaultValue);

        // Add a listener to detect changes in the entry
        inst.addListener(
                entry,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                    double newValue = entry.getDouble(lastValue);

                    // If the value changed and tuningMode is ON, accept the change
                    if(this.lastValue != newValue && Constants.tuningMode) {
                        DogLog.log(key, entry.getDouble(lastValue));
                        System.out.println(newValue);
                        this.lastValue = newValue;
                    } else { 
                        // Otherwise, reset the value to the last known value
                        entry.setDouble(this.lastValue);
                    }
                });
    }

    /* 
     * Return the current value of this tunable number
     * This always returns the last accepted value
     */
    public double get() {
        return lastValue;
    }
}