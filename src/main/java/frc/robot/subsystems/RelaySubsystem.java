// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Logger;

//import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;

/**
 * Vision Subsystem to handle any hardware on robot side that is needed
 * for vision. Such as:
 * 
 * - Relay for ring light
 * 
 */
public class RelaySubsystem extends SubsystemBase {

    // static variable single_instance for a Singleton class 
    private static RelaySubsystem single_instance = null;

    /** 
     * Get Instance of vision subsystem 
     */
    public static RelaySubsystem getInstance() {
        if (single_instance == null) 
            single_instance = new RelaySubsystem(); 
  
        return single_instance; 
    }

    /**
     * Set up all the instance variables related to hardware
     */
    private final Relay m_relay_ringlightRearSmall; 
    private final Relay m_relay_ringlightRearLarge;
    private final Relay m_relay_ringlightFront;

    /**
     * Construct the instance of this class
     */
    private RelaySubsystem() {
        /**
         * Set up hardware stuff
         */
        
        m_relay_ringlightRearSmall = new Relay(Config.RELAY_RINGLIGHT_REAR_SMALL); 
        m_relay_ringlightRearLarge = new Relay(Config.RELAY_RINGLIGHT_REAR_LARGE);
        m_relay_ringlightFront = new Relay(Config.RELAY_RINGLIGHT_FRONT);

        // Setup relays to be controlled from network tables
        addNetworkTableRelay(m_relay_ringlightRearSmall, "Ringlight_Rear_Small");
        addNetworkTableRelay(m_relay_ringlightRearLarge, "Ringlight_Rear_Large");
        addNetworkTableRelay(m_relay_ringlightFront, "Ringlight_Front");

    }

    /**
     * ---------
     * Section of code
     * 
     * Helper methods for this class
     * ---------
     */

    /**
     * Get relay object based on a port number
     */
    private Relay getRelay(int portNumber) {
        switch (portNumber) {
            case Config.RELAY_RINGLIGHT_REAR_SMALL:
                return m_relay_ringlightRearSmall;

            case Config.RELAY_RINGLIGHT_REAR_LARGE:
                return m_relay_ringlightRearLarge;

            case Config.RELAY_RINGLIGHT_FRONT:
                return m_relay_ringlightFront;

            default: {
                System.out.println(String.format("Relay at requested port number %d does not exist in code", portNumber));
                return null;
            }
        }
    }

    /**
     * Add a network table entry to change a relay
     */
    private void addNetworkTableRelay(Relay relay, String networkTableString) {
        // Get the relay table
        NetworkTable relayTable = NetworkTableInstance.getDefault().getTable(Config.RELAY_NETWORKTABLE);

        // Create the entry
        NetworkTableEntry relayEntry = relayTable.getEntry(networkTableString);

        // Force the type to be a boolean
       // relayEntry.forceSetBoolean(false);
        relayEntry.setBoolean(false);
        relayEntry.setPersistent();

        // Relay of by default
        relay.set(Value.kOff);
        
        // Setup the entry listener
        // relayTable.addEntryListener(networkTableString, (table, key, entry, value, flags) -> {
        //     boolean relayBol = (Boolean) value.getValue();
        //     Value relayValue;
        //     if (relayBol) 
        //         relayValue = Value.kOn;
        //     else 
        //         relayValue = Value.kOff;

        //     relay.set(relayValue);
        // }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate); 

    }

    /**
     * ---------
     * Section of code
     * 
     * Public methods for access to actions
     * ---------
     */

    /**
     * Toggle a replay specified by port in config
     * 
     * This method should only be used the desired result is to set both
     * outputs of the relay to on or both to off. Such as a ring light.
     * 
     * Use the values set in config for portNumbers
     */
    public void toggleRelay(int portNumber) {
        Relay relay = getRelay(portNumber);

        // Check if getRelay returned null
        if (relay == null) {
            System.out.println(String.format("Null relay on port number %d", portNumber));
            return;
        }

        // Check for value of relay and set it to the opposite one.
        Value value = relay.get();
        switch (value) {
            case kOn: 
                relay.set(Value.kOff);
                return;

            case kOff:
                relay.set(Value.kOn);
                return;
            
            // Handle cases for kForward & kReverse
            default:
                relay.set(Value.kOff);
                System.out.println(String.format(
                    "toggle relay method was used when the relay on port %d was had value %s which it should only be used for kOn or kOff",
                    portNumber, value.name()));
                return;
        }
    }

    /**
     * Set the value of the relay
     */
    public void setRelay(int portNumber, Value value) {
        Relay relay = getRelay(portNumber);
        relay.set(value);
    }
}
