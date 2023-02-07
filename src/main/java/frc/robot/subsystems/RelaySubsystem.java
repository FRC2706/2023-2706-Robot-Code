// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubsystemChecker;
import frc.robot.SubsystemChecker.SubsystemType;
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
            SubsystemChecker.subsystemConstructed(SubsystemType.RelaySubsystem);
            single_instance = new RelaySubsystem(); 
  
        return single_instance; 
    }

    private final ArrayList<NTRelay> m_relays;

    /**
     * Construct the instance of this class
     */
    private RelaySubsystem() {
        /**
         * Set up hardware stuff
         */
        m_relays = new ArrayList<NTRelay>();
        
        // Setup relays to be controlled from network tables
        addNTRelay(Config.RELAY_RINGLIGHT_REAR_SMALL, "Ringlight_Rear_Small");
        addNTRelay(Config.RELAY_RINGLIGHT_REAR_LARGE, "Ringlight_Rear_Large");
        addNTRelay(Config.RELAY_RINGLIGHT_FRONT, "Ringlight_Front");

    }

    private class NTRelay {
        private int m_portNumber;
        private Relay m_relay;
        private BooleanEntry m_entryNT;
        boolean isOn;

        public NTRelay(int portNumber, String topicName) {
            m_portNumber = portNumber;
            m_relay = new Relay(portNumber);
            
            BooleanTopic topic = NetworkTableInstance.getDefault()
                .getTable(Config.RELAY_NETWORKTABLE)
                .getBooleanTopic(topicName);
            topic.setPersistent(true);

            
            m_entryNT = topic.getEntry(isOn);

            isOn = m_entryNT.get(false);
            setRelay(isOn);
        }

        public void checkNT() {
            boolean entryValue = m_entryNT.get();
            if (entryValue != isOn) {
                setRelay(entryValue);
            }
        }

        public void setRelay(boolean turnOn) {
            isOn = turnOn;
            if (isOn) {
                m_relay.set(Value.kOn);
            } else {
                m_relay.set(Value.kOff);
            }
            updateNT();
        }

        public void toggleRelay() {
            isOn = !isOn;
            setRelay(isOn);
        }

        private void updateNT() {
            m_entryNT.accept(isOn);
        }

        public int getPortNumber() {
            return m_portNumber;
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < m_relays.size(); i++) {
            m_relays.get(i).checkNT();
        }
    }

    /**
     * ---------
     * Section of code
     * 
     * Helper methods for this class
     * ---------
     */

    /**
     * Add a network table entry to change a relay
     */
    private void addNTRelay(int relayPortNumber, String networkTableString) {
        m_relays.add(new NTRelay(relayPortNumber, networkTableString));
     }

    /**
     * Get relay object based on a port number
     */
    private NTRelay getRelay(int portNumber) {
        for (int i = 0; i < m_relays.size(); i++) {
            if (m_relays.get(i).getPortNumber() == portNumber) {
                return m_relays.get(i);
            }
        }

        DriverStation.reportError(
            String.format("Relay at requested port number %d does not exist in code", portNumber),
            true);
        
        return null;
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
        NTRelay relay = getRelay(portNumber);

        // Check if getRelay returned null
        if (relay == null) {
            DriverStation.reportError(
                String.format("Null relay on port number %d", portNumber), 
                true);
            return;
        }

        relay.toggleRelay();
    }

    /**
     * Set the value of the relay
     */
    public void setRelay(int portNumber, boolean turnOn) {
        NTRelay relay = getRelay(portNumber);

        // Check if getRelay returned null
        if (relay == null) {
            DriverStation.reportError(
                String.format("Null relay on port number %d", portNumber), 
                true);
            return;
        }
        relay.setRelay(turnOn);
    }
}
