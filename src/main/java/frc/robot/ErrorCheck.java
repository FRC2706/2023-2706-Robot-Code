// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.ErrorCode;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;

public class ErrorCheck {
    public static int count = 1;
    public static boolean errREV(REVLibError error) {
        if (error == REVLibError.kOk) {
            return true;
        }

        DriverStation.reportError("REV Device Error: " + error.toString(), true);

        return false;
    }

    public static boolean errCTRE(ErrorCode error) {
        if (error == ErrorCode.OK) {
            return true;
        }

        DriverStation.reportError("CTRE Device Error: " + error.toString(), true);

        return false;
    }
}
