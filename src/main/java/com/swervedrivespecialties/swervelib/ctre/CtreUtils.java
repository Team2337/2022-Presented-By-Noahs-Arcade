package com.swervedrivespecialties.swervelib.ctre;

import com.ctre.phoenix.ErrorCode;

import edu.wpi.first.wpilibj.DriverStation;

public final class CtreUtils {
    /**
     * checks the specified error code for issues
     *
     * @param errorCode error code
     * @param message   message to print if error happens
     */
    public static void checkCtreError(ErrorCode errorCode, String message) {
        DriverStation.reportError(message + errorCode, false);
    }
}
