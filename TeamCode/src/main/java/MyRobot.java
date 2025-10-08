package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.jumpypants.murphy.RobotContext;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;

import Subsystems.Wrist;

/**
 * MyRobot class that extends RobotContext to include robot-specific subsystems.
 */
public class MyRobot extends RobotContext {
    public final MecanumDrive drive;
    public final Wrist wrist;

    /**
     * Creates a new RobotContext with the specified telemetry and gamepad references.
     * All parameters are required and cannot be null.
     *
     * @param telemetry the telemetry instance for driver station communication
     * @param gamepad1  the primary gamepad controller
     * @param gamepad2  the secondary gamepad controller
     */
    public MyRobot(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, MecanumDrive drive, Wrist wrist) {
        super(telemetry, gamepad1, gamepad2);
        this.drive = drive;
        this.wrist = wrist;
    }
}