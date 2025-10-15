package org.firstinspires.ftc.teamcode.subSystems;

import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MyRobot;


public class Intake {
    private final Servo rotatingWristServo;
    private final Servo verticalWristServo1;
    private final Servo verticalWristServo2;
    public static final double ROTATING_CLOSE_POSITION = 0.7;
    public static final double WRIST_UP_POSITION = 0.1;
    public static final double WRIST_DOWN_POSITION = 1.0;

    public Intake(HardwareMap hardwareMap) {

        verticalWristServo1 = hardwareMap.get(Servo.class, "verticalWristServoUp");
        verticalWristServo2 = hardwareMap.get(Servo.class, "verticalWristServoDown");
        rotatingWristServo = hardwareMap.get(Servo.class, "rotatinglWristServo");
    }



    public class MoveWristTask extends Task {
        private final double rotatingPos;
        private final double estimatedTimeTaken;
        public MoveWristTask(RobotContext robotContext, double rotatingPos) {
            super(robotContext);
            this.rotatingPos = rotatingPos;
            double currentPosition = rotatingWristServo.getPosition();
            double TIME_COEFFICIENT = 0.5;
            estimatedTimeTaken = Math.abs(rotatingPos - currentPosition) * TIME_COEFFICIENT;
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            verticalWristServo1.setDirection(Servo.Direction.FORWARD);
            verticalWristServo2.setDirection(Servo.Direction.REVERSE);
            rotatingWristServo.setPosition(rotatingPos);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return ELAPSED_TIME.seconds() < estimatedTimeTaken;
        }
    }
}

