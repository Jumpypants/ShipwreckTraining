package org.firstinspires.ftc.teamcode.subSystems;

import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MyRobot;


public class Intake {
    private final Servo rotatingWristServo;
    private final Servo verticalWristServo;
    public static final double ROTATING_CLOSE_POSITION = 0.7;
    public static final double WRIST_UP_POSITION = 0.1;
    public static final double WRIST_DOWN_POSITION = 1.0;

    public Intake(HardwareMap hardwareMap) {

        verticalWristServo = hardwareMap.get(Servo.class, "verticalWristServo");
        rotatingWristServo = hardwareMap.get(Servo.class, "rotatinglWristServo");
    }



    public class MoveWristTask extends Task {
        private final double rotatingPos;
        private final double verticalPos;
        public MoveWristTask(RobotContext robotContext, double rotatingPos, double verticalPos) {
            super(robotContext);
            this.rotatingPos = rotatingPos;
            this.verticalPos = verticalPos;
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            verticalWristServo.setPosition(verticalPos);
            rotatingWristServo.setPosition(rotatingPos);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            double currentPositionRotating = rotatingWristServo.getPosition();
            double currentPositionVertical = verticalWristServo.getPosition();
            double rotatingDifference = Math.abs(rotatingPos - currentPositionRotating);
            double verticalDifference = Math.abs(verticalPos - currentPositionVertical);

            return rotatingDifference < 5 && verticalDifference < 5;
        }
    }
}

