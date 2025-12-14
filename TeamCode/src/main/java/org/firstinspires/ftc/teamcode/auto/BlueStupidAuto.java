package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.Cryptic.Robot;

@Autonomous(name = "BlueStupidAuto")
public class BlueStupidAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot robot = new Robot();
        robot.initialize(this);

        DcMotorEx bandMotor = hardwareMap.get(DcMotorEx.class, "bandMotor");
        bandMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        Servo indexServo = hardwareMap.get(Servo.class, "indexServo");
        Servo transferServo = hardwareMap.get(Servo.class, "transferServo");
        Servo angleServo = hardwareMap.get(Servo.class, "angleServo");

        DcMotorEx powerMotor = hardwareMap.get(DcMotorEx.class, "powerMotor");

        double ANGLE_ONE = 0.8;

        double SERVO_BOTTOM = 0.5;
        double SERVO_TOP = 0.25;

        int DRIVE_TIME_MS = 1200;

        int INDEX_SETTLE_MS = 1000;     // give index servo time to actually move
        int FLYWHEEL_SPINUP_MS = 5000;  // time to reach speed before feeding
        int TRANSFER_UP_MS = 1000;      // how long transfer stays up to feed/shoot
        int TRANSFER_DOWN_MS = 1000;    // time to retract + let ball clear

        double OUTTAKE1 = 0.01;
        double OUTTAKE2 = 0.38;
        double OUTTAKE3 = 0.75;

        waitForStart();
        if (isStopRequested()) return;

        indexServo.setPosition(OUTTAKE1);
        transferServo.setPosition(SERVO_BOTTOM);
        angleServo.setPosition(ANGLE_ONE);

        sleep(200);

        robot.dt.drivebase.setDrivePowers(new PoseVelocity2d(
                new Vector2d(-0.6, 0),  // keep your sign; flip if wrong direction
                0
        ));
        sleep(DRIVE_TIME_MS);
        robot.dt.drivebase.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        sleep(150);

        powerMotor.setPower(-0.6);
        sleep(FLYWHEEL_SPINUP_MS);

        sleep(INDEX_SETTLE_MS);

        transferServo.setPosition(SERVO_TOP);
        sleep(TRANSFER_UP_MS);

        transferServo.setPosition(SERVO_BOTTOM);
        bandMotor.setVelocity(725.0);
        sleep(TRANSFER_DOWN_MS);
        indexServo.setPosition(OUTTAKE2);
        sleep(INDEX_SETTLE_MS);

        transferServo.setPosition(SERVO_TOP);
        sleep(TRANSFER_UP_MS);

        transferServo.setPosition(SERVO_BOTTOM);
        sleep(TRANSFER_DOWN_MS);

        indexServo.setPosition(OUTTAKE3);
        sleep(INDEX_SETTLE_MS);

        transferServo.setPosition(SERVO_TOP);
        sleep(TRANSFER_UP_MS);

        transferServo.setPosition(SERVO_BOTTOM);
        sleep(TRANSFER_DOWN_MS);
        bandMotor.setVelocity(0.0);
        powerMotor.setPower(0.0);
        sleep(100);

        robot.dt.drivebase.setDrivePowers(new PoseVelocity2d(
                new Vector2d(0, 0.6),
                0
        ));
        sleep(1000);
        robot.dt.drivebase.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0)); // STOP
    }
}

