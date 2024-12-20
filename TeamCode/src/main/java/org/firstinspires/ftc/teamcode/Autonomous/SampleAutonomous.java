package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "")
public class SampleAutonomous extends LinearOpMode {

    private static double DRIVE_SPEED = 0.4;
    private static double ROTATE_SPEED = 0.3;

    private DcMotor fl, fr, bl, br;

    public void stoprobot() {
        fl.setPower(0.0);
        fr.setPower(0.0);
        bl.setPower(0.0);
        br.setPower(0.0);
    }

    public void resetTicks() {
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void moveRobot(int flposition, int frposition, int blposition, int brposition, double flpower, double frpower, double blpower, double brpower) {

        fl.setTargetPosition(flposition);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setPower(flpower);

        fr.setTargetPosition(frposition);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setPower(frpower);

        bl.setTargetPosition(blposition);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setPower(blpower);

        br.setTargetPosition(brposition);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setPower(brpower);

        telemetry.addData("", fl.getCurrentPosition());
        telemetry.update();
    }



    @Override
    public void runOpMode() throws InterruptedException {

        fl = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        fl.resetDeviceConfigurationForOpMode();
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fr = hardwareMap.get(DcMotor.class, "frontRightMotor");
        fr.resetDeviceConfigurationForOpMode();
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bl = hardwareMap.get(DcMotor.class, "backLeftMotor");
        bl.resetDeviceConfigurationForOpMode();
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        br = hardwareMap.get(DcMotor.class, "backRightMotor");
        br.resetDeviceConfigurationForOpMode();
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        resetTicks();
        moveRobot(200, 200, 200, 200, -0.5, -0.5, -0.5, -0.5);
        sleep(5000);
        resetTicks();
        moveRobot(200, 200, 200, 200, 0.5, -0.5, 0.5, -0.5);
        sleep(5000);
        resetTicks();
        moveRobot(200, 200, 200, 200, 0.5, 0.5, 0.5, 0.5);
        sleep(5000);
        resetTicks();
        moveRobot(200, 200, 200, 200, -0.5, 0.5, -0.5, 0.5);
        sleep(5000);
        stoprobot();
    }
}