package org.firstinspires.ftc.teamcode;

/*   MIT License
 *   Copyright (c) [2024] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name= "StarterBotOpMode", group="Robot")
//@Disabled
public class Override extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor armMotor = null;
    public CRServo intake = null;
    public Servo wrist = null;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        armMotor = hardwareMap.get(DcMotor.class, "left_arm");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(CRServo.class, "intake");
        wrist = hardwareMap.get(Servo.class, "wrist");

        telemetry.addLine("Robot Ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double y = -Math.pow(gamepad1.left_stick_y, 3);
            double x = Math.pow(gamepad1.left_stick_x, 3);
            double rx = Math.pow(gamepad1.right_stick_x, 3);

            double frontLeft = y + x + rx;
            double frontRight = y - x - rx;

            double max = Math.max(Math.abs(frontLeft), Math.abs(frontRight));
            if (max > 1.0) {
                frontLeft /= max;
                frontRight /= max;
            }

            leftDrive.setPower(frontLeft);
            rightDrive.setPower(frontRight);

            if (gamepad1.a) intake.setPower(-1.0);
            else if (gamepad1.x) intake.setPower(0.0);
            else if (gamepad1.b) intake.setPower(0.5);
        }
    }
}
