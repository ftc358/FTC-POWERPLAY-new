/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Robot: Auto Drive By Time", group="Robot")

public class RobotAutoDriveByTime_Linear extends LinearOpMode {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private CRServo s1 = null;
    private CRServo s2 = null;
    private Servo s3 = null;
    private Servo s4 = null;
    private DcMotor liftmotorright = null;
    private DcMotor liftmotorleft = null;
    private Servo s5 = null;
    private Servo s6 = null;
    private Servo s7 = null;
    private Servo s8 = null;



    private ElapsedTime     runtime = new ElapsedTime();



    private void drive(int x, int y, int r){
        double yd = -0.4*y;
        double xd = 0.4*x * 1.1;
        double rd = 0.3*r;
        double denominator = Math.max(Math.abs(yd) + Math.abs(xd) + Math.abs(rd), 1);
        double frontLeftPower = (yd + xd + rd) / denominator;
        double backLeftPower = (yd - xd + rd) / denominator;
        double frontRightPower = (yd - xd - rd) / denominator;
        double backRightPower = (yd + xd - rd) / denominator;
        leftFrontDrive.setPower(-frontLeftPower);
        leftBackDrive.setPower(-backLeftPower);
        rightFrontDrive.setPower(-frontRightPower);
        rightBackDrive.setPower(-backRightPower);
    }
    @Override
    public void runOpMode() {

        s1 = hardwareMap.crservo.get("s1"); //intake
        s2 = hardwareMap.crservo.get("s2");
        s3 = hardwareMap.servo.get("s3"); //claw
        s4 = hardwareMap.servo.get("s4");
        s5 = hardwareMap.servo.get("sl"); //claw neck
        s6 = hardwareMap.servo.get("sr");
        s7 = hardwareMap.servo.get("s7"); //thing that goes in and out
        s8 = hardwareMap.servo.get("s8");


        leftFrontDrive = hardwareMap.dcMotor.get("lf");
        leftBackDrive = hardwareMap.dcMotor.get("lb");
        rightFrontDrive = hardwareMap.dcMotor.get("rf");
        rightBackDrive = hardwareMap.dcMotor.get("rb");

        liftmotorleft = hardwareMap.dcMotor.get("m1");
        liftmotorright = hardwareMap.dcMotor.get("m2");

        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotorleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotorright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotorleft.setDirection(DcMotor.Direction.REVERSE);
        s3.setPosition(0.4);
        s4.setPosition(0.1);
        s6.setPosition(0.4);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
            drive(1,0,0);
        }




        // Step 2:  Spin right for 1.3 seconds

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.3)) {
            telemetry.addData("Path", "Leg 2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 3:  Drive Backward for 1 Second

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1.0)) {
            telemetry.addData("Path", "Leg 3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
