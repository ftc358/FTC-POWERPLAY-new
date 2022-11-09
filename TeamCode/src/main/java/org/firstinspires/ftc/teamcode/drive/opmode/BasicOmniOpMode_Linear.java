/* Copyright (c) 2021 FIRST. All rights reserved.
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
package org.firstinspires.ftc.teamcode.drive.opmode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware;

@TeleOp(name="mcnanum drive training", group="Linear Opmode")
public class BasicOmniOpMode_Linear extends LinearOpMode {
    Rware r = new Rware();
    private Object HardwareMap;
   /* private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    */
    /*
    private CRServo s1 = null;
    private CRServo s2 = null;
    private Servo s3 = null;
    private Servo s4 = null;
    private DcMotor liftmotorright = null;
    private DcMotor liftmotorleft = null;
    */


    @Override
    public void runOpMode() {
        r.init((com.qualcomm.robotcore.hardware.HardwareMap) HardwareMap);
        /*
        s1  = hardwareMap.get(CRServo.class, "s1");
        s2  = hardwareMap.get(CRServo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        s4 = hardwareMap.get(Servo.class, "s4");
        */


        /*
        liftmotorleft = hardwareMap.get(DcMotor.class, "m1");
        liftmotorright = hardwareMap.get(DcMotor.class, "m2");

         */

        /*
        liftmotorleft.setDirection(DcMotor.Direction.REVERSE);
        s2.setDirection(DcMotorSimple.Direction.REVERSE);
        s3.setPosition(0.4);
        s4.setPosition(0.1);

         */
        boolean clamp = false;
        boolean clamplast = false;
        boolean toggle = false;
        boolean intake = false;
        boolean intakelast = false;
        boolean toggle1 = false;
        waitForStart();
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            /*
            clamplast = clamp;
            clamp = gamepad1.right_bumper;
            if (clamplast == false && clamp == true){
                toggle = !toggle;
            }
            if(toggle){
                s3.setPosition(0.3);
                s4.setPosition(0.2);
            }else{
                s3.setPosition(0.4);
                s4.setPosition(0.1);
            }
            if(gamepad1.left_bumper){
                liftmotorleft.setPower(0.5);
                liftmotorright.setPower(0.5);
            }else if(gamepad1.left_trigger == 1){
                liftmotorleft.setPower(-0.2);
                liftmotorright.setPower(-0.2);
            }else {
                liftmotorleft.setPower(0);
                liftmotorright.setPower(0);
            }
            intakelast = intake;
            intake = gamepad1.a;
            if (intakelast == false && intake == true){
                toggle1 = !toggle1;
            }
            if(toggle1){
                s1.setPower(1);
                s2.setPower(1);
            }else{
                s1.setPower(0);
                s2.setPower(0);
            }
            */

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            r.leftFrontDrive.setPower(-frontLeftPower);
            r.leftBackDrive.setPower(-backLeftPower);
            r.rightFrontDrive.setPower(-frontRightPower);
            r.rightBackDrive.setPower(-backRightPower);
        }
    }
}
