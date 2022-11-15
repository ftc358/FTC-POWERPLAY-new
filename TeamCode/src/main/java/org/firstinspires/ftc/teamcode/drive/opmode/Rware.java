/* Copyright (c) 2022 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Rware {
    Rware r = new Rware();
    /* Declare OpMode members. */

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;

    public CRServo s1 = null;
    public CRServo s2 = null;
    public Servo s3 = null;
    public Servo s4 = null;
    public DcMotor liftmotorright = null;
    public DcMotor liftmotorleft = null;
    public CRServo s5 = null;
    public CRServo s6 = null;
    public Servo s7 = null;
    public Servo s8 = null;
    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Rware() {

    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap HwMap) {

        s1  = HwMap.get(CRServo.class, "s1"); //intake
        s2  = HwMap.get(CRServo.class, "s2");
        s3 = HwMap.get(Servo.class, "s3"); //claw
        s4 = HwMap.get(Servo.class, "s4");
        s5 = HwMap.get(CRServo.class, "sl"); //claw neck
        s6 = HwMap.get(CRServo.class, "sr");
        s7 = HwMap.get(Servo.class, "s7"); //thing that goes in and out
        s8 = HwMap.get(Servo.class, "s8");


        leftFrontDrive = HwMap.get(DcMotor.class, "lf");
        leftBackDrive = HwMap.get(DcMotor.class, "lb");
        rightFrontDrive = HwMap.get(DcMotor.class, "rf");
        rightBackDrive = HwMap.get(DcMotor.class, "rb");

        liftmotorleft = HwMap.get(DcMotor.class, "m1");
        liftmotorright = HwMap.get(DcMotor.class, "m2");


        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftmotorleft.setDirection(DcMotor.Direction.REVERSE);
        s2.setDirection(DcMotorSimple.Direction.REVERSE);
        s3.setPosition(0.4);
        s4.setPosition(0.1);
        s7.setPosition(0);
        s8.setPosition(1);
        r.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }


}




