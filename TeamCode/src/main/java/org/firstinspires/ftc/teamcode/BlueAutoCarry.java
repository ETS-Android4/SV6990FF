package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class BlueAutoCarry extends LinearOpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, intake, outtake, delivery1, delivery2;
    private Servo duckRight, pully, duckleft;
    private DcMotorEx[] motors;
    private final double TPI = 33.5625;
    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

<<<<<<< HEAD
        drive(1, 300);
=======
        drive(1, 323);
>>>>>>> fa5fa4e (Almost done with blue carry - just need to mirror and get cycles done - this is gonna be a pain)

        sleep(1);

        raisePully(2000);

        deposit(500);

        raiseBasket(500);

<<<<<<< HEAD
        dropPully(930);
=======
        dropPully(950);
>>>>>>> fa5fa4e (Almost done with blue carry - just need to mirror and get cycles done - this is gonna be a pain)

        sleep(1);

        drive(-1, 110);

        sleep(1);

        strafe(-1, 1300);

<<<<<<< HEAD
        strafe(-.25, 1600);

        sleep(1);

        dropDuck(4100);      // Duck spinner
=======
        sleep(1);

        dropDuck(4000);      // Duck spinner
>>>>>>> fa5fa4e (Almost done with blue carry - just need to mirror and get cycles done - this is gonna be a pain)

        //heading to warehouse

        sleep(1);

<<<<<<< HEAD
        strafe(1, 300);

        sleep(50);

        turn(-1, 250);

        sleep(100);

        strafe(-1,400);

        sleep(100);

        strafe(1, 1430);

        sleep(1);

        drive(.5, 950);
=======
        strafe(1, 400);

        sleep(10);

        turn(-1, 365);

        sleep(1);

        strafe(1, 1590);

        sleep(1);

        drive(.5, 900);
>>>>>>> fa5fa4e (Almost done with blue carry - just need to mirror and get cycles done - this is gonna be a pain)

        sleep(1);

        drive(-1, 800);

<<<<<<< HEAD
        drive(-.5, 670);

        strafe(-1, 1200);

        drive(-.75, 825); //enter to warehouse

        sleep(50);

        strafe(-1, 750);

        sleep(50);

        intake.setPower(-1);
        delivery1.setPower(1);
        delivery2.setPower(-1);

        drive(-.5, 810); // start cycle

        drive(0, 100);

        drive(.5, 300);

        strafe(-1, 1200);

        drive(0, 50);

        strafe(1, 850);

        drive(0, 75);

        drive(1, 975);

        turn(1,250);
=======
        drive(-.5, 690);

        strafe(-1, 1330);

        drive(-1, 1000); //enter to warehouse

        sleep(100);

        strafe(-1, 800);

        sleep(100);

        intakeBox();

        drive(-.5, 800); // start cycle

        drive(0, 100);

        drive(.5, 600);

        drive(0, 100);

        strafe(1, 650);

        drive(0, 100);

        drive(.5, 1800);
>>>>>>> fa5fa4e (Almost done with blue carry - just need to mirror and get cycles done - this is gonna be a pain)

        raisePully(2000);

        deposit(500);

        raiseBasket(500);

        dropPully(950);

<<<<<<< HEAD
        turn(-1, 150);

        drive(-1, 1400);
=======
        drive(-1, 900);
>>>>>>> fa5fa4e (Almost done with blue carry - just need to mirror and get cycles done - this is gonna be a pain)

    }

    public void initialize() {
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("BR");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        delivery1 = (DcMotorEx) hardwareMap.dcMotor.get("delivery1");
        delivery2 = (DcMotorEx) hardwareMap.dcMotor.get("delivery2");
        outtake = (DcMotorEx) hardwareMap.dcMotor.get("outtake");
        duckRight = (Servo) hardwareMap.get("duckRight");
        pully = (Servo) hardwareMap.get("pully");
        duckleft = (Servo) hardwareMap.get("duckLeft");

        motors = new DcMotorEx[]{leftFront, leftBack, rightFront, rightBack};

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delivery1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delivery2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        waitForStart();
    }

    public void drive(double power, int time) throws InterruptedException {
        timer = new ElapsedTime();
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
        while (timer.milliseconds() <= time) {
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }
        }
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }

<<<<<<< HEAD
    public void strafe(double direction, int time) throws InterruptedException {
=======
    public void strafe(int direction, int time) throws InterruptedException {
>>>>>>> fa5fa4e (Almost done with blue carry - just need to mirror and get cycles done - this is gonna be a pain)
        timer = new ElapsedTime();
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx motor : motors) {
            motor.setPower(1 * direction);
        }
        while (timer.milliseconds() <= time) {
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }
        }
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void turn(int power, int time) throws InterruptedException {
        timer = new ElapsedTime();
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        for (DcMotorEx motor : motors) {
            motor.setPower(power);
        }
        while (timer.milliseconds() <= time) {
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }
        }
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
    }


    public void intakeBox() throws InterruptedException {
        timer = new ElapsedTime();

        intake.setPower(-1);
        delivery1.setPower(1);
        delivery2.setPower(-1);

            if (!opModeIsActive()) {
                throw new InterruptedException();
            }

        intake.setPower(0);
        delivery1.setPower(0);
        delivery2.setPower(0);

    }

    public void dropDuck(int time) throws InterruptedException {
<<<<<<< HEAD
        duckRight.setPosition(.60);
=======
        duckRight.setPosition(.65);
>>>>>>> fa5fa4e (Almost done with blue carry - just need to mirror and get cycles done - this is gonna be a pain)

        timer = new ElapsedTime();
        while (timer.milliseconds() <= time) {
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }
        }
        duckRight.setPosition(.5);
    }

    public void raisePully(int time) throws InterruptedException {
        timer = new ElapsedTime();
        outtake.setPower(1);

        while (timer.milliseconds() <= time)
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }
       /* timer = new ElapsedTime();
        pully.setPosition(0);
        while (timer.milliseconds() <= time) {
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }
*/

    }


    public void dropPully(int time) throws InterruptedException {

       /* timer = new ElapsedTime();
        pully.setPosition(1);
        while (timer.milliseconds() <= time)
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }
            */

        timer = new ElapsedTime();
        outtake.setPower(-1);
        while (timer.milliseconds() <= time) {
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }


        }
        outtake.setPower(0);
    }

    public void deposit(int time) throws InterruptedException {
        timer = new ElapsedTime();
        pully.setPosition(0);

        while (timer.milliseconds() <= time)
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }

    }

    public void raiseBasket(int time) throws InterruptedException {
        timer = new ElapsedTime();
        pully.setPosition(1);

        while (timer.milliseconds() <= time)
            if (!opModeIsActive()) {
                throw new InterruptedException();
            }
    }
}
