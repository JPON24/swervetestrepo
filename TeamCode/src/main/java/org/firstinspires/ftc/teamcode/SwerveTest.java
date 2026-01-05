package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class SwerveTest extends OpMode {
//    @Override
//    public void init() {
//        swerveDt.init(hardwareMap);
//    }
//
//    @Override
//    public void loop() {
//        double leftStickX = gamepad1.left_stick_x;
//        double leftStickY = gamepad1.left_stick_y;
//
//        double rightStickX = gamepad1.right_stick_x;
//        boolean reset = gamepad1.options;
//
//        swerveDt.swerveDrive(leftStickY, leftStickX, rightStickX, reset);
//    }

    final double L = 0.98;
    final double W = 1.00;

    DcMotorEx frontLeftMotor;
    AnalogInput frontLeftAnalog;
    CRServo frontLeftServo;

    DcMotorEx frontRightMotor;
    AnalogInput frontRightAnalog;
    CRServo frontRightServo;

    DcMotorEx backLeftMotor;
    AnalogInput backLeftAnalog;
    CRServo backLeftServo;

    DcMotorEx backRightMotor;
    AnalogInput backRightAnalog;
    CRServo backRightServo;

    PIDController flPID;
    PIDController frPID;

    PIDController rlPID;
    PIDController rrPID;

    ElapsedTime angleTimer = new ElapsedTime();

//    SparkFunOTOS otos;



    double FLkP = 0.002;
    double FLkI = 0.0;
    double FLkD = 0.0001;
    double FLkF = 0.0;

    double FRkP = 0.002;
    double FRkI = 0.0;
    double FRkD = 0.0001;
    double FRkF = 0.0;

    double RLkP = 0.002;
    double RLkI = 0.0;
    double RLkD = 0.0001;
    double RLkF = 0.0;

    double RRkP = 0.002;
    double RRkI = 0.0;
    double RRkD = 0.0001;
    double RRkF = 0.0;

    double inputAngle = 0;
    double turnSpeedDeg = 60;

    double FL_OFFSET = 176; // Test these offsets LMAO
    double FR_OFFSET = 162;
    double BL_OFFSET = 63;
    double BR_OFFSET = 48.6;


    final double GEARBOX_RATIO = 1 / (36.0f / 24.0f);

    double minServoPower = 0.03;


    double lastTargetFR = 0, lastTargetFL = 0, lastTargetRL = 0, lastTargetRR = 0;
    double lastAngleFR = 0, lastAngleFL = 0, lastAngleRL = 0, lastAngleRR = 0;

    double speed = 0.35;

    boolean wasFlippedFL, wasFlippedFR, wasFlippedBL, wasFlippedBR = false;
    double flSpeed, frSpeed, blSpeed, brSpeed = 0;

    ElapsedTime dx = new ElapsedTime();


    @Override
    public void init()
    {
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontLeftServo  = hardwareMap.get(CRServo.class,     "frontLeftServo");
        frontLeftAnalog = hardwareMap.get(AnalogInput.class, "frontLeftAnalog");

        frontRightMotor  = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        frontRightServo  = hardwareMap.get(CRServo.class,     "frontRightServo");
        frontRightAnalog = hardwareMap.get(AnalogInput.class, "frontRightAnalog");

        backLeftMotor  = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backLeftServo  = hardwareMap.get(CRServo.class,     "backLeftServo");
        backLeftAnalog = hardwareMap.get(AnalogInput.class, "backLeftAnalog");

        backRightMotor  = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        backRightServo  = hardwareMap.get(CRServo.class,     "backRightServo");
        backRightAnalog = hardwareMap.get(AnalogInput.class, "backRightAnalog");

        frontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);


//        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
//        otos.calibrateImu();

        flPID = new PIDController(FLkP, FLkI, FLkD, FLkF);
        frPID = new PIDController(FRkP, FRkI, FRkD, FRkF);
        rlPID = new PIDController(RLkP, RLkI, RLkD, RLkF);
        rrPID = new PIDController(RRkP, RRkI, RRkD, RRkF);
    }

    @Override
    public void loop()
    {
        double leftStickX = -gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y;

        double rightStickX = gamepad1.right_stick_x;
        boolean reset = gamepad1.options;

//        inputAngle += dx.seconds() * rightStickX * turnSpeedDeg;

        dx.reset();

        swerveDrive(leftStickY, leftStickX, rightStickX, reset);

//        telemetry.addData("servo angleL: ", getAngle(backLeftAnalog, BL_OFFSET));
//        telemetry.addData("servo angleR: ", getAngle(backRightAnalog, BR_OFFSET));
//        telemetry.addData("servo angle: ", getAngle(frontLeftAnalog));
//        telemetry.addData("servo angle: ", getAngle(frontLeftAnalog));

        telemetry.addData("servo FL:", getAngle(frontLeftAnalog, FL_OFFSET));
        telemetry.addData("servo FR :", getAngle(frontRightAnalog, FR_OFFSET));
        telemetry.addData("servo BL :", getAngle(backLeftAnalog, BL_OFFSET));
        telemetry.addData("servo BR :", getAngle(backRightAnalog, BR_OFFSET));

//        double rawFL = (frontLeftAnalog.getVoltage() / 3.3) * 360.0;
//        telemetry.addData("FL Raw Sensor", "%.1f°", rawFL);
//        telemetry.addData("FL Wheel Angle", "%.1f°", getAngle(frontLeftAnalog, FL_OFFSET));
        telemetry.addData("FL Target", "%.1f°", lastTargetFL);
        telemetry.addData("FR Target", "%.1f°", lastTargetFR);
        telemetry.addData("RL Target", "%.1f°", lastTargetRL);
        telemetry.addData("RR Target", "%.1f°", lastTargetRR);

        telemetry.addData("FR Flipped", wasFlippedFR);
        telemetry.addData("BR Flipped", wasFlippedBR);
        telemetry.addData("BL Flipped", wasFlippedBL);
        telemetry.addData("FL Flipped", wasFlippedFL);

        telemetry.addData("FL Speed", flSpeed);
        telemetry.addData("FR Speed", frSpeed);
        telemetry.addData("RL Speed", blSpeed);
        telemetry.addData("RR Speed", brSpeed);

        //        telemetry.addData("---", "---");

//        telemetry.addData("FR Wheel", "%.1f°", getAngle(frontRightAnalog, FR_OFFSET));
//        telemetry.addData("BL Wheel", "%.1f°", getAngle(backLeftAnalog, BL_OFFSET));
//        telemetry.addData("BR Wheel", "%.1f°", getAngle(backRightAnalog, BR_OFFSET));
//        telemetry.addData("FR Flipped", wasFlippedFR);
//        telemetry.addData("BR Flipped", wasFlippedBR);
//        telemetry.addData("BL Flipped", wasFlippedBL);



        telemetry.update();
    }

    public void swerveDrive(double y_cmd_field, double x_cmd_field, double turn_cmd, boolean reset) {
//        if (reset) {
//            otos.resetTracking();
//        }

//        SparkFunOTOS.Pose2D currentPose = otos.getPosition();
//        double heading_rad = Math.toRadians(currentPose.h);
        double heading_rad = Math.toRadians(0);


        double x_cmd = x_cmd_field * Math.cos(heading_rad) + y_cmd_field * Math.sin(heading_rad);
        double y_cmd = -x_cmd_field * Math.sin(heading_rad) + y_cmd_field * Math.cos(heading_rad);

        if (Math.hypot(x_cmd, y_cmd) < 0.05 && Math.abs(turn_cmd) < 0.05) {
            stopDrive();
            return;
        }

        double R = Math.sqrt(L*L + W*W);

        double y_fr = y_cmd - turn_cmd * L;
        double x_fr = x_cmd - turn_cmd * W;
        double y_fl = y_cmd - turn_cmd * L;
        double x_fl = x_cmd + turn_cmd * W;
        double y_rl = y_cmd + turn_cmd * L;
        double x_rl = x_cmd + turn_cmd * W;
        double y_rr = y_cmd + turn_cmd * L;
        double x_rr = x_cmd - turn_cmd * W;

        double speed_fr = Math.hypot(x_fr, y_fr);
        double speed_fl = Math.hypot(x_fl, y_fl);
        double speed_rl = Math.hypot(x_rl, y_rl);
        double speed_rr = Math.hypot(x_rr, y_rr);

        double angle_fr = Math.toDegrees(Math.atan2(x_fr, y_fr));
        double angle_fl = Math.toDegrees(Math.atan2(x_fl, y_fl));
        double angle_rl = Math.toDegrees(Math.atan2(x_rl, y_rl));
        double angle_rr = Math.toDegrees(Math.atan2(x_rr, y_rr));



        double max = Math.max(Math.max(speed_fr, speed_fl), Math.max(speed_rl, speed_rr));
        if (max > 1.0) {
            speed_fr /= max; speed_fl /= max; speed_rl /= max; speed_rr /= max;
        }

        double current_fr = getAngle(frontRightAnalog, FR_OFFSET);
        double current_fl = getAngle(frontLeftAnalog, FL_OFFSET);
        double current_rl = getAngle(backLeftAnalog, BL_OFFSET);
        double current_rr = getAngle(backRightAnalog, BR_OFFSET);

        current_fr = filter(lastAngleFR, current_fr);
        current_fl = filter(lastAngleFL, current_fl);
        current_rl = filter(lastAngleRL, current_rl);
        current_rr = filter(lastAngleRR, current_rr);

        lastAngleFR = current_fr;
        lastAngleFL = current_fl;
        lastAngleRL = current_rl;
        lastAngleRR = current_rr;


        double[] opt_fr, opt_fl, opt_rl, opt_rr;
        if (speed_fr < 0.05)
        {
            opt_fr = new double[]{lastTargetFR, 0.0, wasFlippedFR ? 1 : 0};
        }
        else
        {
            opt_fr = optimize(angle_fr, speed_fr, current_fr, wasFlippedFR);
        }

        if (speed_fl < 0.05)
        {
            opt_fl = new double[]{lastTargetFL, 0.0, wasFlippedFL ? 1 : 0};
        }
        else
        {
            opt_fl = optimize(angle_fl, speed_fl, current_fl, wasFlippedFL);
        }

        if (speed_rl < 0.05)
        {
            opt_rl = new double[]{lastTargetRL, 0.0, wasFlippedBL ? 1 : 0};
        }
        else
        {
            opt_rl = optimize(angle_rl, speed_rl, current_rl, wasFlippedBL);
        }
        if (speed_rr < 0.05)
        {
            opt_rr = new double[]{lastTargetRR, 0.0, wasFlippedBR ? 1 : 0};
        }
        else
        {
            opt_rr = optimize(angle_rr, speed_rr, current_rr, wasFlippedBR);
        }

        wasFlippedFR = opt_fr[2] > 0;
        wasFlippedFL = opt_fl[2] > 0;
        wasFlippedBL = opt_rl[2] > 0;
        wasFlippedBR = opt_rr[2] > 0;

        frSpeed = opt_fr[1] * speed;
        flSpeed = opt_fl[1] * speed;
        blSpeed = opt_rl[1] * speed;
        brSpeed = opt_rr[1] * speed;

        frontRightMotor.setPower(frSpeed);
        frontLeftMotor.setPower(flSpeed);
        backLeftMotor.setPower(blSpeed);
        backRightMotor.setPower(brSpeed);

//        lastTargetFR = opt_fr[0];
//        lastTargetFL = opt_fl[0];
//        lastTargetRL = opt_rl[0];
//        lastTargetRR = opt_rr[0];

        runPID(opt_fl[0], opt_fr[0], opt_rl[0], opt_rr[0]);
    }

    private void stopDrive() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        runPID(lastTargetFL,lastTargetFR,lastTargetRL,lastTargetRR);
    }

    private void runPID(double tFL ,double tFR, double tRL, double tRR) {
        double dt = angleTimer.seconds();
        if (dt < 0.001) dt = 0.001;

        double servoTargetFL = tFL;
        double servoTargetFR = tFR;
        double servoTargetRL = tRL;
        double servoTargetRR = tRR;

        double currentServoAngleFL = getAngle(frontLeftAnalog, FL_OFFSET);
        double currentServoAngleFR = getAngle(frontRightAnalog, FR_OFFSET);
        double currentServoAngleRL = getAngle(backLeftAnalog, BL_OFFSET);
        double currentServoAngleRR = getAngle(backRightAnalog, BR_OFFSET);

        double velFL = normalizeAngle(servoTargetFL - lastTargetFL) * dt;
        double velFR = normalizeAngle(servoTargetFR - lastTargetFR) * dt;
        double velRL = normalizeAngle(servoTargetRL - lastTargetRL) * dt;
        double velRR = normalizeAngle(servoTargetRR - lastTargetRR) * dt;

        double powerFL = flPID.calculate(servoTargetFL, currentServoAngleFL, velFL);
        double powerFR = frPID.calculate(servoTargetFR, currentServoAngleFR, velFR);
        double powerRL = rlPID.calculate(servoTargetRL, currentServoAngleRL, velRL);
        double powerRR = rrPID.calculate(servoTargetRR, currentServoAngleRR, velRR);

        angleTimer.reset();

//        powerFL = 0.15 * gamepad1.left_stick_x;

        frontLeftServo.setPower(powerFL);
        frontRightServo.setPower(powerFR);
        backLeftServo.setPower(powerRL);
        backRightServo.setPower(powerRR);


        lastTargetFL = tFL;
        lastTargetFR = tFR;
        lastTargetRL = tRL;
        lastTargetRR = tRR;
    }

    private double getAngle(AnalogInput sensor, double offset) {
        double rawAngle = (sensor.getVoltage() / 3.3) * 360.0;

        double adjustedAngle = (rawAngle/GEARBOX_RATIO) - offset; // Coordinate missmatch between servo and wheel space!

        return normalizeAngle(adjustedAngle);
    }

    double alpha = 0.8;

    private double filter(double prev, double angle)
    {
        return (angle * alpha + prev * (1-alpha));
    }

    private double normalizeAngle(double angle) {

        angle = (angle + 180.0) % 360.0;
        if (angle < 0) angle += 360.0;
        return angle - 180.0;
    }

    double FLIP_CENTER = 90;
    double FLIP_BAND = 15;

    private double[] optimize(
            double target,
            double speed,
            double current,
            boolean wasFlipped
    ) {
        double error = normalizeAngle(target - current);
        double absError = Math.abs(error);

        // hysteresis around 90 degrees
        if (!wasFlipped) {
            if (absError > FLIP_CENTER + FLIP_BAND) wasFlipped = true;
        } else {
            if (absError < FLIP_CENTER - FLIP_BAND) wasFlipped = false;
        }

        if (wasFlipped) {
            target = normalizeAngle(target + 180.0);
            speed *= -1;
        }

        return new double[]{target, speed, wasFlipped ? 1 : 0};
    }


    public class PIDController {
        private double kP, kI, kD, kF;
        private double lastError = 0;
        private ElapsedTime timer = new ElapsedTime();

        public PIDController(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            timer.reset();
        }

        public double calculate(double target, double current, double targetVel) {
            double error = normalizeAngle(target - current);
            double dt = timer.seconds();

            if (dt < 0.001) dt = 0.001;

            double pTerm = error * kP;
            double derivative = (error - lastError) / dt;
            double dTerm = derivative * kD;

            // PLEASE SPEED I NEED THIS!!!
            // MY FEED IS KINDA FORWARDLESS!!!
//            double fTerm = targetVel * kF;

            lastError = error;
            timer.reset();

            double output = pTerm + dTerm;

            if (Math.abs(output) < 1e-6) output = 0;
            else output += Math.signum(output) * minServoPower;

            return Range.clip(output, -1, 1);
        }
    }
}