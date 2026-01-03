package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class SwerveTest extends OpMode {

    // Module order indices
    private static final int FL = 0, FR = 1, BL = 2, BR = 3;

    // Set these after a quick calibration:
// driveInvert: +1 means "positive motor power drives robot forward" when module is pointed forward
    private final double[] driveInvert = new double[]{ +1, +1, +1, +1 };

    // steerInvert: +1 means "positive servo power increases measured angle"
    private final double[] steerInvert = new double[]{ +1, +1, +1, +1 };

    // Analog angle unwrap + filter (servo degrees)
    private final double[] servoAngleUnwrapped = new double[]{0,0,0,0};
    private final double[] servoAngleFiltered  = new double[]{0,0,0,0};
    private final boolean[] angleInit          = new boolean[]{false,false,false,false};

    private double angleAlpha = 0.25; // EMA smoothing (0..1). Start ~0.2-0.35

    private final boolean[] isFlipped = new boolean[]{false,false,false,false};
    private double flipEnter = 100.0; // enter flip mode
    private double flipExit  = 80.0;  // exit flip mode


    // Geometry (tune these to your robot’s wheelbase/trackwidth in consistent units)
    final double L = 0.98;
    final double W = 1.00;

    // Hardware
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

    // PID per module (use separate instances!)
    PIDController flPID;
    PIDController frPID;
    PIDController rlPID;
    PIDController rrPID;

    // Timing
    ElapsedTime angleTimer = new ElapsedTime();
    ElapsedTime loopTimer = new ElapsedTime();

    // Gains (start conservative, tune per module)
    double FLkP = 0.003, FLkI = 0.0, FLkD = 0.0, FLkF = 0.0;
    double FRkP = 0.005, FRkI = 0.0, FRkD = 0.0, FRkF = 0.0;
    double RLkP = 0.005, RLkI = 0.0, RLkD = 0.0, RLkF = 0.0;
    double RRkP = 0.005, RRkI = 0.0, RRkD = 0.0, RRkF = 0.0;

    // Offsets (degrees). These should be the “zero” direction you define per module.
    double FL_OFFSET = 0.0;
    double FR_OFFSET = -10.00;
    double BL_OFFSET = 42.2;
    double BR_OFFSET = 20.00;

    // Steering gear ratio: module angle = servo angle * GEARBOX_RATIO
    // You had: 1/(36/24) = 24/36 = 0.666..., i.e. servo turns more than module.
    // Keep this consistent across all conversions.
    final double GEARBOX_RATIO = 1.0 / (36.0 / 24.0); // 0.666666...

    // Servo helpers
    double minServoPower = 0.02;     // overcome static friction
    double deadbandDeg = 8;        // within this error, stop steering

    // Drive scaling
    double driveScale = 0.25;        // overall speed limit for testing

    // Last targets (module angles, in MODULE degrees, not servo degrees)
    double lastTargetFL = 0, lastTargetFR = 0, lastTargetRL = 0, lastTargetRR = 0;

    @Override
    public void init() {
        frontLeftMotor  = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontLeftServo  = hardwareMap.get(CRServo.class, "frontLeftServo");
        frontLeftAnalog = hardwareMap.get(AnalogInput.class, "frontLeftAnalog");

        frontRightMotor  = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        frontRightServo  = hardwareMap.get(CRServo.class, "frontRightServo");
        frontRightAnalog = hardwareMap.get(AnalogInput.class, "frontRightAnalog");

        backLeftMotor  = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backLeftServo  = hardwareMap.get(CRServo.class, "backLeftServo");
        backLeftAnalog = hardwareMap.get(AnalogInput.class, "backLeftAnalog");

        backRightMotor  = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        backRightServo  = hardwareMap.get(CRServo.class, "backRightServo");
        backRightAnalog = hardwareMap.get(AnalogInput.class, "backRightAnalog");

        flPID = new PIDController(FLkP, FLkI, FLkD, FLkF);
        frPID = new PIDController(FRkP, FRkI, FRkD, FRkF);
        rlPID = new PIDController(RLkP, RLkI, RLkD, RLkF);
        rrPID = new PIDController(RRkP, RRkI, RRkD, RRkF);

        angleTimer.reset();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double leftStickX = gamepad1.left_stick_x;   // strafe
        double leftStickY = gamepad1.left_stick_y;   // forward
        double rightStickX = gamepad1.right_stick_x; // rotate
        boolean reset = gamepad1.options;            // reserved

        // Run drive
        swerveDrive(leftStickY, leftStickX, rightStickX, reset);

        // Telemetry: module angles (MODULE degrees)
//        telemetry.addData("FL angle", normalizeAngle(getModuleAngleDeg(frontLeftAnalog, FL_OFFSET)));
//        telemetry.addData("FR angle", normalizeAngle(getModuleAngleDeg(frontRightAnalog, FR_OFFSET)));
//        telemetry.addData("BL angle", normalizeAngle(getModuleAngleDeg(backLeftAnalog, BL_OFFSET)));
//        telemetry.addData("BR angle", normalizeAngle(getModuleAngleDeg(backRightAnalog, BR_OFFSET)));

        telemetry.addData("Last tgt FL", lastTargetFL);
        telemetry.addData("Last tgt FR", lastTargetFR);
        telemetry.addData("Last tgt BL", lastTargetRL);
        telemetry.addData("Last tgt BR", lastTargetRR);

        telemetry.update();
    }

    /**
     * Field-centric stub currently uses heading 0.
     * y_cmd_field: forward (+), x_cmd_field: strafe (+), turn_cmd: CCW (+)
     */
    public void swerveDrive(double y_cmd_field, double x_cmd_field, double turn_cmd, boolean reset) {
        // Heading (rad). Replace with IMU/OTOS if desired.
        double heading_rad = Math.toRadians(0);

        // Field-centric transform
        double x_cmd = x_cmd_field * Math.cos(heading_rad) + y_cmd_field * Math.sin(heading_rad);
        double y_cmd = -x_cmd_field * Math.sin(heading_rad) + y_cmd_field * Math.cos(heading_rad);

        // Stop if commands are small
        if (Math.hypot(x_cmd, y_cmd) < 0.05 && Math.abs(turn_cmd) < 0.05) {
            stopDrive();
            return;
        }

        // Per-module vectors
        double y_fr = y_cmd - turn_cmd * L;
        double x_fr = x_cmd - turn_cmd * W;

        double y_fl = y_cmd - turn_cmd * L;
        double x_fl = x_cmd + turn_cmd * W;

        double y_rl = y_cmd + turn_cmd * L;
        double x_rl = x_cmd + turn_cmd * W;

        double y_rr = y_cmd + turn_cmd * L;
        double x_rr = x_cmd - turn_cmd * W;

        // Speeds
        double speed_fr = Math.hypot(x_fr, y_fr);
        double speed_fl = Math.hypot(x_fl, y_fl);
        double speed_rl = Math.hypot(x_rl, y_rl);
        double speed_rr = Math.hypot(x_rr, y_rr);

        // Desired module angles (MODULE degrees). atan2(x, y) matches your original convention.
        double angle_fr = Math.toDegrees(Math.atan2(x_fr, y_fr));
        double angle_fl = Math.toDegrees(Math.atan2(x_fl, y_fl));
        double angle_rl = Math.toDegrees(Math.atan2(x_rl, y_rl));
        double angle_rr = Math.toDegrees(Math.atan2(x_rr, y_rr));

        // Normalize drive speeds if any > 1
        double max = Math.max(Math.max(speed_fr, speed_fl), Math.max(speed_rl, speed_rr));
        if (max > 1.0) {
            speed_fr /= max;
            speed_fl /= max;
            speed_rl /= max;
            speed_rr /= max;
        }

        // Current module angles (MODULE degrees, normalized to [-180,180])
        // Current module angles (use FILTERED, continuous-ish)
        double current_fl = normalizeAngle(getFilteredModuleAngleDeg(FL, frontLeftAnalog,  FL_OFFSET));
        double current_fr = normalizeAngle(getFilteredModuleAngleDeg(FR, frontRightAnalog, FR_OFFSET));
        double current_bl = normalizeAngle(getFilteredModuleAngleDeg(BL, backLeftAnalog,   BL_OFFSET));
        double current_br = normalizeAngle(getFilteredModuleAngleDeg(BR, backRightAnalog,  BR_OFFSET));

// OPTIMIZE with hysteresis to prevent flip-chatter
        double[] opt_fl = optimizeHysteresis(FL, angle_fl, speed_fl, current_fl);
        double[] opt_fr = optimizeHysteresis(FR, angle_fr, speed_fr, current_fr);
        double[] opt_bl = optimizeHysteresis(BL, angle_rl, speed_rl, current_bl);
        double[] opt_br = optimizeHysteresis(BR, angle_rr, speed_rr, current_br);

// Drive motors with per-module inversion
        frontLeftMotor.setPower(opt_fl[1] * driveScale * driveInvert[FL]);
        frontRightMotor.setPower(opt_fr[1] * driveScale * driveInvert[FR]);
        backLeftMotor.setPower(opt_bl[1] * driveScale * driveInvert[BL]);
        backRightMotor.setPower(opt_br[1] * driveScale * driveInvert[BR]);

// Store last targets (MODULE degrees)
        lastTargetFL = opt_fl[0];
        lastTargetFR = opt_fr[0];
        lastTargetRL = opt_bl[0];
        lastTargetRR = opt_br[0];

// Run steering PID
        runPID(lastTargetFL, lastTargetFR, lastTargetRL, lastTargetRR);

    }

    private void stopDrive() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);

        // Hold last steering targets
        runPID(lastTargetFL, lastTargetFR, lastTargetRL, lastTargetRR);
    }

    /**
     * Steering control.
     * Targets are MODULE degrees. We convert to SERVO degrees for PID + analog comparison.
     */
    // Replace your ENTIRE runPID(...) with this version.
// It uses the new filtered/unwrapped helpers for angle feedback and correct indexing for inversion.

    private void runPID(double tFL_moduleDeg, double tFR_moduleDeg, double tBL_moduleDeg, double tBR_moduleDeg) {
        double dt = angleTimer.seconds();
        angleTimer.reset();
        if (dt < 0.001) dt = 0.001;

        // Convert MODULE target -> SERVO target (servo degrees)
        double tFL_servoDeg = moduleToServoDeg(normalizeAngle(tFL_moduleDeg));
        double tFR_servoDeg = moduleToServoDeg(normalizeAngle(tFR_moduleDeg));
        double tBL_servoDeg = moduleToServoDeg(normalizeAngle(tBL_moduleDeg));
        double tBR_servoDeg = moduleToServoDeg(normalizeAngle(tBR_moduleDeg));

        // Current SERVO angles (filtered + unwrapped) for feedback
        double curFL_servoDeg = normalizeAngle(updateFilteredUnwrappedServoDeg(FL, frontLeftAnalog,  FL_OFFSET));
        double curFR_servoDeg = normalizeAngle(updateFilteredUnwrappedServoDeg(FR, frontRightAnalog, FR_OFFSET));
        double curBL_servoDeg = normalizeAngle(updateFilteredUnwrappedServoDeg(BL, backLeftAnalog,   BL_OFFSET));
        double curBR_servoDeg = normalizeAngle(updateFilteredUnwrappedServoDeg(BR, backRightAnalog,  BR_OFFSET));

        // Last SERVO targets (computed from last MODULE targets) for velocity estimate
        double lastFL_servoDeg = moduleToServoDeg(normalizeAngle(lastTargetFL));
        double lastFR_servoDeg = moduleToServoDeg(normalizeAngle(lastTargetFR));
        double lastBL_servoDeg = moduleToServoDeg(normalizeAngle(lastTargetRL)); // your "RL" var is BL
        double lastBR_servoDeg = moduleToServoDeg(normalizeAngle(lastTargetRR)); // your "RR" var is BR

        // Target velocity estimate in SERVO deg/s
        double velFL = normalizeAngle(tFL_servoDeg - lastFL_servoDeg) / dt;
        double velFR = normalizeAngle(tFR_servoDeg - lastFR_servoDeg) / dt;
        double velBL = normalizeAngle(tBL_servoDeg - lastBL_servoDeg) / dt;
        double velBR = normalizeAngle(tBR_servoDeg - lastBR_servoDeg) / dt;

        // PID outputs (each module uses its own controller)
        double powerFL = flPID.calculate(tFL_servoDeg, curFL_servoDeg, velFL, deadbandDeg, minServoPower);
        double powerFR = frPID.calculate(tFR_servoDeg, curFR_servoDeg, velFR, deadbandDeg, minServoPower);
        double powerBL = rlPID.calculate(tBL_servoDeg, curBL_servoDeg, velBL, deadbandDeg, minServoPower);
        double powerBR = rrPID.calculate(tBR_servoDeg, curBR_servoDeg, velBR, deadbandDeg, minServoPower);

        // Apply steer inversion per module (critical for mirrored modules)
        frontLeftServo.setPower(powerFL * steerInvert[FL]);
        frontRightServo.setPower(powerFR * steerInvert[FR]);
        backLeftServo.setPower(powerBL * steerInvert[BL]);
        backRightServo.setPower(powerBR * steerInvert[BR]);

        // Update stored last targets (MODULE degrees)
        lastTargetFL = normalizeAngle(tFL_moduleDeg);
        lastTargetFR = normalizeAngle(tFR_moduleDeg);
        lastTargetRL = normalizeAngle(tBL_moduleDeg); // keep your variable name but treat as BL
        lastTargetRR = normalizeAngle(tBR_moduleDeg);
    }

    // fix fl gears

    /**
     * Reads analog sensor and returns SERVO angle in degrees in [-180, 180] after offset.
     */
    private double rawServoAngleDeg(AnalogInput sensor, double offsetDeg) {
        double rawDeg = (sensor.getVoltage() / 3.3) * 360.0;
        return normalizeAngle(rawDeg - offsetDeg); // wrapped [-180,180]
    }

    private double updateFilteredUnwrappedServoDeg(int idx, AnalogInput sensor, double offsetDeg) {
        double a = rawServoAngleDeg(sensor, offsetDeg);

        if (!angleInit[idx]) {
            angleInit[idx] = true;
            servoAngleUnwrapped[idx] = a;
            servoAngleFiltered[idx] = a;
            return servoAngleFiltered[idx];
        }

        // unwrap: keep continuity vs previous unwrapped
        double prev = servoAngleUnwrapped[idx];

        // compute smallest delta from previous wrapped value
        double prevWrapped = normalizeAngle(prev);
        double delta = normalizeAngle(a - prevWrapped);

        servoAngleUnwrapped[idx] = prev + delta;

        // EMA filter in unwrapped space
        servoAngleFiltered[idx] = servoAngleFiltered[idx] + angleAlpha * (servoAngleUnwrapped[idx] - servoAngleFiltered[idx]);
        return servoAngleFiltered[idx];
    }

    private double getFilteredModuleAngleDeg(int idx, AnalogInput sensor, double offsetDeg) {
        double servoDeg = updateFilteredUnwrappedServoDeg(idx, sensor, offsetDeg);
        return servoDeg * GEARBOX_RATIO; // module deg (continuous-ish)
    }

    private double moduleToServoDeg(double moduleDeg) { return moduleDeg / GEARBOX_RATIO; }

    private double normalizeAngle(double angleDeg) {
        while (angleDeg > 180.0) angleDeg -= 360.0;
        while (angleDeg < -180.0) angleDeg += 360.0;
        return angleDeg;
    }

    /**
     * Swerve optimization:
     * - Minimize steering delta (shortest path).
     * - If > 90 deg away, rotate target 180 deg and flip wheel speed.
     *
     * Returns [optimizedTargetDeg, optimizedSpeed]
     */
    private double[] optimizeHysteresis(int idx, double targetDeg, double speed, double currentDeg) {
        double delta = normalizeAngle(targetDeg - currentDeg);

        // hysteresis on flip decision
        if (!isFlipped[idx]) {
            if (Math.abs(delta) > flipEnter) isFlipped[idx] = true;
        } else {
            if (Math.abs(delta) < flipExit) isFlipped[idx] = false;
        }

        if (isFlipped[idx]) {
            targetDeg = normalizeAngle(targetDeg - Math.signum(delta) * 180.0);
            speed *= -1.0;
        }

        return new double[]{ normalizeAngle(targetDeg), speed };
    }


    public static class PIDController {
        private final double kP, kI, kD, kF;

        private double lastError = 0.0;
        private double integral = 0.0;
        private final ElapsedTime timer = new ElapsedTime();

        public PIDController(double kP, double kI, double kD, double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;
            timer.reset();
        }

        /**
         * PID on ANGLE error (degrees), with optional feedforward on target velocity (deg/s).
         * deadbandDeg: if abs(error) < deadbandDeg -> output 0.
         * minOut: adds a small bias to overcome friction once outside deadband.
         */
        public double calculate(double targetDeg, double currentDeg, double targetVelDegPerS,
                                double deadbandDeg, double minOut) {

            double error = normalizeAngleStatic(targetDeg - currentDeg);

            double dt = timer.seconds();
            timer.reset();
            if (dt < 0.001) dt = 0.001;

            // Integral with basic anti-windup clamp (tune as needed)
            integral += error * dt;
            integral = Range.clip(integral, -200.0, 200.0);

            double derivative = (error - lastError) / dt;
            lastError = error;

            double pTerm = kP * error;
            double iTerm = kI * integral;
            double dTerm = kD * derivative;
            double fTerm = kF * targetVelDegPerS;

            double out = pTerm + iTerm + dTerm + fTerm;

            if (Math.abs(error) < deadbandDeg) return 0.0;

            // Add minimum power to break static friction
            out += Math.signum(out) * minOut;

            return Range.clip(out, -1.0, 1.0);
        }

        private static double normalizeAngleStatic(double angleDeg) {
            while (angleDeg > 180.0) angleDeg -= 360.0;
            while (angleDeg < -180.0) angleDeg += 360.0;
            return angleDeg;
        }
    }
}
