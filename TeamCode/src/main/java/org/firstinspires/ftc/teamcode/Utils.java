package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Utils {

    public static class RampRate {
        Double m_lastValue = 0.0;
        Double m_maxRate;
        ElapsedTime m_timer;

        public RampRate(Double maxRate) {
            m_maxRate = maxRate;
            m_timer = new ElapsedTime();
        }

        public Double update(Double current) {

            Double maxValue = m_lastValue * (1 + m_maxRate);
            if (current > maxValue) {
                m_lastValue = maxValue;
                return maxValue;
            } else {
                m_lastValue = current;
                return current;
            }
        }
    }
    public static boolean isCloseEnough(Double actual, Double target, Double tolerance) {
        Double allowedDiff;

        if (target >= -1.0 && target <= 1.0) {
            allowedDiff = Math.abs(target * tolerance);
        } else {
            allowedDiff = 1.0;
        }
        Double error = Math.abs(target - actual);
        return (error <= allowedDiff);
    }

    public static boolean isCloseToZero(Double target, Double tolerance) {
        return (
                (tolerance * -1) <= target && target <= tolerance
                );
    }

    /**
     * Returns modulus of error where error is the difference between the reference
     * and a measurement.
     *
     * @param reference Reference input of a controller.
     * @param measurement The current measurement.
     * @param minimumInput The minimum value expected from the input.
     * @param maximumInput The maximum value expected from the input.
     */
    public static double getModulusError(double reference, double measurement, double minimumInput,
                                         double maximumInput) {
        double error = reference - measurement;
        double modulus = maximumInput - minimumInput;

        // Wrap error above maximum input
        int numMax = (int) ((error + maximumInput) / modulus);
        error -= numMax * modulus;

        // Wrap error below minimum input
        int numMin = (int) ((error + minimumInput) / modulus);
        error -= numMin * modulus;

        return error;
    }
}

