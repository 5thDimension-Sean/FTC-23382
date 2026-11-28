package org.firstinspires.ftc.teamcode.pedro.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

final class LimelightFrames {
    static final int PIPELINE = 0;
    private static final long MAX_AGE_MS = 100;

    private double timestamp;
    private long lastAdvanceNanos;
    private boolean clockAdvancing;

    LLResult freshResult(Limelight3A limelight) {
        LLResult result = limelight.getLatestResult();
        if (result == null || result.getPipelineIndex() != PIPELINE) {
            return null;
        }

        double nextTimestamp = result.getTimestamp();
        if (!Double.isFinite(nextTimestamp) || nextTimestamp <= 0) {
            return null;
        }

        // The SDK's staleness measures the HTTP poll, which can return a repeated frame.
        long now = System.nanoTime();
        if (nextTimestamp != timestamp) {
            clockAdvancing = nextTimestamp > timestamp && timestamp > 0;
            timestamp = nextTimestamp;
            lastAdvanceNanos = now;
        }

        long ageMs = result.getStaleness();
        return clockAdvancing && now - lastAdvanceNanos <= MAX_AGE_MS * 1_000_000
                && ageMs >= 0 && ageMs <= MAX_AGE_MS && result.isValid() ? result : null;
    }
}
