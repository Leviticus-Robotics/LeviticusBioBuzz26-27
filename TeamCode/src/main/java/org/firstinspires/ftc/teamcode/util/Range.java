package org.firstinspires.ftc.teamcode.util;

public class Range {
    private final double low;
    private final double high;

    public Range(double low, double high) {
        if (!Double.isFinite(low) || !Double.isFinite(high)) {
            throw new IllegalArgumentException("range bounds must be finite");
        }
        if (low == high) throw new IllegalArgumentException("range must have non-zero width");
        this.low = Math.min(low, high);
        this.high = Math.max(low, high);
    }

    public double low() { return low; }
    public double high() { return high; }

    public double getValueRatio(double value) {
        return (value - low) / (high - low);
    }

    public double getValueFromRatio(double ratio) {
        return low + (high - low) * ratio;
    }

    public double toRange(double value, Range newRange) {
        if (newRange == null) throw new IllegalArgumentException("target range is null");
        double ratio = this.getValueRatio(value);
        return newRange.getValueFromRatio(ratio);
    }

    public boolean contains(double value) {
        return value >= low && value <= high;
    }

    public double clip(double value) {
        return Math.max(Math.min(value, high), low);
    }

    public double range() {
        return high - low;
    }
}
