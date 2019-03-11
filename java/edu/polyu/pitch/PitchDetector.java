package edu.polyu.pitch;

public interface PitchDetector {

	PitchDetectionResult getPitch(final float[] audioBuffer);
}
