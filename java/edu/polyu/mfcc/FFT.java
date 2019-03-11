/*
OC Volume - Java Speech Recognition Engine
Copyright (c) 2002-2004, OrangeCow organization
All rights reserved.

Redistribution and use in source and binary forms,
with or without modification, are permitted provided
that the following conditions are met:

 * Redistributions of source code must retain the
above copyright notice, this list of conditions
and the following disclaimer.
 * Redistributions in binary form must reproduce the
above copyright notice, this list of conditions
and the following disclaimer in the documentation
and/or other materials provided with the
distribution.
 * Neither the name of the OrangeCow organization
nor the names of its contributors may be used to
endorse or promote products derived from this
software without specific prior written
permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS
AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

Contact information:
Please visit http://ocvolume.sourceforge.net.
 */
//package org.ioe.tprsa.audio.feature;

package edu.polyu.mfcc;


public class FFT {

	private int numPoints;
	private float real[];
	private float imag[];

	public FFT(int numPoints) {
		this.numPoints = numPoints;
		real = new float[numPoints];
		imag = new float[numPoints];		
	}
		
	public void computeFFT(float signal[]) {
		if (signal.length != this.numPoints) {
			System.err.println("Error in FFT: signal length mismatch.");
		}
		// Move the N point signal into the real part of the complex DFT's time domain
		// Set all of the samples in the imaginary part to zero
		real = signal;
		for (int i = 0; i < imag.length; i++) {
			real[i] = signal[i];
			imag[i] = (float)0.0;
		}
		// perform FFT using the real & imag array. Return results in real[] and imag[]
		doFFT();
	}

	public float[] getReal() {
		return real;
	}

	public float[] getImag() {
		return imag;
	}

	/**
	 * performs Fast Fourier Transformation<br>
	 */
	private void doFFT() {
		if (numPoints == 1) { return; }
		final double pi = Math.PI;
		final int numStages = (int) (Math.log(numPoints) / Math.log(2));
		int halfNumPoints = numPoints >> 1;
		int j = halfNumPoints;
		// FFT time domain decomposition carried out by "bit reversal sorting"
		// algorithm
		int k = 0;
		for (int i = 1; i < numPoints - 2; i++) {
			if (i < j) {
				// swap
				float tempReal = real[j];
				float tempImag = imag[j];
				real[j] = real[i];
				imag[j] = imag[i];
				real[i] = tempReal;
				imag[i] = tempImag;
			}
			k = halfNumPoints;
			while (k <= j) {
				j -= k;
				k >>= 1;
			}
			j += k;
		}

		// loop for each stage
		for (int stage = 1; stage <= numStages; stage++) {
			int LE = 1;
			for (int i = 0; i < stage; i++) {
				LE <<= 1;
			}
			int LE2 = LE >> 1;
			double UR = 1;
			double UI = 0;
			// calculate sine & cosine values
			double SR = Math.cos(pi / LE2);
			double SI = -Math.sin(pi / LE2);
			// loop for each sub DFT
			for (int subDFT = 1; subDFT <= LE2; subDFT++) {
				// loop for each butterfly
				for (int butterfly = subDFT - 1; butterfly <= numPoints - 1; butterfly += LE) {
					int ip = butterfly + LE2;
					// butterfly calculation
					float tempReal = (float) (real[ip] * UR - imag[ip] * UI);
					float tempImag = (float) (real[ip] * UI + imag[ip] * UR);
					real[ip] = real[butterfly] - tempReal;
					imag[ip] = imag[butterfly] - tempImag;
					real[butterfly] += tempReal;
					imag[butterfly] += tempImag;
				}

				double tempUR = UR;
				UR = tempUR * SR - UI * SI;
				UI = tempUR * SI + UI * SR;
			}
		}
	}
}