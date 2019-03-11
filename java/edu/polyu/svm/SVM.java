package edu.polyu.svm;

import java.io.InputStream;



public interface SVM {
	public void readSVM(String weightFile);
	public void readSVM(InputStream is);
	public void loadSVM(String line); 
	public double compScore(double[] x);
}
