package org.sbs.bears.coyote.servopkg;

import java.util.*;
/**
 * Created by Paul Lutus <lutusp@arachnoid.com> and Ian Clarke <ian.clarke@gmail.com> a long time ago in a galaxy far far away (not really).
 * Modified by Marc D Nichitiu on Jan 28 to add derivative and max/min functions. 
 */
public class Polynomial extends ArrayList<Double> {
	private static final long serialVersionUID = 1692843494322684190L;

	public Polynomial()
	{
		super();
	}
	public Polynomial(final int p) {
		super(p);
		for(int i = 0; i < p; i++)
		{
			this.add(0.0); // create zero polynomial
		}
	}

	public double getY(final double x) {
		double ret = 0;
		for (int p=0; p<size(); p++) {
			ret += get(p)*(Math.pow(x, p));
		}
		return ret;
	}

	public double getMax(double lowerRangeLimit, double upperRangeLimit)
	{
		ArrayList<Double> zeroes = this.takeDerivative().getZeroes(lowerRangeLimit,upperRangeLimit);
		double maxSoln = lowerRangeLimit -1;
		double maxValue = -Double.MAX_VALUE;
		for (double zero : zeroes)
		{
			double result = this.getY(zero);
			if(result > maxValue)
			{
				maxSoln = zero;
				maxValue = result;
			}
		}
		return maxSoln;
	}
	public ArrayList<Double> getZeroes(double lowerRangeLimit, double upperRangeLimit)
	{
		ArrayList<Double> ps = findPossibleZeros(lowerRangeLimit,upperRangeLimit);
		ArrayList<Double> zeroes = new ArrayList<>();
        for(Double i : ps) {
            //System.out.println("Trying x = " + i);
            //System.out.println("Y value: " + this.getY(i));
            Double solution = halley(i);
            if(solution.isNaN() || solution.isInfinite()) {
                continue;
            }
            else {
                if(Math.abs(this.getY(solution)) < .1) { 
                    zeroes.add(solution);
                }
            }
        }
        return zeroes;

	}

	public void setCoeff(final int power, final double newCoeff)
	{
		try {this.set(power,newCoeff);}
		catch(Exception e)
		{
			this.add(newCoeff);
		}
	}
	public double getCoeff(final int power)
	{
		return this.get(power);
	}
	public Polynomial takeDerivative()
	{
		int power = this.size() - 1; // zero is counted in size
		//System.out.println("power: " + power);
		Polynomial derivd = new Polynomial(power-1);
		for(int i = 1; i <= power; i++)
		{
			derivd.setCoeff(i-1,this.getCoeff(i)*i);
		}
		return derivd;
	}


	public ArrayList<Double> findPossibleZeros(double minX,double maxX) {
        ArrayList<Double> possibleSolutions = new ArrayList<Double>();
        double a = minX;
        while(a <= maxX) {
            double sol1 = Math.abs(this.getY(a));
            if(sol1 <= 20) {
                possibleSolutions.add(sol1); 
            }
            a += .1;
        }
        return possibleSolutions;
    }

    public double halley(double guess) {
        Polynomial dp = this.takeDerivative();
        Polynomial d2p = dp.takeDerivative();
        double valP = this.getY(guess);
        while(Math.abs(valP) > 0.01) {
            double valDP = dp.getY(guess);
            double valD2P = d2p.getY(guess);
            guess = guess - (2*valP*valDP)/(2*valD2P*valD2P-valP*valD2P);
            valP = this.getY(guess);
        }
        return guess;
    }

	@Override
	public String toString() {
		final StringBuilder ret = new StringBuilder();
		for (int x = size() - 1; x > -1; x--) {
			ret.append(get(x) + (x > 0 ? "x^" + x + " + " : ""));
		}
		return ret.toString();
	}
}