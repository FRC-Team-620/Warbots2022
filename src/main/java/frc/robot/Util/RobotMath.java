package frc.robot.Util;

import java.util.function.BiConsumer;

public abstract class RobotMath
{
	public static double linearMap(double n, double sourceMin, double sourceMax, double outputMin, double outputMax)
	{
		return (n - sourceMin) / (sourceMax - sourceMin) * (outputMax - outputMin) + outputMin;
	}
	
	public static double xKinkedMap(double n, double sourceMin, double sourceMax, double flatVal, double flatStart, double flatEnd, double outputMin, double outputMax)
	{
		if(n < flatStart)
			return linearMap(n, sourceMin, flatStart, outputMin, flatVal);
		else if(n > flatEnd)
			return linearMap(n, flatEnd, sourceMax, flatVal, outputMax);
		else
			return flatVal;
	}
	
	public static double yKinkedMap(double n, double sourceMin, double sourceMax, double jumpN, double jumpMin, double jumpMax, double outputMin, double outputMax)
	{
		if(n < jumpN)
			return linearMap(n, sourceMin, jumpN, outputMin, jumpMin);
		else if(n > jumpN)
			return linearMap(n, jumpN, sourceMax, jumpMax, outputMax);
		else
			return (jumpMin + jumpMax) / 2;
	}
	
	public static double curve(double n, double curve)
	{
		return Math.copySign(Math.pow(Math.abs(n), curve), n);
	}
	
	public static double constrain(double n, double lowerBound, double upperBound)
	{
		if(lowerBound > upperBound)
			return constrain(n, upperBound, lowerBound);
		
		if(n < lowerBound)
			return lowerBound;
		
		if(n > upperBound)
			return upperBound;
		
		return n;
	}
	
	public static boolean oneNonZero(double... numbers)
	{
		for (double n : numbers)
			if (n != 0)
				return true;
		
		return false;
	}
	
	public static <T> void linkNextAndPrevWithSelfReferencingCaps(T[] array, BiConsumer<T, T> setNext, BiConsumer<T, T> setPrev)
	{
		T first = array[0];
		setPrev.accept(first, first);
		
		for(int i = 1; i < array.length; ++i)
			setPrev.accept(array[i], array[i - 1]);
		for(int i = 0; i < array.length - 1; ++i)
			setNext.accept(array[i], array[i + 1]);
		
		T last = array[array.length - 1];
		setNext.accept(last, last);
	}
    public static double deadZone(double value, double deadZone, double endZone){
		return RobotMath.xKinkedMap(value, -1, 1, 0, -deadZone, deadZone, -1, 1);
	}
}