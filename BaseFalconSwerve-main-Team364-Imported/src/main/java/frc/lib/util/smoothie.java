package frc.lib.util;
import java.util.*; 

public class smoothie { 

    private final Queue<Double> Dataset = new LinkedList<Double>(); 

    private final int prd; 

    private double s;

    public smoothie(int prd) 

    { 

       this.prd = prd; 

    }
    
public void addData(double n) 

{ 

    s += n; 

    Dataset.add(n); 



    if (Dataset.size() > prd) 

    { 

        s -= Dataset.remove(); 

    } 

} 

public double getMean() 

{ 

    return s / prd; 

} 

/*public static void main(String[] args) 

{ 

    double[] data = {1, 3, 5, 6, 8, 12, 18, 21, 22, 25}; 

    int prd = 3; 

    smoothie ob = new smoothie(prd); 

    for (double i : data) { 

        ob.addData(i); 

        System.out.println("Number added is " + 

                            i + ", SMA = " + ob.getMean()); 

    } 

} */

}