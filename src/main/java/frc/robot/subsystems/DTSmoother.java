package frc.robot.subsystems;

public class DTSmoother {


    double prevActual = 0;
    double threshold, multiplierConst, changeConst;
    public DTSmoother(double theshold, double multiplierConst, double changeConst){

        this.threshold = theshold;
        this.multiplierConst = multiplierConst;
        this.changeConst = changeConst;
    }



    public double Smoother(double expected){

        double actual = prevActual;
        double changedVal = 0;
        if (Math.abs(expected) - Math.abs(actual) < threshold){
            changedVal = expected;
        }

        double distanceMultiplier = multiplierConst/(Math.abs(expected - actual));

        double change = Math.pow(changeConst, distanceMultiplier) - 1;

        if (expected > actual){
            changedVal =  actual + change;
        } else if (expected < actual) {
            changedVal =  actual - change;
        }
        prevActual = changedVal;
        return changedVal;

    }


}
