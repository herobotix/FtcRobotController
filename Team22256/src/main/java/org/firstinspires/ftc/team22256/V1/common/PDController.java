package org.firstinspires.ftc.team22256.common;

public class PDController {
    private double PCoefficient;
    private double Dcoefficient;


    public PDController(double PCoefficient, double DCoefficient){
    this.PCoefficient = PCoefficient;
    this.Dcoefficient = DCoefficient;
    }


    public double UpdatePD(double error){



    double p = PCoefficient*error;

    double d = Dcoefficient;
    double output = p+d;
    return output;

    }
    public double getDcoefficient() {
        return Dcoefficient;
    }

    public double getPCoefficient() {
        return PCoefficient;
    }

    public void setPCoefficient(double PCoefficient) {
        this.PCoefficient = PCoefficient;
    }

    public void setDcoefficient(double Dcoefficient) {
        this.Dcoefficient = Dcoefficient;
    }
}
