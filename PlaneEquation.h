/*************************************************************************
	> File Name: PlaneEquation.h
	> Author: 
	> Mail: 
	> Created Time: 2019年01月04日 星期五 20时53分18秒
 ************************************************************************/
#ifndef PLANEEQUATION_H
#define PLANEEQUATION_H
#include<iostream>

class PlaneEquation
{
    public:
        PlaneEquation(double _A=0.0,double _B=0.0,double _C=0.0,double _D=0.0):A(_A),B(_B),C(_C),D(_D){};
        virtual ~PlaneEquation(){};
        PlaneEquation(const PlaneEquation&);
        PlaneEquation &operator=(const PlaneEquation&);
        friend std::ostream &operator<<(std::ostream&, const PlaneEquation&);

        double GetA() { return A; }
        void SetA(double val) { A = val; }
        double GetB() { return B; }
        void SetB(double val) { B = val; }
        double GetC() { return C; }
        void SetC(double val) { C = val; }
        double GetD() { return D; }
        void SetD(double val) { D = val; }

    protected:

    private:
        double A;
        double B;
        double C;
        double D;
};

#endif // PLANEEQUATION_H
