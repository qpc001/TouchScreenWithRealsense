/*************************************************************************
	> File Name: SelfDefSelfDefPoint.h
	> Author: 
	> Mail: 
	> Created Time: 2019年01月04日 星期五 20时55分35秒
 ************************************************************************/
#ifndef SelfDefPoint_H
#define SelfDefPoint_H
#include<iostream>
#include<cmath>

class SelfDefPoint
{
    public:
        SelfDefPoint(double _x=0.0,double _y=0.0,double _z=0.0):X(_x),Y(_y),Z(_z){};
        virtual ~SelfDefPoint(){};
        SelfDefPoint(const SelfDefPoint &other) { X=other.X; Y=other.Y; Z=other.Z; };
        SelfDefPoint operator=(const SelfDefPoint&);
        SelfDefPoint operator+(const SelfDefPoint&);
        SelfDefPoint operator-(const SelfDefPoint&);
        SelfDefPoint operator*(SelfDefPoint&);
        friend std::ostream &operator<<(std::ostream &output, const SelfDefPoint &rhs);

        double GetX() { return X; }
        void SetX(double val) { X = val; }
        double GetY() { return Y; }
        void SetY(double val) { Y = val; }
        double GetZ() { return Z; }
        void SetZ(double val) { Z = val; }
        double dotX(SelfDefPoint &pt) { return (pt.GetX()*GetX()+pt.GetY()*GetY()+pt.GetZ()*GetZ()); }
        double mod() { return sqrt(dotX(*this)); }

    protected:

    private:
        double X;
        double Y;
        double Z;
};

#endif // SelfDefPoint_H
