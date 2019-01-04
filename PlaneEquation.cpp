/*************************************************************************
	> File Name: PlaneEquation.cpp
	> Author: 
	> Mail: 
	> Created Time: 2019年01月04日 星期五 20时54分06秒
 ************************************************************************/
#include "PlaneEquation.h"

PlaneEquation::PlaneEquation(const PlaneEquation &other)
{
    //copy ctor
    A=other.A;
    B=other.B;
    C=other.C;
    D=other.D;
}

PlaneEquation &PlaneEquation::operator=(const PlaneEquation &rhs)
{
    if (this == &rhs) return *this; // handle self assignment
    //assignment operator
    A=rhs.A;
    B=rhs.B;
    C=rhs.C;
    D=rhs.D;
    return *this;
}

std::ostream &operator<<(std::ostream &output, const PlaneEquation &rhs)
{
    if (rhs.A!=0.0) {
        if (rhs.A==-1.0) output<<"-";
        else if (rhs.A!=1.0) output<<rhs.A;
        output<<"X";
    }

    if (rhs.B!=0.0) {
        if (rhs.B>0.0) {
            output<<"+";
            if (rhs.B!=1.0) output<<rhs.B;
        } else if (rhs.B!=-1.0) output<<rhs.B;
                else output<<"-";
        output<<"Y";
    }

    if (rhs.C!=0.0) {
        if (rhs.C>0.0) {
            output<<"+";
            if (rhs.C!=1.0) output<<rhs.C;
        } else if (rhs.C!=-1.0) output<<rhs.C;
                else output<<"-";
        output<<"Z";
    }

    if (rhs.D!=0.0) {
        if (rhs.D>0.0) output<<"+";
        output<<rhs.D;
    }

    output<<"=0";

    return output;
}
