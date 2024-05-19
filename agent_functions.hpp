
#include <iostream>
#include <vector>
#include  <array>
#include <cmath>
#include <tuple>


#define NUM_OF_COEFFS 5

std::array<double,NUM_OF_COEFFS> calculateCoefficients(double v0, double a0, double sf, double vf, double af, double tf) 
{
    
    double c1 = v0;
    double c2 = a0;
    double c3 = (3 * af - 9 * a0) / tf + (60 * sf) / (tf * tf * tf) - 12 * (2 * vf + 3 * v0) / (tf * tf);
    double c4 = (36 * a0 - 24 * af) / (tf * tf) - 360 * sf / (tf * tf * tf * tf) + 24 * (7 * vf + 8 * v0) / (tf * tf * tf);
    double c5 = 60 * (af - a0) / (tf * tf * tf) + 720 * sf / (tf * tf * tf * tf * tf) - 360 * (vf + v0) / (tf * tf * tf * tf);

   
    std::array<double,NUM_OF_COEFFS> coefficients = {c1, c2, c3, c4, c5};

    return coefficients;
}
double calculateJopt(double c3, double c4, double c5, double t) {
    return c3 + c4 * t + 0.5 * c5 * t * t;
}


void printCoefficients(const std::array<double,NUM_OF_COEFFS>& coefficients) {
    for (double coeff : coefficients) {
        std::cout << coeff << " ";
    }
    std::cout << std::endl;
}

double finaloptTimestop(double v0, double a0, double sf) {
    return 10 * sf / (2 * v0 + sqrt(4 * v0 * v0 + 5 * a0 * sf));
};

std::array<double, NUM_OF_COEFFS> stopPrimitive(double a0, double v0, double sf) {
    std::array<double, NUM_OF_COEFFS> m;
    double smax, tf;

    // Check initial conditions
    if (v0 <= 0.0 || sf == 0.0) {
        return {0.0,0.0,0.0,0.0,0.0}; // m is empty, smax and tf are zero
    }

    // Calculate smax and tf based on the conditions
    if ((4 *v0*v0) + 5 * a0 * sf < 0) {
        smax = -(4 * v0*v0 )/ (5 * a0);
        tf = 10 * smax / (2 * v0);
    } else {
        smax = sf;
        tf = finaloptTimestop(v0, a0, smax); // Using the existing function from the file
    }

    // Placeholder for evalPrimitiveCoeffs - replace with actual implementation
    m = calculateCoefficients(v0, a0,smax,0.0,0.0, tf);
    return m;
}

double LowLevelControl(double& a0, const std::array<double, NUM_OF_COEFFS>& m, double delta_t, double amin, double amax) {
    

    
    double areq = a0 + delta_t / 2 * (calculateJopt(m[2],m[3],m[4],0) + calculateJopt(m[2],m[3],m[4],delta_t));


    
    areq = std::min(std::max(areq, -amin), amax);

   
    a0 = areq;
    return areq;
}
class PIDController {
    private:
    double a0=0.0;
    double amax = 10;
    double Pgain;
    double Igain;
    double eint = 0.0;  
    double e_prev = 0.0;

    
public:

    
    PIDController(double Pgain, double Igain) : Pgain(Pgain), Igain(Igain), eint(0.0) {}

    
     double getPedalRequest(double a, double dt, std::array<double, NUM_OF_COEFFS> m) {
        
        double areq = a0 + (dt *0.5 )* (calculateJopt(m[2],m[3],m[4],0) + calculateJopt(m[2],m[3],m[4],dt));
        areq = std::min(std::max(areq, -amax), amax);       
        a0 = areq;

        double e = areq - a;
        eint += e * dt;

        double Req_pedal = e * Pgain + eint * Igain;
        e_prev = e;

        return Req_pedal;
    }

   
    void resetIntegral() {
        eint = 0.0;
        e_prev = 0.0;
        a0 = 0.0;
        
        
    }
};

void resetIntegral(){
   double eint=0.0;
};

double finalOptVel(double v0, double a0, double sf, double tf) {
    
    double vf = (15.0 / 8.0) * (sf/tf) - (a0 * tf / 8.0) - (7.0 * v0 / 8.0);
    return vf;
}

double finalOptTime(double v0, double a0, double sf, double vf) {
   
    double q = sqrt(60 * a0 * sf + pow(7 * v0 + 8 * vf, 2));
    double Tvf = 30 * sf / (7 * v0 + 8 * vf + q);
    return Tvf;
}

double max(double num1, double num2) {
    
    return (num1 > num2) ? num1 : num2;
}
double min(double num1, double num2) {
    
    return (num1 < num2) ? num1 : num2;
}

std::tuple<std::array<double, NUM_OF_COEFFS>, std::array<double, NUM_OF_COEFFS>>Passprimitive(double a0, double v0, double sf, double Vmin, double Vmax, double Tmin, double Tmax) {


    double T_vmin=0, T_vmax=0, T_star, V_star,T1, T2;
    std::array<double, NUM_OF_COEFFS> m1 = {0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, NUM_OF_COEFFS> m2 = {0.0, 0.0, 0.0, 0.0, 0.0};   
    if(a0>=0){
        T_vmin=finalOptTime(v0,a0,sf,Vmin);
        T_vmax=finalOptTime(v0,a0,sf,Vmax);
    }
    else{
        T_star = sqrt(15*sf/-a0);
        V_star = 0.25*(sqrt(15-a0*sf)-7*v0);
        if(V_star< Vmin && V_star< Vmax){
            T_vmin=finalOptTime(v0, a0,sf,Vmin);
            T_vmax=finalOptTime(v0,a0,sf,Vmax);
        }
        else if (Vmin < V_star && V_star < Vmax) // check the condition 
        {
            T_vmin= T_star;
            T_vmax= finalOptTime(v0,a0,sf,Vmax);
        }
        else{
            T_vmin=0;
            T_vmax =0;
        }
        
    }
    if(Tmin == 0 && Tmax == 0){
        T1= T_vmax;
        T2=T_vmin;
    }
    else{
        T1= max(Tmin,T_vmax);
        T2= min(Tmax,T_vmin);
    }
    
    if (T_vmax!=0 && T1 > 0 && T1 <= T2) {
        Vmin=finalOptVel(v0,a0,sf,T2);
        Vmax=finalOptVel(v0,a0,sf,T1);
        m1=calculateCoefficients(v0,a0,sf,Vmax,0.0,T1);
        m2=calculateCoefficients(v0,a0,sf,Vmin,0.0,T2);
        
    }   
    else{
        m1={0.0,0.0,0.0,0.0,0.0};
        m2={0.0,0.0,0.0,0.0,0.0};
    }
    return std::make_tuple(m1,m2);
}

double FinalOptVelj0(double v0, double a0, double sf, double tf) {
    
    double vfj0 = 0.125 * ((20.0 * sf /tf )- tf * 3.0 * a0 - 12.0 * v0);
    return vfj0;
}

double finalOptTimej0_sol1(double v0, double a0, double sf) {
   double finalopttimej0_sol1;
   double q= sqrt(5)*sqrt(8*a0*sf+5*v0*v0);
   finalopttimej0_sol1 = (10*sf)/5*v0-q;
    return finalopttimej0_sol1;
}
double finalOptTimej0_sol2(double v0, double a0, double sf) {
   double finalopttimej0_sol2;
   double q= sqrt(5)*sqrt(8*a0*sf+5*v0*v0);
   finalopttimej0_sol2 = (10*sf)/(q+5*v0);
    return finalopttimej0_sol2;
}

std::array<double, NUM_OF_COEFFS> Passprimitve_with_j0(double a0, double v0, double sf, double Vmin, double Vmax){
    double T1,V1,T2,V2;
    std::array<double, NUM_OF_COEFFS> m1;
    T1=finalOptTimej0_sol1(v0,a0,sf);
    V1=FinalOptVelj0(v0,a0,sf,T1);
    if (Vmin<V1 && Vmin<Vmax){
        m1=calculateCoefficients(v0,a0,sf,V1,0.0,T1);
    }
    else{
        T2=finalOptTimej0_sol2(v0,a0,sf);
        V2=FinalOptVelj0(v0,a0,sf,T2);
        if(Vmin<V2 && Vmin<Vmax){
            m1=calculateCoefficients(v0,a0,sf,V2,0.0,T2);
        }
        else{
            m1={0.0,0.0,0.0,0.0,0.0};
        }
    }
    return m1;
}
std::array<double, NUM_OF_COEFFS>choose_m_star(double v0, double a0, double lookhead, double Vr){
    std::tuple<std::array<double, NUM_OF_COEFFS>, std::array<double, NUM_OF_COEFFS>> tuple;
    tuple=Passprimitive(a0,v0,lookhead,Vr,Vr,0.0,0.0);
    std::array<double,NUM_OF_COEFFS>tup1=std::get<0>(tuple);
    std::array<double,NUM_OF_COEFFS>tup2=std::get<1>(tuple);
    std::array<double,NUM_OF_COEFFS>m_star;
    if (tup1[2] < tup2[2]) {
    m_star = tup1;
    } else {
    m_star = tup2;
    }
    return m_star;
}
