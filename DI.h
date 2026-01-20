#ifndef DI_H
#define DI_H
#include <sys/types.h>
#include <dirent.h>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <cmath>
#include "ompl/base/State.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <map>
#include <cstring>
#include <unsupported/Eigen/Polynomials>
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/util/Console.h"
using namespace std;
namespace ob = ompl::base;

inline Eigen::VectorXd ArrivetimeDDI(double x01, double x02, double x03, double x04, double x11, double x12, double x13, double  x14)
{
     			Eigen::VectorXd time_equ(5);
                       time_equ(0) = - 36*x01*x01 + 72*x01*x11 - 36*x02*x02 + 72*x02*x12 - 36*x11*x11 - 36*x12*x12;  
		        time_equ(1) = 24*x03*x11 - 24*x02*x04 - 24*x01*x13 - 24*x01*x03 - 24*x02*x14 + 24*x04*x12 + 24*x11*x13 + 24*x12*x14;
		        time_equ(2) = -4*x03*x03 - 4*x03*x13 - 4*x04*x04 - 4*x04*x14 - 4*x13*x13 - 4*x14*x14;
		        time_equ(3) = 0.0;
		        time_equ(4) = 1.0;
		        return time_equ;  
}    

inline double estimatedTimeDDI(ompl::base::State *src_,  ompl::base::State *tar_) 
{
         const auto *src10D = src_->as<ob::RealVectorStateSpace::StateType>(); 
         const auto *tar10D = tar_->as<ob::RealVectorStateSpace::StateType>();
         Eigen::VectorXd coeff = ArrivetimeDDI(src10D->values[0], src10D->values[1], src10D->values[2], src10D->values[3], tar10D->values[0], tar10D->values[1], tar10D->values[2], tar10D->values[3]);
    
         Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
         solver.compute(coeff);
         auto roots = solver.roots(); // Using auto for convenience
         //std::cout << "Roots: \n" << roots << std::endl;
         double optimalArriveTime = std::numeric_limits<double>::infinity();
         for (int i = 0; i < roots.size(); i++) {
            if (std::abs(roots[i].imag()) < 1e-8 && roots[i].real() >= 0) {
                //std::cout << "Real root: " << roots[i].real() << std::endl;
                optimalArriveTime = roots[i].real() < optimalArriveTime ? roots[i].real(): optimalArriveTime;
            }
         }
   return optimalArriveTime;      
}

inline ompl::base::Cost EdgeCostDDI(double t_s, double x01, double x02, double x03, double x04, double x11, double x12, double x13, double x14) 
{
              double cost_ =  t_s + (4*x03*x03)/t_s + (12*x01*x01)/(t_s*t_s*t_s) + (4*x04*x04)/t_s + (12*x02*x02)/(t_s*t_s*t_s) +(4*x13*x13)/t_s + (12*x11*x11)/(t_s*t_s*t_s) + (4*x14*x14)/t_s + (12*x12*x12)/(t_s*t_s*t_s) +(12*x01*x03)/(t_s*t_s)  + (12*x02*x04)/(t_s*t_s)- (24*x01*x11)/(t_s*t_s*t_s) + (12*x01*x13)/(t_s*t_s)  - (12*x03*x11)/(t_s*t_s)  + (4*x03*x13)/t_s - (24*x02*x12)/(t_s*t_s*t_s) + (12*x02*x14)/(t_s*t_s) -(12*x04*x12)/(t_s*t_s)  + (4*x04*x14)/t_s - (12*x11*x13)/(t_s*t_s)  - (12*x12*x14)/(t_s*t_s);  
		ompl::base::Cost edgeCost(cost_);
return edgeCost;
}

inline ompl::base::Cost estimatedCostDDI(ompl::base::State *src_,  ompl::base::State *tar_, double arriveTime_)
{
      const auto *src10D = src_->as<ob::RealVectorStateSpace::StateType>(); 
      const auto *tar10D = tar_->as<ob::RealVectorStateSpace::StateType>();
      return EdgeCostDDI(arriveTime_,src10D->values[0], src10D->values[1], src10D->values[2], src10D->values[3],tar10D->values[0], tar10D->values[1], tar10D->values[2], tar10D->values[3]);
} 

inline void steeringDDI(double t, double t_s, double x01, double x02, double x03, double x04, double x11, double x12, double x13, double x14, bool pathN, ompl::base::State *state)
{
             const ob::RealVectorStateSpace::StateType *rstate = static_cast<ob::RealVectorStateSpace::StateType *>(state); 
             rstate->values[0] = x11 + x13*(t - t_s) + ((t - t_s)*(t - t_s)*(t - t_s)*(2*x01 - 2*x11 + t_s*x03 + t_s*x13))/(t_s*t_s*t_s) + ((t - t_s)*(t - t_s)*(3*x01 - 3*x11 + t_s*x03 + 2*t_s*x13))/(t_s*t_s);
             rstate->values[1] = x12 + x14*(t - t_s) + ((t - t_s)*(t - t_s)*(t - t_s)*(2*x02 - 2*x12 + t_s*x04 + t_s*x14))/(t_s*t_s*t_s) + ((t - t_s)*(t-t_s)*(3*x02 - 3*x12 + t_s*x04 + 2*t_s*x14))/(t_s*t_s);
             
             //if(pathN)
             //std::cout <<" (" << rstate->values[0] << "," << rstate->values[1] << ") --" << std::endl;
             
             if(pathN)
             {
                    rstate->values[2] = x13 + (2*(t - t_s)*(3*x01 - 3*x11 + t_s*x03 + 2*t_s*x13))/(t_s*t_s) + (3*(t - t_s)*(t - t_s)*(2*x01 - 2*x11 + t_s*x03 + t_s*x13))/(t_s*t_s*t_s);
                    rstate->values[3] = x14 + (2*(t - t_s)*(3*x02 - 3*x12 + t_s*x04 + 2*t_s*x14))/(t_s*t_s) + (3*(t - t_s)*(t - t_s)*(2*x02 - 2*x12 + t_s*x04 + t_s*x14))/(t_s*t_s*t_s);
             }      
}

inline void interpolateDDI(double t, double t_s, const ompl::base::State *src_, const ompl::base::State * tar_, ompl::base::State *state, bool pathN)
{

      const auto *src10D = src_->as<ob::RealVectorStateSpace::StateType>(); 
      const auto *tar10D = tar_->as<ob::RealVectorStateSpace::StateType>();  
      steeringDDI(t,t_s,src10D->values[0], src10D->values[1], src10D->values[2], src10D->values[3],tar10D->values[0], tar10D->values[1], tar10D->values[2], tar10D->values[3],pathN,state); 
}


#endif
