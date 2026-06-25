#ifndef STEERING_H
#define STEERING_H
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
    
inline Eigen::VectorXd ArrivetimeMGLQ(double x01, double x02, double x03, double x04, double x05, double x06, double x07, double x08, double x09, double x010, double x11, double x12, double x13, double  x14, double x15,  double x16, double x17, double x18, double  x19, double x110)
{   

			double et1 = x06*x16*(-3.38e+2/1.25e+2)-x09*x19*1.259493448399385e-1-x010*x110*1.259493448399385e-1-x06*x06*(3.38e+2/1.25e+2);
			double et2 = x09*x09*(-2.518986896798771e-1)-x010*x010*2.518986896798771e-1-x16*x16*(3.38e+2/1.25e+2);
			double et3 = x19*x19*(-2.518986896798771e-1)-x110*x110*2.518986896798771e-1;
			double et4 = x03*x06*(-1.6224e+1)-x07*x09*7.556960690396312-x08*x010*7.556960690396312-x03*x16*1.6224e+1+x06*x13*1.6224e+1;
			double et5 = x07*x19*(-3.778480345198156)+x09*x17*3.778480345198156;
			double et6 = x010*x18*3.778480345198156+x13*x16*1.6224e+1+x17*x19*7.556960690396312;
			double et7 = x08*x110*(-3.778480345198156)+x18*x110*7.556960690396312;
			double et8 = x04*x010*(-4.62199430605279)+x05*x09*4.62199430605279+x03*x13*4.8672e+1;
			double et9 = x05*x19*3.466495729539593+x07*x17*7.934808724916128e+1;
			double et10 = x09*x15*3.466495729539593-x010*x14*3.466495729539593;
			double et11 = x08*x18*7.934808724916128e+1+x15*x19*4.62199430605279;
			double et12 = x04*x110*(-3.466495729539593)-x14*x110*4.62199430605279-x03*x03*2.4336e+1;
			double et13 = x07*x07*(-5.667720517797234e+1)-x08*x08*5.667720517797234e+1-x13*x13*2.4336e+1;
			double et14 = x17*x17*(-5.667720517797234e+1)-x18*x18*5.667720517797234e+1;
			double et15 = x01*x010*(-1.078465338078984e+1)+x02*x09*1.078465338078984e+1;
			double et16 = x04*x08*(-6.932991459079186e+1)+x05*x07*6.932991459079186e+1;
			double et17 = x02*x19*1.078465338078984e+1-x09*x12*1.078465338078984e+1;
			double et18 = x010*x11*1.078465338078984e+1+x04*x18*6.008592597868628e+1;
			double et19 = x05*x17*(-6.008592597868628e+1)+x07*x15*6.008592597868628e+1;
			double et20 = x08*x14*(-6.008592597868628e+1)-x12*x19*1.078465338078984e+1;
			double et21 = x14*x18*6.932991459079186e+1-x15*x17*6.932991459079186e+1;
			double et22 = x01*x110*(-1.078465338078984e+1)+x11*x110*1.078465338078984e+1;
			double et23 = x01*x08*(-1.617698007118477e+2)+x02*x07*1.617698007118477e+2;
			double et24 = x04*x14*(-4.004786095968269e+1)+x01*x18*1.617698007118477e+2;
			double et25 = x02*x17*(-1.617698007118477e+2)-x07*x12*1.617698007118477e+2;
			double et26 = x08*x11*1.617698007118477e+2-x05*x15*4.004786095968269e+1;
			double et27 = x11*x18*(-1.617698007118477e+2)+x12*x17*1.617698007118477e+2;
			double et28 = x04*x04*(-2.120180874336142e+1)-x05*x05*2.120180874336142e+1;
			double et29 = x14*x14*(-2.120180874336142e+1)-x15*x15*2.120180874336142e+1;
			double et30 = x01*x04*(-9.894177413568665e+1)-x02*x05*9.894177413568665e+1;
			double et31 = x01*x14*(-9.894177413568665e+1)+x04*x11*9.894177413568665e+1;
			double et32 = x02*x15*(-9.894177413568665e+1)+x05*x12*9.894177413568665e+1;
			double et33 = x11*x14*9.894177413568665e+1+x12*x15*9.894177413568665e+1;
			double et34 = x01*x11*2.308641396499355e+2+x02*x12*2.308641396499355e+2;
			double et35 = x01*x01*(-1.154320698249678e+2)-x02*x02*1.154320698249678e+2;
			double et36 = x11*x11*(-1.154320698249678e+2)-x12*x12*1.154320698249678e+2;  

			Eigen::VectorXd time_equ(9);


		        time_equ(0) = et34+et35+et36;
		        time_equ(1) = et30+et31+et32+et33;
		        time_equ(2) = et23+et24+et25+et26+et27+et28+et29;
		        time_equ(3) = et15+et16+et17+et18+et19+et20+et21+et22;
		        time_equ(4) = et8+et9+et10+et11+et12+et13+et14;
		        time_equ(5) = et4+et5+et6+et7;
		        time_equ(6) = et1+et2+et3;
		        time_equ(7) = 0.0;
		        time_equ(8) = 1.0;
		        return time_equ;
}

inline double estimatedTimeMGLQ(ompl::base::State *src_,  ompl::base::State *tar_) 
{
         const auto *src10D = src_->as<ob::RealVectorStateSpace::StateType>(); 
         const auto *tar10D = tar_->as<ob::RealVectorStateSpace::StateType>();
         Eigen::VectorXd coeff = ArrivetimeMGLQ(src10D->values[0], src10D->values[1], src10D->values[2], src10D->values[3], src10D->values[4], src10D->values[5], src10D->values[6], src10D->values[7], src10D->values[8], src10D->values[9],tar10D->values[0], tar10D->values[1], tar10D->values[2], tar10D->values[3], tar10D->values[4], tar10D->values[5], tar10D->values[6], tar10D->values[7], tar10D->values[8], tar10D->values[9]);
    
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



        
inline ompl::base::Cost EdgeCostMGLQ(double t_s, double x01, double x02, double x03, double x04, double x05, double x06, double x07, double x08, double x09, double x010, double x11, double x12, double x13, double x14, double x15, double x16, double x17, double x18, double x19, double x110) 
{

		double t2 = 1.0/t_s;
		double t3 = t2*t2;
		double t4 = pow(t2,3);
		double t6 = pow(t2,5);
		double t8 = pow(t2,7);
		double t5 = pow(t3,2);
		double t7 = pow(t3,3);
		double et1 = t_s+t4*x03*x03*8.112+t2*x06*x06*(3.38e+2/1.25e+2)+t8*x01*x01*1.649029568928111e+1;
		double et2 = t6*x04*x04*4.240361748672285;
		double et3 = t8*x02*x02*1.649029568928111e+1;
		double et4 = t2*x09*x09*2.518986896798771e-1;
		double et5 = t4*x07*x07*1.889240172599078e+1;
		double et6 = t6*x05*x05*4.240361748672285;
		double et7 = t2*x010*x010*2.518986896798771e-1;
		double et8 = t4*x08*x08*1.889240172599078e+1+t4*x13*x13*8.112+t2*x16*x16*(3.38e+2/1.25e+2);
		double et9 = t8*x11*x11*1.649029568928111e+1;
		double et10 = t6*x14*x14*4.240361748672285;
		double et11 = t8*x12*x12*1.649029568928111e+1;
		double et12 = t2*x19*x19*2.518986896798771e-1;
		double et13 = t4*x17*x17*1.889240172599078e+1;
		double et14 = t6*x15*x15*4.240361748672285;
		double et15 = t4*x18*x18*1.889240172599078e+1;
		double et16 = t2*x110*x110*2.518986896798771e-1+t3*x03*x06*8.112;
		double et17 = t7*x01*x04*1.649029568928111e+1;
		double et18 = t7*x02*x05*1.649029568928111e+1;
		double et19 = t6*x01*x08*3.235396014236954e+1;
		double et20 = t6*x02*x07*(-3.235396014236954e+1);
		double et21 = t5*x01*x010*2.696163345197461;
		double et22 = t5*x02*x09*(-2.696163345197461);
		double et23 = t5*x04*x08*1.733247864769797e+1;
		double et24 = t5*x05*x07*(-1.733247864769797e+1);
		double et25 = t4*x04*x010*1.540664768684264;
		double et26 = t4*x05*x09*(-1.540664768684264);
		double et27 = t3*x07*x09*3.778480345198157-t4*x03*x13*1.6224e+1;
		double et28 = t8*x01*x11*(-3.298059137856222e+1);
		double et29 = t3*x08*x010*3.778480345198157+t3*x03*x16*8.112-t3*x06*x13*8.112;
		double et30 = t7*x01*x14*1.649029568928111e+1;
		double et31 = t7*x04*x11*(-1.649029568928111e+1);
		double et32 = t8*x02*x12*(-3.298059137856222e+1)+t2*x06*x16*(3.38e+2/1.25e+2);
		double et33 = t6*x04*x14*8.009572191936539;
		double et34 = t7*x02*x15*1.649029568928111e+1;
		double et35 = t7*x05*x12*(-1.649029568928111e+1);
		double et36 = t6*x01*x18*(-3.235396014236954e+1);
		double et37 = t6*x02*x17*3.235396014236954e+1;
		double et38 = t6*x07*x12*3.235396014236954e+1;
		double et39 = t6*x08*x11*(-3.235396014236954e+1);
		double et40 = t5*x02*x19*(-2.696163345197461);
		double et41 = t5*x09*x12*2.696163345197461;
		double et42 = t5*x010*x11*(-2.696163345197461);
		double et43 = t6*x05*x15*8.009572191936539;
		double et44 = t5*x04*x18*(-1.502148149467157e+1);
		double et45 = t5*x05*x17*1.502148149467157e+1;
		double et46 = t5*x07*x15*(-1.502148149467157e+1);
		double et47 = t5*x08*x14*1.502148149467157e+1;
		double et48 = t4*x05*x19*(-1.155498576513198);
		double et49 = t4*x07*x17*(-2.64493624163871e+1);
		double et50 = t4*x09*x15*(-1.155498576513198);
		double et51 = t4*x010*x14*1.155498576513198;
		double et52 = t3*x07*x19*1.889240172599078;
		double et53 = t3*x09*x17*(-1.889240172599078);
		double et54 = t2*x09*x19*1.259493448399386e-1;
		double et55 = t4*x08*x18*(-2.64493624163871e+1);
		double et56 = t3*x010*x18*(-1.889240172599078)-t3*x13*x16*8.112;
		double et57 = t7*x11*x14*(-1.649029568928111e+1);
		double et58 = t7*x12*x15*(-1.649029568928111e+1);
		double et59 = t6*x11*x18*3.235396014236954e+1;
		double et60 = t6*x12*x17*(-3.235396014236954e+1);
		double et61 = t5*x12*x19*2.696163345197461;
		double et62 = t5*x14*x18*(-1.733247864769797e+1);
		double et63 = t5*x15*x17*1.733247864769797e+1;
		double et64 = t4*x15*x19*(-1.540664768684264);
		double et65 = t3*x17*x19*(-3.778480345198157);
		double et66 = t5*x01*x110*2.696163345197461;
		double et67 = t4*x04*x110*1.155498576513198;
		double et68 = t3*x08*x110*1.889240172599078;
		double et69 = t2*x010*x110*1.259493448399386e-1;
		double et70 = t5*x11*x110*(-2.696163345197461);
		double et71 = t4*x14*x110*1.540664768684264;
		double et72 = t3*x18*x110*(-3.778480345198157);
		ompl::base::Cost edgeCost(et1+et2+et3+et4+et5+et6+et7+et8+et9+et10+et11+et12+et13+et14+et15+et16+et17+et18+et19+et20+et21+et22+et23+et24+et25+et26+et27+et28+et29+et30+et31+et32+et33+et34+et35+et36+et37+et38+et39+et40+et41+et42+et43+et44+et45+et46+et47+et48+et49+et50+et51+et52+et53+et54+et55+et56+et57+et58+et59+et60+et61+et62+et63+et64+et65+et66+et67+et68+et69+et70+et71+et72);
return edgeCost;
}

inline ompl::base::Cost estimatedCostMGLQ(ompl::base::State *src_,  ompl::base::State *tar_, double arriveTime_)
{
      const auto *src10D = src_->as<ob::RealVectorStateSpace::StateType>(); 
      const auto *tar10D = tar_->as<ob::RealVectorStateSpace::StateType>();
      return EdgeCostMGLQ(arriveTime_,src10D->values[0], src10D->values[1], src10D->values[2], src10D->values[3], src10D->values[4], src10D->values[5], src10D->values[6], src10D->values[7], src10D->values[8], src10D->values[9],tar10D->values[0], tar10D->values[1], tar10D->values[2], tar10D->values[3], tar10D->values[4], tar10D->values[5], tar10D->values[6], tar10D->values[7], tar10D->values[8], tar10D->values[9]);
} 

inline ompl::base::Cost lqEdgeCostMGLQ(ompl::base::State *src_,  ompl::base::State *tar_, double &arriveTime_ )
{
        arriveTime_ = estimatedTimeMGLQ(src_, tar_); 
        return estimatedCostMGLQ(src_, tar_, arriveTime_);
}


inline ompl::base::Cost motionEdgeCostMGLQ(ompl::base::State *src_,  ompl::base::State *tar_)
{
        double arriveTime_ = estimatedTimeMGLQ(src_, tar_); 
        return estimatedCostMGLQ(src_, tar_, arriveTime_);

}

inline void steeringMGLQ(double t, double t_s, double x01, double x02, double x03, double x04, double x05, double x06, double x07, double x08, double x09, double x010, double x11, double x12, double x13, double x14, double x15, double x16, double x17, double x18, double x19, double x110, bool pathN, ompl::base::State *state)
{
               const ob::RealVectorStateSpace::StateType *rstate = static_cast<ob::RealVectorStateSpace::StateType *>(state); 
		double t2 = t_s*x06;
		double t3 = t_s*x16;
		double t4 = t_s*t_s;
		double t5 = t_s*t_s*t_s;
		double t6 = x03*2.0;
		double t7 = x03*3.0;
		double t8 = x13*2.0;
		double t9 = x13*3.0;
		double t11 = -t_s;
		double t15 = 1.0/pow(t_s,5);//et1
		double t17 = 1.0/pow(t_s,7);
		double t21 = x01*4.0e+3;
		double t22 = x02*4.0e+3;
		double t23 = x11*4.0e+3;
		double t24 = x12*4.0e+3;
		double t25 = x01*5.6e+3;
		double t26 = x02*5.6e+3;
		double t27 = x11*5.6e+3;
		double t28 = x12*5.6e+3;
		double t29 = x01*7.0e+3;
		double t30 = x02*7.0e+3;
		double t31 = x11*7.0e+3;
		double t32 = x12*7.0e+3;
		double t33 = t_s*x04*2.0e+3;
		double t34 = t_s*x05*2.0e+3;
		double t35 = t_s*x14*2.0e+3;
		double t36 = t_s*x15*2.0e+3;
		double t37 = t_s*x04*2.6e+3;
		double t38 = t_s*x05*2.6e+3;
		double t39 = t_s*x04*3.0e+3;
		double t40 = t_s*x05*3.0e+3;
		double t41 = t_s*x14*3.0e+3;
		double t42 = t_s*x15*3.0e+3;
		double t43 = t_s*x14*4.0e+3;
		double t44 = t_s*x15*4.0e+3;
		double t45 = t_s*x04*6.8e+3;
		double t46 = t_s*x05*6.8e+3;
		double t47 = t_s*x14*7.2e+3;
		double t48 = t_s*x15*7.2e+3;
		double t55 = x01*1.4e+4;
		double t56 = x02*1.4e+4;
		double t57 = x11*1.4e+4;
		double t58 = x12*1.4e+4;
		double t103 = t*(9.81e+2/1.0e+2);
		double t104 = t_s*(9.81e+2/1.0e+2);
		double t112 = t*6.351759915993783e+1;
		double t113 = t_s*6.351759915993783e+1;
		double t10 = t3*2.0;
		double t12 = 1.0/t4;
		double t13 = 1.0/t5;
		double t18 = -t8;
		double t19 = -t9;
		double t20 = t+t11;//et1
		double t49 = -t23;
		double t50 = -t24;
		double t51 = -t27;
		double t52 = -t28;
		double t53 = -t31;
		double t54 = -t32;
		double t65 = t5*x09*3.27e+2;
		double t66 = t5*x010*3.27e+2;
		double t67 = t5*x19*3.27e+2;
		double t68 = t5*x110*3.27e+2;
		double t69 = -t57;
		double t70 = -t58;
		double t73 = t5*x19*6.54e+2;
		double t74 = t5*x110*6.54e+2;
		double t75 = t5*x09*9.81e+2;
		double t76 = t5*x010*9.81e+2;
		double t77 = t5*x19*1.308e+3;
		double t78 = t5*x110*1.308e+3;
		double t79 = t4*x07*3.924e+3;
		double t80 = t4*x08*3.924e+3;
		double t81 = t4*x17*3.924e+3;
		double t82 = t4*x18*3.924e+3;
		double t83 = t4*x07*4.578e+3;
		double t84 = t4*x08*4.578e+3;
		double t85 = t4*x07*4.905e+3;
		double t86 = t4*x08*4.905e+3;
		double t87 = t4*x17*6.54e+3;
		double t88 = t4*x18*6.54e+3;
		double t97 = t4*x17*9.81e+3;
		double t98 = t4*x18*9.81e+3;
		double t99 = t4*x07*1.2753e+4;
		double t100 = t4*x08*1.2753e+4;
		double t101 = t4*x17*1.4715e+4;
		double t102 = t4*x18*1.4715e+4;
		double t108 = -t104;
		double t114 = -t113;
		double t14 = t12*t12;//et1
		double t16 = t12*t12*t12;
		double t59 = t20*t20;//et1
		double t60 = t20*t20*t20;//et1
		double t62 = pow(t20,5);//et1
		double t64 = pow(t20,7);//et2
		double t71 = -t65;
		double t72 = -t67;
		double t89 = -t73;
		double t90 = -t75;
		double t91 = -t77;
		double t92 = -t79;
		double t93 = -t82;
		double t94 = -t83;
		double t95 = -t85;
		double t96 = -t88;
		double t105 = -t98;
		double t106 = -t99;
		double t107 = -t102;
		double t109 = t2+t3+t6+t18;
		double t110 = t2+t7+t10+t19;
		double t111 = t103+t108;
		double t122 = t112+t114;
		double t61 = t59*t59;//et1
		double t63 = pow(t59,3);
		double t115 = t21+t33+t35+t49+t66+t68+t80+t93;//et2
		double t116 = t25+t37+t41+t51+t66+t74+t84+t96;//et1
		double t117 = t22+t34+t36+t50+t71+t72+t81+t92;
		double t118 = t29+t39+t43+t53+t66+t78+t86+t105;//et1
		double t119 = t26+t38+t42+t52+t71+t87+t89+t94;
		double t120 = t30+t40+t44+t54+t71+t91+t95+t97;
		double t121 = t45+t47+t55+t69+t76+t78+t100+t107;
		double t123 = t46+t48+t56+t70+t90+t91+t101+t106;
		double et1 = x11+t20*x14+t59*x18*(9.81e+2/2.0e+2)+t60*x110*(3.27e+2/2.0e+2)+t14*t61*t118*5.0e-3+t15*t62*t116*1.5e-2;
		double et2 = t17*t64*t115*5.0e-3+t16*t63*t121*5.0e-3;
		double et3 = x12+t20*x15-t59*x17*(9.81e+2/2.0e+2)-t60*x19*(3.27e+2/2.0e+2)+t14*t61*t120*5.0e-3+t15*t62*t119*1.5e-2;
		double et4 = t17*t64*t117*5.0e-3+t16*t63*t123*5.0e-3;
		double et5 = x14+t111*x18+t59*x110*(9.81e+2/2.0e+2)+t14*t60*t118*2.0e-2+t15*t61*t116*7.5e-2;
		double et6 = t17*t63*t115*3.5e-2+t16*t62*t121*3.0e-2;
		double et7 = x15-t59*x19*(9.81e+2/2.0e+2)-t111*x17+t14*t60*t120*2.0e-2+t15*t61*t119*7.5e-2;
		double et8 = t17*t63*t117*3.5e-2+t16*t62*t123*3.0e-2;
		double et9 = x17+t20*x19-t14*t59*t120*6.116207951070337e-3-t15*t60*t119*3.058103975535168e-2;
		double et10 = t17*t62*t117*(-2.140672782874618e-2)-t16*t61*t123*1.529051987767584e-2;
		double et11 = x18+t20*x110+t14*t59*t118*6.116207951070337e-3+t15*t60*t116*3.058103975535168e-2;
		double et12 = t17*t62*t115*2.140672782874618e-2+t16*t61*t121*1.529051987767584e-2;
		double et13 = x19-t15*t59*t119*9.174311926605505e-2-t17*t61*t117*1.070336391437309e-1;
		double et14 = t16*t60*t123*(-6.116207951070337e-2)-t14*t120*t122*1.925830960855329e-4;
		double et15 = x110+t15*t59*t116*9.174311926605505e-2+t17*t61*t115*1.070336391437309e-1;
		double et16 = t16*t60*t121*6.116207951070337e-2+t14*t118*t122*1.925830960855329e-4;
               rstate->values[0] = et1+et2;
               rstate->values[1] = et3+et4;
               rstate->values[2] = x13+t20*x16+t12*t59*t110+t13*t60*t109;
               if(pathN)
               {
                    rstate->values[3] = et5+et6;
                    rstate->values[4] = et7+et8;
                    rstate->values[5] = x16+t12*t110*(t*(2.5e+2/1.69e+2)-t_s*(2.5e+2/1.69e+2))*(1.69e+2/1.25e+2)+t13*t59*t109*3.0;
                    rstate->values[6] = et9+et10;
                    rstate->values[7] = et11+et12;
                    rstate->values[8] = et13+et14;
                    rstate->values[9] = et15+et16;
               }
               //cerr << rstate->values[0] << " " << rstate->values[1] << " " << rstate->values[2] << endl;
               //<< " " << et5+et6 << " " << et7+et8 << " " <<  (x16+t12*t110*(t*(2.5e+2/1.69e+2)-t_s*(2.5e+2/1.69e+2))*(1.69e+2/1.25e+2)+t13*t59*t109*3.0) << " " << et9+et10 << " " << et11+et12 << " " << et13+et14 << "  " << et15+et16;
} 
       
        
inline void interpolateMGLQ(double t, double t_s, const ompl::base::State *src_, const ompl::base::State * tar_, ompl::base::State *state, bool pathN)
{

      const auto *src10D = src_->as<ob::RealVectorStateSpace::StateType>(); 
      const auto *tar10D = tar_->as<ob::RealVectorStateSpace::StateType>();  
      steeringMGLQ(t,t_s,src10D->values[0], src10D->values[1], src10D->values[2], src10D->values[3], src10D->values[4], src10D->values[5], src10D->values[6], src10D->values[7], src10D->values[8], src10D->values[9],tar10D->values[0], tar10D->values[1], tar10D->values[2], tar10D->values[3], tar10D->values[4], tar10D->values[5], tar10D->values[6], tar10D->values[7], tar10D->values[8], tar10D->values[9],pathN,state); 
}
 
 
 
        
#endif

