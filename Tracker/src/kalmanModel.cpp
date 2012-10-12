#include "boost_1_49_0/boost/numeric/ublas/io.hpp"
#include <BayesFilter/unsFlt.hpp>
#include <iostream>

using namespace Bayesian_filter;
using namespace Bayesian_filter_matrix;
/*
 * Prediction model
 */
class Human_predict : public Linear_predict_model
{
protected:
	const double _dt;
public:
	Human_predict(double dt, double accelerationVariance):
		Linear_predict_model(4, 2), _dt(dt)
  {
  Fx(0, 0) = 1.;   Fx(0, 1) = 0.;   Fx(0, 2) = _dt;  Fx(0, 3) = 0.;
	Fx(1, 0) = 0.;   Fx(1, 1) = 1.;   Fx(1, 2) = 0.;   Fx(1, 3) = _dt;
	Fx(2, 0) = 0.;   Fx(2, 1) = 0.;   Fx(2, 2) = 1.;   Fx(2, 3) = 0.;
	Fx(3, 0) = 0.;   Fx(3, 1) = 0.;   Fx(3, 2) = 0.;   Fx(3, 3) = 1.;

	q[0] = accelerationVariance;
	q[1] = accelerationVariance;

	G(0, 0) = _dt * _dt / 2.; 	G(0, 1) = 0.;	
	G(1, 0) = 0.;  						  G(1, 1) = _dt * _dt / 2.;
	G(2, 0) = _dt;							G(2, 1) = 0.;
	G(3, 0) = 0.;							  G(3, 1) = _dt;

	};

	~Human_predict(){};
};

/*
 * Observation model
 */
class Human_observe : public Linear_uncorrelated_observe_model
{
protected:
	const double _positionVariance;
public:
	Human_observe (double positionVariance):
		Linear_uncorrelated_observe_model(4, 2), _positionVariance(positionVariance){

		Hx(0, 0) = 1.0; 	Hx(0, 1) = 0.; 	  Hx(0, 2) = 0.; 	  Hx(0, 3) = 0.;
		Hx(1, 0) = 0.; 	  Hx(1, 1) = 1.0; 	Hx(1, 2) = 0.; 	  Hx(1, 3) = 0.;

		Zv[0] = _positionVariance;
		Zv[1] = _positionVariance;
	};
  
	~Human_observe(){};	
};

class MahalanobisParameters
{
public:
	MahalanobisParameters() : x(0), y(0), matrix(Empty)
	{
		matrix.resize(2, 2, false);
	}
	double x;
	double y;
	Bayesian_filter_matrix::SymMatrix matrix;
	
};

