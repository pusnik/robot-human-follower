#include <ros/ros.h>
#include "kalmanStates.h"


using namespace Bayesian_filter;
using namespace Bayesian_filter_matrix;

//Acceleration Variance: 4.0
KalmanStates::KalmanStates(double dt, double positionVariance, double accelerationVariance) :
		//depth_multiplier: std::pow(0.005 / 1.96, 2)
		_dt(dt), _positionVariance(positionVariance), _depthMultiplier(0.000006508),
		_accelerationVariance(accelerationVariance)
{
	predict_model = new Human_predict (dt, accelerationVariance);
	observe_model = new Human_observe (positionVariance);
	_filter = new Unscented_scheme(4);
}

KalmanStates::KalmanStates(const KalmanStates& orig){
	*this = orig;
}

KalmanStates& KalmanStates::operator=(const KalmanStates& orig){
	this->_dt = orig._dt;
	this->_positionVariance = orig._positionVariance;
	this->_depthMultiplier = orig._depthMultiplier;
	this->_accelerationVariance = orig._accelerationVariance;
	this->predict_model = new Human_predict(_dt, _accelerationVariance);
	this->observe_model = new Human_observe(_positionVariance);
	this->_filter = new Unscented_scheme(4, 2);

	this->_filter->s.resize(orig._filter->s.size(), false);
	for(size_t i = 0; i < orig._filter->s.size(); i++){
		this->_filter->s(i) = orig._filter->s(i);
	}

	this->_filter->S.resize(orig._filter->S.size1(), orig._filter->S.size2(), false);
	for(size_t i = 0; i < orig._filter->S.size1(); i++){
		for(size_t j = 0; j < orig._filter->S.size2(); j++){
			this->_filter->S(i, j) = orig._filter->S(i, j);
		}
	}

	this->_filter->SI.resize(orig._filter->SI.size1(), orig._filter->SI.size2(), false);
	for(size_t i = 0; i < orig._filter->SI.size1(); i++){
		for(size_t j = 0; j < orig._filter->SI.size2(); j++){
			this->_filter->SI(i, j) = orig._filter->SI(i, j);
		}
	}

	this->_filter->x.resize(orig._filter->x.size(), false);
	for(size_t i = 0; i < orig._filter->x.size(); i++){
		this->_filter->x(i) = orig._filter->x(i);
	}

	this->_filter->X.resize(orig._filter->X.size1(), orig._filter->X.size2(), false);
	
	for(size_t i = 0; i < orig._filter->X.size1(); i++){
		for(size_t j = 0; j < orig._filter->X.size2(); j++){
			this->_filter->X(i, j) = orig._filter->X(i, j);
		}
	}

	this->_filter->XX.resize(orig._filter->XX.size1(), orig._filter->XX.size2(), false);
	for(size_t i = 0; i < orig._filter->XX.size1(); i++){
		for(size_t j = 0; j < orig._filter->XX.size2(); j++){
			this->_filter->XX(i, j) = orig._filter->XX(i, j);
		}
	}
	return *this;
}

KalmanStates::~KalmanStates()
{
	delete predict_model;
	delete observe_model;
	delete _filter;
}

void KalmanStates::init(double x, double y, double distance){

	Vec state(4);
	SymMatrix cov(4, 4);

	state[0] = x;
	state[1] = y;
	state[2] = 0.0;
	state[3] = 0.0;

	cov(0, 0) = 0.;	cov(0, 1) = 0.;	cov(0, 2) = 0.;			cov(0, 3) = 0.;
	cov(1, 0) = 0.;	cov(1, 1) = 0.;	cov(1, 2) = 0.;			cov(1, 3) = 0.;
	cov(2, 0) = 0.;	cov(2, 1) = 0.;	cov(2, 2) = 1.;   	cov(2, 3) = 0.;
	cov(3, 0) = 0.;	cov(3, 1) = 0.;	cov(3, 2) = 0.;			cov(3, 3) = 1.;

	_filter->init_kalman(state, cov);
	update(x, y, distance);
}

void KalmanStates::predict(){
	_filter->predict(*predict_model);
}

void KalmanStates::predict(double& x, double& y, double& vx, double& vy){
	predict();

	x = _filter->x[0];
	y = _filter->x[1];
	vx = _filter->x[2];
	vy = _filter->x[3];
}

void KalmanStates::update(){
	_filter->update();
}

void KalmanStates::update(double x, double y, double distance){
	Vec observation(2);

	observation[0] = x;
	observation[1] = y;

	//printf("%d %f %f %f ", _id, x, y, height);

	observe_model->Zv[0] = _positionVariance + std::pow(distance, 4) * _depthMultiplier;
	observe_model->Zv[1] = _positionVariance + std::pow(distance, 4) * _depthMultiplier;

	_filter->observe(*observe_model, observation);
	_filter->update();
}

void KalmanStates::getMahalanobisParameters(MahalanobisParameters& mp){
	mp.matrix = _filter->SI;
	mp.x = _filter->x[0];
	mp.y = _filter->x[1];
}

double KalmanStates::performMahalanobisDistance(double x, double y, const MahalanobisParameters& mp){
	Vec v(2);
	v[0] = x - mp.x;
	v[1] = y - mp.y;
	return prod_SPDT(v, mp.matrix);
}

SymMatrix KalmanStates::getInnovationCovariance(){
	return _filter->SI;
}

void KalmanStates::getState(double& x, double& y, double& vx, double& vy){
	x = _filter->x[0];
	y = _filter->x[1];
	vx = _filter->x[2];
	vy = _filter->x[3];
}

void KalmanStates::getState(double& x, double& y){
	x = _filter->x[0];
	y = _filter->x[1];
}

