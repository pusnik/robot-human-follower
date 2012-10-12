#include "kalmanModel.cpp"
#include "boost_1_49_0/boost/numeric/ublas/io.hpp"
#include <BayesFilter/unsFlt.hpp>

using namespace Bayesian_filter;
using namespace Bayesian_filter_matrix;

class KalmanStates
{
protected:

	double _dt;
	double _positionVariance;
	double _depthMultiplier;
	double _accelerationVariance;

	Unscented_scheme* _filter;

	Human_predict* predict_model;
	Human_observe* observe_model;

public:
	KalmanStates(double dt, double positionVariance, double accelerationVariance);
	KalmanStates(const KalmanStates& orig);
	KalmanStates& operator=(const KalmanStates& orig);
	virtual ~KalmanStates();

	void init(double x, double y, double distance);
	void predict();
	void predict(double& x, double& y, double& vx, double& vy);
	void update();
	void update(double x, double y, double distance);
	void getState(double& x, double& y, double& vx, double& vy);
	void getState(double& x, double& y);
	void getMahalanobisParameters(MahalanobisParameters& mp);
	static double performMahalanobisDistance(double x, double y, const MahalanobisParameters& mp);

	SymMatrix getInnovationCovariance();

};

