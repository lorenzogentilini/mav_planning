#include <mav_planning/containers.hpp>

// Constructor
Polynomial::Polynomial(int _poly_order): poly_coeff(_poly_order+1){
	poly_coeff.setZero();
	poly_order = _poly_order;
	base_coeff = computeBaseCoefficients();
}

Polynomial::Polynomial(Eigen::VectorXd coeffs): poly_coeff(coeffs){ 
	poly_order = poly_coeff.size()-1;
	base_coeff = computeBaseCoefficients();
}

Polynomial::Polynomial(int _poly_order, double st, double et, std::vector<double> stConstraints, std::vector<double> etConstraints):
	poly_coeff(_poly_order+1){

	poly_order = _poly_order;
	base_coeff = computeBaseCoefficients();

	Eigen::MatrixXd A, b;
	A.resize(poly_order+1, poly_order+1);
	b.resize(poly_order+1, 1);

	A.setZero();
	b.setZero();
	
	for(uint i = 0; i < (poly_order+1)/2; i++)
		A.row(i) = getBaseWithTime(i, st);

	for(uint i = 0; i < (poly_order+1)/2; i++)
		A.row(i + (poly_order+1)/2) = getBaseWithTime(i, et);

	for(uint i = 0; i < stConstraints.size(); i++)
		b(i) = stConstraints[i];

	for(uint i = 0; i < etConstraints.size(); i++)
		b(i + (poly_order+1)/2) = etConstraints[i];

	poly_coeff = A.fullPivHouseholderQr().solve(b);
}

Polynomial::~Polynomial(){
	;
}

void Polynomial::setCoefficients(Eigen::VectorXd coeffs){
	if(coeffs.size() != poly_order+1){
		ROS_ERROR("[POLY]: Order does't Match");
		exit(-1);
	}

	poly_coeff = coeffs;
}

Eigen::MatrixXd Polynomial::computeBaseCoefficients(){
	Eigen::MatrixXd coefficients(poly_order+1, poly_order+1);

	coefficients.setZero();
	coefficients.row(0).setOnes();

	int degree = poly_order;
	int order = degree;
	for(int n = 1; n < poly_order+1; n++){
		for(int i = degree - order; i < poly_order+1; i++){
			coefficients(n, i) = (order - degree + i)*coefficients(n - 1, i);
		}
		order--;
	}

	return coefficients;
}

bool Polynomial::computeMinMaxCandidates(double st, double et, int dd, std::vector<double>& candidates){
	Eigen::VectorXcd roots;
	bool success = getRoots(dd + 1, roots);

	if(!success){
		ROS_WARN("[POLY]: Not Able to Find Roots");
		return false;
	} else if(!selectCandidatesFromRoots(st, et, roots, candidates)){
		ROS_WARN("[POLY]: Not Able to Select Candidates");
		return false;
	}

	return true;
}

bool Polynomial::getRoots(int dd, Eigen::VectorXcd& roots){
	return findRootsJenkinsTraub(getCoefficients(dd), roots);
}

bool Polynomial::selectCandidatesFromRoots(double st, double et, Eigen::VectorXcd roots, std::vector<double>& candidates){
	if(st > et){
		ROS_WARN("[POLY]: Initial Time Greater Than Final");
		return false;
	}

	// Start and End Times are Valid Candidates
	candidates.reserve(roots.size() + 2);
	candidates.push_back(st);
	candidates.push_back(et);

	for(uint i = 0; i < roots.size(); i++){
		// Consider Only Real Roots
		if(abs(roots(i).imag()) > std::numeric_limits<double>::epsilon())
			continue;

		double candidate = roots(i).real();
		if(candidate > st && candidate < et)
			candidates.push_back(candidate);
	}

	return true;
}

bool Polynomial::selectMinMaxFromCandidates(std::vector<double>& candidates, int dd, std::pair<double, double>& min, std::pair<double, double>& max){
	if(candidates.empty()){
		ROS_WARN("[POLY]: Cannot Find Extrema");
		return false;
	}

	min.first = candidates[0];
	min.second = std::numeric_limits<double>::max();
	max.first = candidates[0];
	max.second = std::numeric_limits<double>::lowest();

	for(std::vector<double>::iterator iter = candidates.begin();
			iter != candidates.end(); iter++){
			
		double actualValue = evaluate(dd, *iter);
		if(actualValue < min.second){
			min.first = *iter;
			min.second = actualValue;
		}
		
		if(actualValue > max.second){
			max.first = *iter;
			max.second = actualValue;
		}
	}

	return true;
}

Eigen::VectorXd Polynomial::getCoefficients(uint dd){
	Eigen::VectorXd result;
	result.setZero();

	if(dd > poly_order || dd < 0)
		return result;

	if(dd == 0)
		return poly_coeff;

	result = base_coeff.row(dd).cwiseProduct(poly_coeff.transpose());
	return result;
}

double Polynomial::getBaseCoefficient(int dd, int idx){
	if(dd > poly_order || dd < 0){
		ROS_WARN("[POLY]: Order Not Compliant");
		return 0.0;
	}

	return base_coeff(dd, idx);
}

Eigen::VectorXd Polynomial::getBaseWithTime(uint dd, double tt){
	Eigen::VectorXd result(poly_order+1);
	result.setZero();

	if(dd > poly_order)
		return result;

	double timePow = tt;
	result(dd) = base_coeff(dd, dd);
	for(int i = dd+1; i < poly_order+1; i++){
		result(i) = base_coeff(dd, i)*timePow;
		timePow = timePow*tt;
	}

	return result;
}

double Polynomial::evaluate(uint dd, double tt){
	if(dd > poly_order)
		return 0.0;

	return poly_coeff.dot(getBaseWithTime(dd, tt));
}

bool Polynomial::computeMinMax(double st, double et, int dd, std::pair<double, double>& min, std::pair<double, double>& max){
	std::vector<double> candidates;
	if(!computeMinMaxCandidates(st, et, dd, candidates))
		return false;

	// Evaluate Minimum and Maximum.
	return selectMinMaxFromCandidates(candidates, dd, min, max);
}

Eigen::VectorXd Polynomial::convolve(uint dd, uint kernel){
	Eigen::VectorXd d = getCoefficients(dd).tail(poly_order + 1 - dd);
	Eigen::VectorXd d_ = getCoefficients(kernel).tail(poly_order + 1 - kernel);

	int convolutionDimension = d_.size() + d.size() - 1;

	Eigen::VectorXd convolved = Eigen::VectorXd::Zero(convolutionDimension);
	Eigen::VectorXd kernelReverse = d_.reverse();

	for(uint i = 0; i < convolutionDimension; i++){
		int dataIDX = i - d_.size() + 1;

		int lowerBound = std::max(0, -dataIDX);
		int upperBound = std::min(d_.size(), d.size() - dataIDX);

		for (int kernelIDX = lowerBound; kernelIDX < upperBound; ++kernelIDX) {
			convolved[i] += kernelReverse[kernelIDX]*d[dataIDX + kernelIDX];
		}
	}

	return convolved;
}