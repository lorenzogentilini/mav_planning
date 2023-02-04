#pragma once
#include <Eigen/Eigen>
#include <iostream>
#include <fstream>
#include <cctype>
#include <cmath>
#include <cfloat>

int findLastNonZeroCoeff(Eigen::VectorXd coefficients);
bool findRootsJenkinsTraub(Eigen::VectorXd coefficients_increasing, Eigen::VectorXcd& roots);