#include "rev/api/motionplanning/planning/spline/spline.hpp"
#include <iostream>

void Spline::parameterize() {
    double totalLength = nodes[nodes.size() - 1]->cumulativeDistance;

    for (const RRT::Node* node : nodes) {
        t.push_back(node->cumulativeDistance/totalLength);
    }
}

void Spline::calculateKnots() {
    double n = nodes.size() - 1;

    for (int i = 0; i <= degree; i++) {
        knots[i] = 0.0;
    }

    for (int j = 1; j <= n - degree; j++) {
        double sum = 0;

        for (int i = 0; i < degree; i++) {
            sum += t[i + j];
        }

        knots[j + degree] = sum / degree;
    }

    for (int i = n + 1; i <= n + degree + 1; i++) {
        knots[i] = 1.0;
    }
}

double Spline::N(int i, double t, unsigned int p) const {
    int n = nodes.size() - 1;
    if (p == 0) {
        if (t >= knots[i] && t < knots[i+1]) return 1.0;
        // For the very end (last node), ensure last basis function is active at t==knots.back()
        if (i == n && std::abs(t - knots.back()) < 1e-10) return 1.0;
        return 0.0;
    }

    double leftTerm = 0;
    double rightTerm = 0;

    double leftDenom = knots[i + p] - knots[i];
    if (leftDenom != 0) {
        leftTerm += (t - knots[i]) / leftDenom * N(i, t, p-1);
    }

    double rightDenom = knots[i + p + 1] - knots[i + 1];
    if (rightDenom != 0) {
        rightTerm += (knots[i + p + 1] - t) / rightDenom * N(i+1, t, p-1);
    }

    return leftTerm + rightTerm;
}

void Spline::calculateControlPoints(double smoothingFactor) {
    int n = nodes.size() - 1;

    Eigen::MatrixXd Nmat(n+1, n+1);
    Eigen::VectorXd Px(n+1), Py(n+1);

    for (int i = 0; i <= n; i++) {
        Px(i) = nodes[i]->point.x();
        Py(i) = nodes[i]->point.y();
    }

    for (int i = 0; i <= n; i++) {
        for (int j = 0; j <= n; j++) {
            Nmat(i, j) = N(j, t[i], degree);
        }
    }

    Eigen::MatrixXd A = Nmat.transpose() * Nmat + smoothingFactor * Eigen::MatrixXd::Identity(n+1, n+1);
    Eigen::VectorXd bx = Nmat.transpose() * Px;
    Eigen::VectorXd by = Nmat.transpose() * Py;

    // normalization factor: average diagonal
    double scale = A.trace() / static_cast<double>(n + 1);
    double lambda = smoothingFactor * scale; // normalized smoothing factor

    // Optional safety: clamp alpha to reasonable bounds
    // alpha = std::clamp(alpha, 1e-8, 1.0); // if desired

    Eigen::MatrixXd Reg = A + lambda * Eigen::MatrixXd::Identity(n+1, n+1);

    Eigen::VectorXd ctrlX = Reg.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd ctrlY = Reg.colPivHouseholderQr().solve(by);

    controlPoints.resize(n+1);
    for (int i = 0; i <= n; i++) {
        controlPoints[i] = Eigen::Vector2d(ctrlX(i), ctrlY(i));
    }
}

void Spline::calculateDerivativeControlPoints() {
    int n = controlPoints.size() - 1;

    derivativeControlPoints.resize(n);
    for (int i = 0; i < n; i++) {
        double denom = knots[i + degree] - knots[i];
        double coeff = (denom != 0.0) ? (degree / denom) : 0.0;
        derivativeControlPoints[i] = coeff * (controlPoints[i + 1] - controlPoints[i]);
    }
}

void Spline::calculateSecondDerivativeControlPoints() {
    int n = derivativeControlPoints.size() - 1;

    secondDerivativeControlPoints.resize(n);
    for (int i = 0; i < n; i++) {
        double denom = knots[i + degree + 1] - knots[i + 2];
        double coeff = (denom != 0.0) ? (degree / denom) : 0.0;
        secondDerivativeControlPoints.push_back(coeff * (derivativeControlPoints[i + 1] - derivativeControlPoints[i]));
    }
}

Eigen::Vector2d Spline::calculateAtT(double t) const {
    Eigen::Vector2d point(0,0);

    int n = controlPoints.size() - 1;
    for (int i = 0; i <= n; i++) {
        point += N(i, t, degree) * controlPoints[i];
    }

    return point;
}

Eigen::Vector2d Spline::calculateDerivativeAtT(double t) const {
    Eigen::Vector2d point(0,0);

    for (int i = 0; i < derivativeControlPoints.size(); i++) {
        point += N(i + 1, t, degree - 1) * derivativeControlPoints[i];
    }

    return point;
}

Eigen::Vector2d Spline::calculateSecondDerivativeAtT(double t) const {
    Eigen::Vector2d point(0,0);

    for (int i = 0; i < secondDerivativeControlPoints.size(); i++) {
        point += N(i, t, degree - 2) * secondDerivativeControlPoints[i];
    }

    return point;
}

std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> Spline::sampleSplineT(int numSamples) const {
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> sampledCurve;
    if (controlPoints.empty()) return sampledCurve;

    double tStart = knots[degree];
    double tEnd = knots[knots.size() - degree - 1];
    double dt = (tEnd - tStart) / (numSamples - 1);

    sampledCurve.reserve(numSamples);

    for (int s = 0; s < numSamples; s++) {
        double param = tStart + s * dt;
        if (param > tEnd) param = tEnd - 1e-9;

        sampledCurve.push_back(calculateAtT(param));
    }

    return sampledCurve;
}

std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> Spline::sampleSpline(int numSamples) const {
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> sampledCurve;
    if (controlPoints.empty()) return sampledCurve;

    double tStart = knots[degree];
    double tEnd = knots[knots.size() - degree - 1];

    double sStart = 0;
    double sEnd = arcLengthTable.maxS;
    double dS = (sEnd - sStart) / (numSamples - 1);

    sampledCurve.reserve(numSamples);

    for (int i = 0; i < numSamples; i++) {
        double param = sStart + i * dS;
        if (param > sEnd) param = sEnd - 1e-9;

        sampledCurve.push_back(calculateAt(param));
    }

    return sampledCurve;
}

double Spline::nearestS(Eigen::Vector2d point, int numSamples) const {
    // Coarsely sample curve
    auto sampledCurve = sampleSpline(numSamples);

    double sStart = 0;
    double sEnd = arcLengthTable.maxS;
    double dS = (sEnd - sStart) / (numSamples - 1);

    double bestS = 0;
    double bestSquaredDist = 1e9;
    {
        double s = 0;
        for (const Eigen::Vector2d &vec : sampledCurve) {
            double dist = (point - vec).squaredNorm();
            if (dist < bestSquaredDist) {
                bestS = s;
                bestSquaredDist = dist;
            }
            s += dS;
        }
    }

    // // Newton-Rhapson
    // double s = bestS;
    // double epsilon = 1e-6;
    // int maxIter = 15;

    // while (maxIter-- > 0) {
    //     Eigen::Vector2d Ct = calculateAt(s);
    //     Eigen::Vector2d Cdt = calculateDerivativeAt(s);
    //     Eigen::Vector2d Cddt = calculateSecondDerivativeAt(s);
    //     double distanceDerivative      = 2 * (Ct - point).dot(Cdt);
    //     double distanceSecondDerivative = 2 * (Cdt.dot(Cdt) + (Ct - point).dot(Cddt));

    //     if (std::abs(distanceSecondDerivative) < 1e-8 || std::isnan(distanceSecondDerivative)) {
    //         break; // Can't safely step
    //     }
    //     double sNext = s - distanceDerivative / distanceSecondDerivative;

    //     // Clamp t to curve parameter range
    //     s = sEnd ? sNext > sEnd : sNext;
    //     s = sStart ? sNext < sStart : sNext;

    //     // Optional: fast exit if already close
    //     if (std::abs(distanceDerivative) < epsilon) {
    //         break;
    //     }
    // }

    return bestS;
}

void Spline::generateArcLengthMapping(int numSamples) {
    double tStart = knots[degree];
    double tEnd = knots[knots.size() - degree - 1];
    double dt = (tEnd - tStart) / (numSamples - 1);

    std::vector<double> tSamples;

    auto points = sampleSplineT(numSamples);
    tSamples.resize(points.size());

    tSamples[0] = 0;
    for (size_t i = 1; i < points.size(); i++) {
        tSamples[i] = tSamples[i-1] + dt;
    }

    arcLengthTable.generate(tSamples, points);
}