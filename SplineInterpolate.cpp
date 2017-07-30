#include "SplineInterpolate.h"

#include <cassert>
#include <tuple>

void SplineInterpolate::start(double t, double x, double y, double vx, double vy)
{
	xvec.push_back(x);
	yvec.push_back(y);
	vxvec.push_back(vx);
	vyvec.push_back(vy);
	tvec.push_back(t);
}

void SplineInterpolate::end(double t, double x, double y, double vx, double vy)
{
	xvec.push_back(x);
	yvec.push_back(y);
	tvec.push_back(t);
	//i=0
	a.push_back(0);
	b.push_back(1);
	c.push_back(0);
	//i=1~n-1
	for (int i = 1; i < (int)xvec.size() - 1; i++) {
		double ph = tvec[i] - tvec[i-1];
		double h = tvec[i + 1] - tvec[i];
		a.push_back(h / (ph + h));
		b.push_back(2);
		c.push_back(ph / (ph + h));
		vxvec.push_back((ph*(xvec[i + 1] - xvec[i]) / h + h*(xvec[i] - xvec[i - 1]) / ph) / (ph + h));
		vyvec.push_back((ph*(yvec[i + 1] - yvec[i]) / h + h*(yvec[i] - yvec[i - 1]) / ph) / (ph + h));
	}
	//i=n
	a.push_back(0);
	b.push_back(1);
	c.push_back(0);
	vxvec.push_back(vx);
	vyvec.push_back(vy);

	//solve
	for (int i = 0; i < (int)xvec.size() - 1; i++) {
		vxvec[i] /= b[i];
		vyvec[i] /= b[i];
		c[i] /= b[i];

		b[i + 1] -= a[i + 1] * c[i];
		vxvec[i + 1] -= a[i + 1] * vxvec[i];
		vyvec[i + 1] -= a[i + 1] * vyvec[i];
	}
	for (int i = xvec.size() - 1; i > 0; i--) {
		vxvec[i - 1] -= c[i - 1] * vxvec[i];
		vyvec[i - 1] -= c[i - 1] * vyvec[i];
	}
}

void SplineInterpolate::append(double t, double x, double y)
{
	xvec.push_back(x);
	yvec.push_back(y);
	tvec.push_back(t);
}

std::vector<std::tuple<double, double>> SplineInterpolate::get(const std::vector<double> &tt) const
{
	int j = 0;
	std::vector<std::tuple<double, double>> ans;
	for (auto t : tt) {
		while (t > tvec[j] && j < (int)tvec.size()) j++;
		assert(j > 0 && j < (int)tvec.size());
		double h = tvec[j] - tvec[j-1];
		double dpt = t - tvec[j - 1];
		double dt = t - tvec[j];
		ans.emplace_back(
			(vxvec[j - 1] * dpt * dt * dt +
				vxvec[j] * dpt * dpt * dt) / h / h +
			(xvec[j - 1] * dt * dt * (2 * dpt + h) +
				xvec[j] * dpt * dpt * (-2 * dt + h)) / h / h / h, 
			(vyvec[j - 1] * dpt * dt * dt +
				vyvec[j] * dpt * dpt * dt) / h / h +
			(yvec[j - 1] * dt * dt * (2 * dpt + h) +
				yvec[j] * dpt * dpt * (-2 * dt + h)) / h / h / h
		);
	}
	return ans;
}
