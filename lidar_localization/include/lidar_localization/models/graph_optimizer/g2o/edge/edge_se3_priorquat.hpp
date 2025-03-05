/*
 * @Description: 姿态的先验边
 * @Author: Ren Qian
 * @Date: 2020-03-01 18:05:35
 */
#ifndef LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_SE3_PRIORQUAT_HPP_
#define LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_SE3_PRIORQUAT_HPP_

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace g2o {
class EdgeSE3PriorQuat : public g2o::BaseUnaryEdge<3, Eigen::Quaterniond, g2o::VertexSE3> {
  public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	EdgeSE3PriorQuat()
      :g2o::BaseUnaryEdge<3, Eigen::Quaterniond, g2o::VertexSE3>() {
	}

	void computeError() override {
		const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
		//.linear() 提取了其中的旋转矩阵部分
		Eigen::Quaterniond estimate = Eigen::Quaterniond(v1->estimate().linear());
		/*
		estimate 是一个 Eigen::Quaterniond 对象，代表一个四元数。
		.coeffs() 是 Eigen 库中 Quaternion 类的一个成员函数。
		它返回一个 Eigen::Vector4d，其中包含了四元数的虚部 (x, y, z) 
		和实部 (w)。注意顺序是 x, y, z, w，而不是 w, x, y, z。 这是 
		Eigen 库四元数的存储约定。
		

		确保四元数的实部 w 为正数，从而消除四元数表示旋转的二义性。
		四元数的二义性: 对于任何一个旋转，都可以用两个互为相反数的四元数来表示，
		即 q 和 -q 代表相同的旋转。 这是因为四元数表示旋转时，单位四元数 q 和 
		-q 对应的旋转矩阵是相同的。 这个性质会导致在优化过程中，四元数的值在正
		负号之间跳变，从而影响优化的稳定性和收敛速度。

		消除二义性的方法: 为了消除这种二义性，通常约定四元数的实部 w 为正数。 
		也就是说，如果 w 是负数，我们就将四元数的所有分量取反，得到一个等价
		的四元数，其 w 值为正数。

		*/
		if(estimate.w() < 0) {
			estimate.coeffs() = -estimate.coeffs();
		}
		/*
		这里计算的是四元数虚部向量的差异。 
		estimate.vec() 返回一个 Eigen::Vector3d，
		包含四元数的 x, y, z 分量。 因为四元数模长为 1，
		只使用虚部也能表达旋转的差异。
		*/
		_error = estimate.vec() - _measurement.vec();
	}

	void setMeasurement(const Eigen::Quaterniond& m) override {
		_measurement = m;
		if(m.w() < 0.0) {
			_measurement.coeffs() = -m.coeffs();
		}
	}

	virtual bool read(std::istream& is) override {
		Eigen::Quaterniond q;
		is >> q.w() >> q.x() >> q.y() >> q.z();
		setMeasurement(q);
		for (int i = 0; i < information().rows(); ++i)
			for (int j = i; j < information().cols(); ++j) {
				is >> information()(i, j);
				if (i != j)
					information()(j, i) = information()(i, j);
			}
		return true;
	}

	virtual bool write(std::ostream& os) const override {
		Eigen::Quaterniond q = _measurement;
		os << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
		for (int i = 0; i < information().rows(); ++i)
			for (int j = i; j < information().cols(); ++j)
				os << " " << information()(i, j);
		return os.good();
	}
};
}

#endif
