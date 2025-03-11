/*
 * @Description: GNSS 坐标做观测时使用的先验边
 * @Author: Ren Qian
 * @Date: 2020-03-01 18:05:35
 */
#ifndef LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_SE3_PRIORXYZ_HPP_
#define LIDAR_LOCALIZATION_MODELS_GRAPH_OPTIMIZER_G2O_EDGE_EDGE_SE3_PRIORXYZ_HPP_

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace g2o {
	/*
	1、第一个参数：3
	误差维度：表示该边的误差是一个三维向量（3个分量）。
	在这个例子中，误差计算会涉及三个维度（x, y, z）。
	
	2、第二个参数：Eigen::Vector3d
	测量值类型：说明该边的测量值是 Eigen::Vector3d 类型，
	即一个三维双精度浮点向量。这通常表示三维空间中的一个点或位置。
	
	3、第三个参数：g2o::VertexSE3
	顶点类型：表示该边连接到一个 VertexSE3 类型的顶点。
	VertexSE3 表示三维空间中的刚体变换（位置和旋转），
	包含六个自由度（三个位置，三个旋转）。
	*/
class EdgeSE3PriorXYZ : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3> {
  public:
  /*
	什么时候需要使用它？
	1、自定义的 Eigen 数据结构：如果你定义了一个继承自 
	Eigen::Matrix 或 Eigen::Array 的类，必须 在类中添加这个宏。
	2、包含 Eigen 对象的类：如果你定义了一个类，其中包含 Eigen 
	对象作为成员变量，也需要添加这个宏，确保整个类的内存对齐。
 */
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	/*
	这样的调用是C++继承机制中的标准做法，特别是在处理模板类和通用基础结构时。
	通过调用父类的构造函数，子类确保了继承自父类的部分被正确地初始化，
	这对于子类的功能和行为是必要的。
	*/
	EdgeSE3PriorXYZ()
    	:g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3>() {
	}

	void computeError() override {
		const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);

		Eigen::Vector3d estimate = v1->estimate().translation();
		_error = estimate - _measurement;
	}

    void setMeasurement(const Eigen::Vector3d& m) override {
		_measurement = m;
	}

	virtual bool read(std::istream& is) override {
    	Eigen::Vector3d v;
		is >> v(0) >> v(1) >> v(2);

    	setMeasurement(Eigen::Vector3d(v));

		for (int i = 0; i < information().rows(); ++i)
			for (int j = i; j < information().cols(); ++j) {
				is >> information()(i, j);
				if (i != j)
					information()(j, i) = information()(i, j);
			}
			return true;
		}

	virtual bool write(std::ostream& os) const override {
    	Eigen::Vector3d v = _measurement;
		os << v(0) << " " << v(1) << " " << v(2) << " ";
		for (int i = 0; i < information().rows(); ++i)
			for (int j = i; j < information().cols(); ++j)
				os << " " << information()(i, j);
		return os.good();
	}
};
}

#endif
