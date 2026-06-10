#pragma once

#include <vector>
#include <eigen3/Eigen/Core>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct FittedLine {
	bool k_is_exist = false;
	float k_ = 0.0f;
	float b_ = 0.0f;
	std::vector<Eigen::Vector3f> points_;
};

class LineFitting {
public:

	LineFitting(int min_line_length, int line_length_threshold);
	void runLineExtraction(const std::vector<Eigen::Vector3f>& cloud);
	bool hasDegradation() const { return is_degradation_; }
	std::vector<FittedLine> getLines() const { return lines_; }
	int getLinesSize() const { return lines_.size(); }

private:

	int min_line_length_;
	int line_length_threshold_;
	std::vector<std::vector<Eigen::Vector3f>> clusters_;
	std::vector<FittedLine> lines_;
	bool is_degradation_ = true;

private:

	bool checkDegradationByLineAngles();

	void removePointsNearOrigin(std::vector<Eigen::Vector3f>& cloud, double radius);

	double distance(const Eigen::Vector3f& point_1, const Eigen::Vector3f& point_2) const;
	std::vector<int> regionQuery(int idx, const std::vector<Eigen::Vector3f>& points) const;
	void dbscan(const std::vector<Eigen::Vector3f>& points, std::vector<int>& labels);
	void extractClusters(const std::vector<Eigen::Vector3f>& points, const std::vector<int>& labels);

	void fitLines(std::vector<Eigen::Vector3f>& cloud);
	void sortClusterAlongPrincipalAxis(std::vector<Eigen::Vector3f>& cluster);
	void markIndexDetection(const std::vector<Eigen::Vector3f>& pointcloud, std::vector<int>& mark_index) const;

	bool fittingLine2D(const std::vector<Eigen::Vector3f>& data, double& k, double& b, bool& k_exist) const;

};