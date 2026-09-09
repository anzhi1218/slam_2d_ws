#pragma once

#include <vector>
#include <unordered_map>
#include <eigen3/Eigen/Dense>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

struct FittedLine {
	bool k_is_exist = false;
	float k_ = 0.0f;
	float b_ = 0.0f;
	Eigen::Vector2f p0_;
	Eigen::Vector2f p1_;
	std::vector<Eigen::Vector2f> points_;
};

class LineFitting {
public:

	LineFitting(int min_line_length, int line_length_threshold);

	void runLineExtraction(const std::vector<Eigen::Vector2f>& cloud);
	bool hasDegradation() const { return is_degradation_; }

	const std::vector<FittedLine>& getLines() const { return lines_; }
	int getLinesSize() const { return static_cast<int>(lines_.size()); }

private:

	int min_line_length_;
	int line_length_threshold_;
	std::vector<std::vector<Eigen::Vector2f>> clusters_;
	std::vector<FittedLine> lines_;
	bool is_degradation_ = true;

	struct GridKey {
		int x;
		int y;
		bool operator==(const GridKey& other) const { return x == other.x && y == other.y; }
	};

	struct GridKeyHash {
		std::size_t operator()(const GridKey& key) const {
			return static_cast<std::size_t>(key.x) ^ (static_cast<std::size_t>(key.y) << 16);
		}
	};

	std::unordered_map<GridKey, std::vector<int>, GridKeyHash> grid_;
	std::vector<GridKey> nearby_offsets_;

	std::vector<int> seedQueue;
	std::vector<int> region_query_;
	const float eps_ = 0.17f;
	float center_x_ = 0.0f;
	float center_y_ = 0.0f;

	bool checkDegradationByLineAngles();
	void removePointsNearOrigin(std::vector<Eigen::Vector2f>& cloud);
	void initNearbyOffsets();
	void buildGrid(const std::vector<Eigen::Vector2f>& points);
	void regionQuery(int idx, const std::vector<Eigen::Vector2f>& points, std::vector<int>& neighbors);
	void dbscan(const std::vector<Eigen::Vector2f>& points, std::vector<int>& labels);
	void extractClusters(const std::vector<Eigen::Vector2f>& points, const std::vector<int>& labels);

	void fitLines(std::vector<Eigen::Vector2f>& points_cloud);
	void sortClusterAlongPrincipalAxis(std::vector<Eigen::Vector2f>& cluster);
	void markIndexDetection(const std::vector<Eigen::Vector2f>& pointcloud, std::vector<int>& mark_index) const;
	bool fittingLine2D(const std::vector<Eigen::Vector2f>& data, float& k, float& b, bool& k_exist) const;
	void pointToLineSubpoint(float point_x, float point_y, float p1_x, float p1_y,
		float p2_x, float p2_y, float& line_pt_x, float& line_pt_y);
};