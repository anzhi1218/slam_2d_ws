#pragma once

#include <cstddef>
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
	std::vector<Eigen::Vector2f> points_;
	bool fit_success_ = false;
	float fit_rms_error_ = 0.0f;
	float fit_max_error_ = 0.0f;
};

class LineFitting {
public:

	LineFitting(int min_line_length, int line_length_threshold);

	void runLineExtraction(const std::vector<Eigen::Vector2f>& cloud);
	bool hasDegradation() const { return is_degradation_; }

	const std::vector<FittedLine>& getLines() const { return lines_; }
	int getLinesSize() const { return static_cast<int>(lines_.size()); }
	int getValidLineCount() const;

private:

	int min_line_length_;
	int line_length_threshold_;
	std::vector<std::vector<Eigen::Vector2f>> clusters_;
	std::vector<std::vector<int>> cluster_scan_indices_;
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
	std::vector<float> point_eps_;

	// 自适应邻域半径: eps = clamp(eps_min_ * range / eps_reference_range_, eps_min_, eps_max_)
	const float eps_min_ = 0.25f;
	const float eps_max_ = 0.65f;
	const float eps_reference_range_ = 5.0f;

	bool checkDegradationByLineAngles();
	void removePointsNearOrigin(const std::vector<Eigen::Vector2f>& cloud,
		std::vector<int>& scan_indices);
	void initNearbyOffsets();
	void buildGrid(const std::vector<Eigen::Vector2f>& cloud,
		const std::vector<int>& scan_indices);
	void regionQuery(int idx, const std::vector<Eigen::Vector2f>& cloud,
		const std::vector<int>& scan_indices, std::vector<int>& neighbors);
	void dbscan(const std::vector<Eigen::Vector2f>& cloud,
		const std::vector<int>& scan_indices, std::vector<int>& labels);
	void extractClusters(const std::vector<Eigen::Vector2f>& cloud,
		const std::vector<int>& labels, const std::vector<int>& scan_indices);

	void fitLines(const std::vector<Eigen::Vector2f>& points_cloud,
		const std::vector<int>& scan_indices);
	void updateLineFitMetrics(FittedLine& line) const;
	void markIndexDetection(const std::vector<Eigen::Vector2f>& pointcloud,
		const std::vector<int>& scan_indices, std::vector<int>& mark_index) const;
	bool fittingLine2D(const std::vector<Eigen::Vector2f>& cloud, int first_index, int last_index,
		float& k, float& b, bool& k_exist) const;
};
