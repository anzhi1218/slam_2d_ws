#include "linear_fitting.h"
#include <algorithm>
#include <cmath>
#include <utility>

LineFitting::LineFitting(int min_line_length, int line_length_threshold)
	: min_line_length_(min_line_length), line_length_threshold_(line_length_threshold) {
	initNearbyOffsets();
}

void LineFitting::initNearbyOffsets() {
	nearby_offsets_.clear();
	nearby_offsets_.reserve(9);
	for (int dx = -1; dx <= 1; ++dx) {
		for (int dy = -1; dy <= 1; ++dy) {
			nearby_offsets_.push_back({ dx, dy });
		}
	}
}

void LineFitting::buildGrid(const std::vector<Eigen::Vector2f>& cloud,
	const std::vector<int>& scan_indices) {
	grid_.clear();
	const int n = static_cast<int>(scan_indices.size());
	for (int i = 0; i < n; ++i) {
		const Eigen::Vector2f& point = cloud[scan_indices[i]];
		GridKey key{
			static_cast<int>(std::round(point.x() / eps_max_)),
			static_cast<int>(std::round(point.y() / eps_max_))
		};
		grid_[key].push_back(i);
	}
}

void LineFitting::regionQuery(int idx, const std::vector<Eigen::Vector2f>& cloud,
	const std::vector<int>& scan_indices, std::vector<int>& neighbors) {
	neighbors.clear();
	const Eigen::Vector2f& point = cloud[scan_indices[idx]];
	const float cell_size_inv = 1.0f / eps_max_;
	const int cx = static_cast<int>(std::round(point.x() * cell_size_inv));
	const int cy = static_cast<int>(std::round(point.y() * cell_size_inv));
	const float px = point.x();
	const float py = point.y();
	for (const GridKey& offset : nearby_offsets_) {
		const auto it = grid_.find(GridKey{ cx + offset.x, cy + offset.y });
		if (it == grid_.end()) {
			continue;
		}
		for (int neighbor_index : it->second) {
			if (neighbor_index == idx) {
				neighbors.push_back(neighbor_index);
				continue;
			}
			const Eigen::Vector2f& neighbor = cloud[scan_indices[neighbor_index]];
			const float dx = px - neighbor.x();
			const float dy = py - neighbor.y();
			const float pair_eps = 0.5f * (point_eps_[idx] + point_eps_[neighbor_index]);
			if (dx * dx + dy * dy <= pair_eps * pair_eps) {
				neighbors.push_back(neighbor_index);
			}
		}
	}
}

void LineFitting::removePointsNearOrigin(const std::vector<Eigen::Vector2f>& cloud,
	std::vector<int>& scan_indices) {
	const float radius_sq = 0.2f * 0.2f;
	int write_index = 0;
	for (int read_index = 0; read_index < static_cast<int>(scan_indices.size()); ++read_index) {
		const int source_index = scan_indices[read_index];
		const Eigen::Vector2f& point = cloud[source_index];
		const float distance_sq = point.x() * point.x() + point.y() * point.y();
		if (distance_sq >= radius_sq) {
			if (write_index != read_index) {
				scan_indices[write_index] = source_index;
			}
			++write_index;
		}
	}
	scan_indices.resize(write_index);
}

void LineFitting::runLineExtraction(const std::vector<Eigen::Vector2f>& cloud)
{
	lines_.clear();
	clusters_.clear();
	cluster_scan_indices_.clear();
	is_degradation_ = true;
	seedQueue.clear();
	region_query_.clear();
	point_eps_.clear();
	std::vector<int> scan_indices(cloud.size());
	for (std::size_t i = 0; i < scan_indices.size(); ++i) {
		scan_indices[i] = static_cast<int>(i);
	}
	removePointsNearOrigin(cloud, scan_indices);
	if (scan_indices.size() < static_cast<std::size_t>(min_line_length_)) {
		return;
	}
	std::vector<int> labels;
	dbscan(cloud, scan_indices, labels);
	extractClusters(cloud, labels, scan_indices);
	for (std::size_t i = 0; i < clusters_.size(); ++i) {
		if (clusters_[i].size() >= static_cast<std::size_t>(min_line_length_)) {
			fitLines(clusters_[i], cluster_scan_indices_[i]);
		}
	}
	is_degradation_ = checkDegradationByLineAngles();
}

int LineFitting::getValidLineCount() const {
	int count = 0;
	for (const FittedLine& line : lines_) {
		if (line.fit_success_ &&
			line.points_.size() >= static_cast<size_t>(line_length_threshold_)) {
			++count;
		}
	}
	return count;
}

void LineFitting::dbscan(const std::vector<Eigen::Vector2f>& cloud,
	const std::vector<int>& scan_indices, std::vector<int>& labels) {
	const int n = static_cast<int>(scan_indices.size());
	labels.assign(n, -2);
	int clusterId = 0;
	point_eps_.resize(n);
	for (int i = 0; i < n; ++i) {
		const float range = cloud[scan_indices[i]].norm();
		const float range_scaled_eps = eps_min_ * range / eps_reference_range_;
		point_eps_[i] = (std::min)(eps_max_, (std::max)(eps_min_, range_scaled_eps));
	}
	buildGrid(cloud, scan_indices);
	const int minPts = min_line_length_;
	for (int i = 0; i < n; ++i) {
		if (labels[i] != -2) {
			continue;
		}
		regionQuery(i, cloud, scan_indices, region_query_);
		if (static_cast<int>(region_query_.size()) < minPts) {
			labels[i] = -1;
			continue;
		}
		labels[i] = clusterId;
		seedQueue.clear();
		for (int neighbor : region_query_) {
			if (labels[neighbor] == -2) {
				labels[neighbor] = clusterId;
				seedQueue.push_back(neighbor);
			}
			else if (labels[neighbor] == -1) {
				labels[neighbor] = clusterId;
			}
		}
		for (std::size_t j = 0; j < seedQueue.size(); ++j) {
			regionQuery(seedQueue[j], cloud, scan_indices, region_query_);
			if (static_cast<int>(region_query_.size()) >= minPts) {
				for (int neighbor : region_query_) {
					if (labels[neighbor] == -2) {
						labels[neighbor] = clusterId;
						seedQueue.push_back(neighbor);
					}
					else if (labels[neighbor] == -1) {
						labels[neighbor] = clusterId;
					}
				}
			}
		}
		++clusterId;
	}
	grid_.clear();
}

void LineFitting::extractClusters(const std::vector<Eigen::Vector2f>& cloud,
	const std::vector<int>& labels, const std::vector<int>& scan_indices) {
	int max_label = -1;
	for (int label : labels) {
		max_label = (std::max)(max_label, label);
	}
	if (max_label < 0) {
		return;
	}
	clusters_.resize(max_label + 1);
	cluster_scan_indices_.resize(max_label + 1);
	for (int i = 0; i < static_cast<int>(scan_indices.size()); ++i) {
		const int label = labels[i];
		if (label >= 0) {
			const int source_index = scan_indices[i];
			clusters_[label].push_back(cloud[source_index]);
			cluster_scan_indices_[label].push_back(source_index);
		}
	}
}

void LineFitting::fitLines(const std::vector<Eigen::Vector2f>& points_cloud,
	const std::vector<int>& scan_indices) {
	if (points_cloud.size() < static_cast<size_t>(min_line_length_) ||
		points_cloud.size() != scan_indices.size()) {
		return;
	}
	auto appendSegment = [this, &points_cloud, &scan_indices](
		int first_index, int last_index) -> bool {
		if (first_index < 0 || last_index < first_index ||
			last_index >= static_cast<int>(points_cloud.size())) {
			return false;
		}
		const int segment_point_count = last_index - first_index + 1;
		const int required_point_count = (std::max)(min_line_length_, line_length_threshold_);
		if (segment_point_count < required_point_count) {
			return false;
		}
		for (int i = first_index + 1; i <= last_index; ++i) {
			if (scan_indices[i] != scan_indices[i - 1] + 1) {
				return false;
			}
		}
		float k = 0.0f, b = 0.0f;
		bool k_exist = false;
		if (!fittingLine2D(points_cloud, first_index, last_index, k, b, k_exist)) {
			return false;
		}
		FittedLine line;
		line.k_ = k;
		line.b_ = b;
		line.k_is_exist = k_exist;
		line.points_.assign(
			points_cloud.begin() + first_index, points_cloud.begin() + last_index + 1);
		line.fit_success_ = true;
		updateLineFitMetrics(line);
		lines_.push_back(std::move(line));
		return true;
	};
	// A cluster with scan gaps must not be fitted as one continuous surface.
	const float max_cluster_rms_error = 0.05f;
	const float max_cluster_max_error = 0.12f;
	if (appendSegment(0, static_cast<int>(points_cloud.size()) - 1)) {
		const FittedLine& whole_cluster_line = lines_.back();
		if (whole_cluster_line.fit_rms_error_ <= max_cluster_rms_error &&
			whole_cluster_line.fit_max_error_ <= max_cluster_max_error) {
			return;
		}
		lines_.pop_back();
	}
	// Split at geometric corners and at gaps in original scan indices.
	std::vector<int> mark_index;
	markIndexDetection(points_cloud, scan_indices, mark_index);
	for (size_t i = 0; i + 1 < mark_index.size(); ++i) {
		appendSegment(mark_index[i], mark_index[i + 1]);
	}
}

void LineFitting::updateLineFitMetrics(FittedLine& line) const {
	line.fit_success_ = false;
	line.fit_rms_error_ = 0.0f;
	line.fit_max_error_ = 0.0f;
	if (line.points_.size() < 2) {
		return;
	}
	const float denominator = line.k_is_exist
		? std::sqrt(line.k_ * line.k_ + 1.0f)
		: 1.0f;
	double squared_error_sum = 0.0;
	float max_error = 0.0f;
	for (const Eigen::Vector2f& point : line.points_) {
		const float error = line.k_is_exist
			? std::fabs(line.k_ * point.x() - point.y() + line.b_) / denominator
			: std::fabs(point.x() - line.b_);
		if (!std::isfinite(error)) {
			return;
		}
		squared_error_sum += static_cast<double>(error) * error;
		max_error = (std::max)(max_error, error);
	}
	line.fit_rms_error_ = static_cast<float>(std::sqrt(
		squared_error_sum / static_cast<double>(line.points_.size())));
	line.fit_max_error_ = max_error;
	line.fit_success_ = std::isfinite(line.fit_rms_error_);
}

void LineFitting::markIndexDetection(const std::vector<Eigen::Vector2f>& pointcloud,
	const std::vector<int>& scan_indices,
	std::vector<int>& mark_index) const {
	if (pointcloud.empty() || pointcloud.size() != scan_indices.size()) {
		mark_index.clear();
		return;
	}
	std::vector<int> marks{ 0, static_cast<int>(pointcloud.size()) - 1 };
	for (int i = 1; i < static_cast<int>(scan_indices.size()); ++i) {
		if (scan_indices[i] != scan_indices[i - 1] + 1) {
			marks.push_back(i - 1);
			marks.push_back(i);
		}
	}
	std::sort(marks.begin(), marks.end());
	marks.erase(std::unique(marks.begin(), marks.end()), marks.end());
	std::vector<std::pair<int, int>> pending_intervals;
	for (std::size_t i = 0; i + 1 < marks.size(); ++i) {
		pending_intervals.push_back({ marks[i], marks[i + 1] });
	}
	const float split_threshold_sq = 0.12f * 0.12f;
	while (!pending_intervals.empty()) {
		const std::pair<int, int> interval = pending_intervals.back();
		pending_intervals.pop_back();
		const int front = interval.first;
		const int back = interval.second;
		if (back <= front + 1) {
			continue;
		}
		const float p1_x = pointcloud[front].x();
		const float p1_y = pointcloud[front].y();
		const float seg_dx = pointcloud[back].x() - p1_x;
		const float seg_dy = pointcloud[back].y() - p1_y;
		const float seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;
		if (seg_len_sq < 1e-12f) {
			continue;
		}
		float max_dist_sq = 0.0f;
		int max_dist_index = front + 1;
		for (int j = front + 1; j < back; ++j) {
			const float point_x = pointcloud[j].x();
			const float point_y = pointcloud[j].y();
			const float a = point_x - p1_x;
			const float b = point_y - p1_y;
			const float param = (a * seg_dx + b * seg_dy) / seg_len_sq;
			const float dx = p1_x + param * seg_dx - point_x;
			const float dy = p1_y + param * seg_dy - point_y;
			const float dist_sq = dx * dx + dy * dy;
			if (dist_sq > max_dist_sq) {
				max_dist_sq = dist_sq;
				max_dist_index = j;
			}
		}
		if (max_dist_sq > split_threshold_sq) {
			marks.push_back(max_dist_index);
			pending_intervals.push_back({ front, max_dist_index });
			pending_intervals.push_back({ max_dist_index, back });
		}
	}
	std::sort(marks.begin(), marks.end());
	marks.erase(std::unique(marks.begin(), marks.end()), marks.end());
	mark_index = std::move(marks);
}

bool LineFitting::fittingLine2D(const std::vector<Eigen::Vector2f>& cloud,
	int first_index, int last_index, float& k, float& b, bool& k_exist) const {
	const int point_count = last_index - first_index + 1;
	if (first_index < 0 || last_index >= static_cast<int>(cloud.size()) || point_count < 2) {
		k = 0.0f;
		b = 0.0f;
		k_exist = false;
		return false;
	}
	float center_x = 0.0f;
	float center_y = 0.0f;
	for (int i = first_index; i <= last_index; ++i) {
		center_x += cloud[i].x();
		center_y += cloud[i].y();
	}
	center_x /= static_cast<float>(point_count);
	center_y /= static_cast<float>(point_count);
	float cov_xx = 0.0f;
	float cov_xy = 0.0f;
	float cov_yy = 0.0f;
	for (int i = first_index; i <= last_index; ++i) {
		const float dx = cloud[i].x() - center_x;
		const float dy = cloud[i].y() - center_y;
		cov_xx += dx * dx;
		cov_xy += dx * dy;
		cov_yy += dy * dy;
	}
	if (cov_xx + cov_yy < 1e-12f) {
		k = 0.0f;
		b = center_x;
		k_exist = false;
		return false;
	}
	const float direction_angle = 0.5f * std::atan2(2.0f * cov_xy, cov_xx - cov_yy);
	const float direction_x = std::cos(direction_angle);
	const float direction_y = std::sin(direction_angle);
	if (std::fabs(direction_x) < 1e-6f) {
		k = 0.0f;
		b = center_x;
		k_exist = false;
	}
	else {
		k = direction_y / direction_x;
		b = center_y - k * center_x;
		k_exist = true;
	}
	return std::isfinite(k) && std::isfinite(b);
}

bool LineFitting::checkDegradationByLineAngles() {
	std::vector<float> valid_angles;
	for (const FittedLine& line : lines_) {
		if (!line.fit_success_ || line.points_.size() < line_length_threshold_) {
			continue;
		}
		float angle;
		if (line.k_is_exist) {
			angle = std::atan(line.k_);
			if (angle < 0.0f) {
				angle += M_PI;
			}
		}
		else {
			angle = M_PI / 2.0f;
		}
		valid_angles.push_back(angle);
	}
	if (valid_angles.size() < 2) {
		return true;
	}
	for (std::size_t i = 0; i < valid_angles.size(); ++i) {
		for (std::size_t j = i + 1; j < valid_angles.size(); ++j) {
			float diff = std::fabs(valid_angles[i] - valid_angles[j]);
			if (diff > M_PI / 2.0f) {
				diff = M_PI - diff;
			}
			if (diff * (180.0f / M_PI) > 20.0f) {
				return false;
			}
		}
	}
	return true;
}
