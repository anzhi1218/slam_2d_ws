#include "linear_fitting.h"
#include <algorithm>
#include <cmath>
#include <iostream>

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

void LineFitting::buildGrid(const std::vector<Eigen::Vector2f>& points) {
	grid_.clear();

	const int n = static_cast<int>(points.size());

	for (int i = 0; i < n; ++i) {
		const float x = points[i].x();
		const float y = points[i].y();

		GridKey key{
			static_cast<int>(std::round(x / eps_)),
			static_cast<int>(std::round(y / eps_))
		};

		grid_[key].push_back(i);
	}
}

void LineFitting::regionQuery(int idx, const std::vector<Eigen::Vector2f>& points,
	std::vector<int>& neighbors) {
	neighbors.clear();

	const Eigen::Vector2f& pt = points[idx];
	const float cell_size_inv = 1.0f / eps_;
	const int cx = static_cast<int>(std::round(pt.x() * cell_size_inv));
	const int cy = static_cast<int>(std::round(pt.y() * cell_size_inv));
	const float eps2 = eps_ * eps_;
	const float px = pt.x();
	const float py = pt.y();

	for (const GridKey& offset : nearby_offsets_) {

		std::unordered_map<GridKey, std::vector<int>, GridKeyHash>::const_iterator it =
			grid_.find(GridKey{ cx + offset.x, cy + offset.y });

		if (it == grid_.end()) continue;

		const std::vector<int>& cell_points = it->second;
		const int cell_size = static_cast<int>(cell_points.size());
		for (int k = 0; k < cell_size; ++k) {
			const int ni = cell_points[k];
			if (ni == idx) continue;

			const float dx2 = px - points[ni].x();
			const float dy2 = py - points[ni].y();

			if (dx2 * dx2 + dy2 * dy2 <= eps2) {
				neighbors.push_back(ni);
			}
		}
	}
}

void LineFitting::removePointsNearOrigin(std::vector<Eigen::Vector2f>& cloud) {

	const float radius = 0.2f;
	const float r2 = radius * radius;

	int idx = 0;
	for (int i = 0; i < static_cast<int>(cloud.size()); ++i) {

		const float dist2 = cloud[i].x() * cloud[i].x() + cloud[i].y() * cloud[i].y();

		if (dist2 >= r2) {
			if (idx != i) {
				cloud[idx] = cloud[i];
			}
			++idx;
		}
	}
	cloud.resize(idx);
}

void LineFitting::runLineExtraction(const std::vector<Eigen::Vector2f>& cloud) {

	lines_.clear();
	clusters_.clear();

	// 1. 点云预处理
	std::vector<Eigen::Vector2f> filtered_cloud = cloud;
	removePointsNearOrigin(filtered_cloud);

	if (filtered_cloud.size() < static_cast<size_t>(min_line_length_)) {
		return;
	}

	// 2. 先计算点云的中心
	center_x_ = 0.0f;
	center_y_ = 0.0f;
	for (const auto& pt : cloud) {
		center_x_ += pt.x();
		center_y_ += pt.y();
	}
	center_x_ /= static_cast<float>(cloud.size());
	center_y_ /= static_cast<float>(cloud.size());

	// 3. DBSCAN 聚类
	std::vector<int> labels;
	dbscan(filtered_cloud, labels);

	// 4. 提取聚类
	extractClusters(filtered_cloud, labels);

	// 5. 直线拟合
	for (auto& cluster : clusters_) {
		if (cluster.size() >= static_cast<size_t>(min_line_length_)) {
			fitLines(cluster);
		}
	}

	// 6. 退化检测
	is_degradation_ = checkDegradationByLineAngles();
}


void LineFitting::dbscan(const std::vector<Eigen::Vector2f>& points, std::vector<int>& labels) {

	const int n = static_cast<int>(points.size());
	labels.assign(n, -2);
	int clusterId = 0;

	// Step 1: Build spatial grid
	buildGrid(points);

	const int minPts = 8;

	for (int i = 0; i < n; ++i) {

		if (labels[i] != -2) continue;

		// Step 2: Region query
		regionQuery(i, points, region_query_);

		if (static_cast<int>(region_query_.size()) < minPts) {
			labels[i] = -1;
			continue;
		}

		labels[i] = clusterId;
		seedQueue.clear();

		for (int nb : region_query_) {
			if (labels[nb] == -2) {
				labels[nb] = clusterId;
				seedQueue.push_back(nb);
			}
		}

		// Step 3: Seed queue expansion
		for (size_t j = 0; j < seedQueue.size(); ++j) {
			const int currentIdx = seedQueue[j];

			regionQuery(currentIdx, points, region_query_);

			if (static_cast<int>(region_query_.size()) >= minPts) {
				for (int nb : region_query_) {
					if (labels[nb] == -2) {
						labels[nb] = clusterId;
						seedQueue.push_back(nb);
					}
				}
			}
		}

		++clusterId;
	}

	// Step 4: Clear grid
	grid_.clear();
}


void LineFitting::extractClusters(const std::vector<Eigen::Vector2f>& points,
	const std::vector<int>& labels) {
	int max_label = -1;
	for (int label : labels) {
		if (label > max_label) {
			max_label = label;
		}
	}

	if (max_label < 0) {
		return;
	}

	clusters_.resize(max_label + 1);

	for (int i = 0; i < static_cast<int>(points.size()); ++i) {
		const int label = labels[i];
		if (label >= 0) {
			clusters_[label].push_back(points[i]);
		}
	}
}


void LineFitting::fitLines(std::vector<Eigen::Vector2f>& points_cloud) {

	if (points_cloud.size() < static_cast<size_t>(min_line_length_)) {
		return;
	}

	//sortClusterAlongPrincipalAxis(points_cloud);

	if (points_cloud.size() > 1) {
		std::sort(points_cloud.begin(), points_cloud.end(),
			[this](const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
				float angle_a = std::atan2(a.y() - center_y_, a.x() - center_x_);
				float angle_b = std::atan2(b.y() - center_y_, b.x() - center_x_);
				return angle_a < angle_b;
			});
	}

	std::vector<int> mark_index;
	markIndexDetection(points_cloud, mark_index);

	for (size_t i = 0; i + 1 < mark_index.size(); ++i) {
		const int front_index_id = mark_index[i];
		const int back_index_id = mark_index[i + 1];
		const int seg_size = back_index_id - front_index_id;

		if (seg_size < min_line_length_) {
			continue;
		}

		float k = 0.0f, b = 0.0f;
		bool k_exist = false;

		std::vector<Eigen::Vector2f> segment_points(points_cloud.begin() + front_index_id,
			points_cloud.begin() + back_index_id);

		if (!fittingLine2D(segment_points, k, b, k_exist)) {
			continue;
		}

		FittedLine line;
		line.k_ = k;
		line.b_ = b;
		line.k_is_exist = k_exist;
		line.points_ = std::move(segment_points);

		// 计算线段的端点坐标
		float min_x = line.points_[0].x();
		float max_x = min_x;
		float min_y = line.points_[0].y();
		float max_y = min_y;

		for (const auto& pt : line.points_) {
			const float x = pt.x();
			const float y = pt.y();
			if (x < min_x) min_x = x;
			else if (x > max_x) max_x = x;
			if (y < min_y) min_y = y;
			else if (y > max_y) max_y = y;
		}

		float p1_x, p1_y, p2_x, p2_y;
		if (k_exist) {
			p1_x = min_x; p1_y = k * min_x + b;
			p2_x = max_x; p2_y = k * max_x + b;
		}
		else {
			p1_x = b; p1_y = min_y;
			p2_x = b; p2_y = max_y;
		}

		const Eigen::Vector2f& front_pt = line.points_.front();
		const Eigen::Vector2f& back_pt = line.points_.back();
		float start_x, start_y, end_x, end_y;

		pointToLineSubpoint(front_pt.x(), front_pt.y(), p1_x, p1_y, p2_x, p2_y, start_x, start_y);
		pointToLineSubpoint(back_pt.x(), back_pt.y(), p1_x, p1_y, p2_x, p2_y, end_x, end_y);

		line.p0_ = Eigen::Vector2f(start_x, start_y);
		line.p1_ = Eigen::Vector2f(end_x, end_y);

		lines_.push_back(std::move(line));
	}
}

void LineFitting::sortClusterAlongPrincipalAxis(std::vector<Eigen::Vector2f>& cluster) {

	if (cluster.size() <= 1) {
		return;
	}

	float cx = 0.0f;
	float cy = 0.0f;
	for (const auto& pt : cluster) {
		cx += pt.x();
		cy += pt.y();
	}

	cx /= static_cast<float>(cluster.size());
	cy /= static_cast<float>(cluster.size());

	std::sort(cluster.begin(), cluster.end(),
		[cx, cy](const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
			float angle_a = std::atan2(a.y() - cy, a.x() - cx);
			float angle_b = std::atan2(b.y() - cy, b.x() - cx);
			return angle_a < angle_b;
		});
}

void LineFitting::markIndexDetection(const std::vector<Eigen::Vector2f>& pointcloud,
	std::vector<int>& mark_index) const {
	std::vector<int> mark_index_t;
	mark_index_t.push_back(0);
	mark_index_t.push_back(static_cast<int>(pointcloud.size()) - 1);

	const float split_thred = 0.12f;
	const float split_thred_sq = split_thred * split_thred;

	while (true) {
		std::vector<int> cur_mark_index;

		for (size_t i = 0; i + 1 < mark_index_t.size(); ++i) {
			const int front_index_id = mark_index_t[i];
			const int back_index_id = mark_index_t[i + 1];

			if (back_index_id <= front_index_id + 1) {
				continue;
			}

			const float p1_x = pointcloud[front_index_id].x();
			const float p1_y = pointcloud[front_index_id].y();
			const float p2_x = pointcloud[back_index_id].x();
			const float p2_y = pointcloud[back_index_id].y();

			const float seg_dx = p2_x - p1_x;
			const float seg_dy = p2_y - p1_y;
			const float seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;

			if (seg_len_sq < 1e-12f) {
				continue;
			}

			float max_dist_sq = 0.0f;
			int max_dist_index = front_index_id + 1;

			for (int j = front_index_id + 1; j < back_index_id; ++j) {
				const float point_x = pointcloud[j].x();
				const float point_y = pointcloud[j].y();

				const float A = point_x - p1_x;
				const float B = point_y - p1_y;
				const float param = (A * seg_dx + B * seg_dy) / seg_len_sq;

				const float line_pt_x = p1_x + param * seg_dx;
				const float line_pt_y = p1_y + param * seg_dy;

				const float dist_sq = (line_pt_x - point_x) * (line_pt_x - point_x) +
					(line_pt_y - point_y) * (line_pt_y - point_y);

				if (dist_sq > max_dist_sq) {
					max_dist_sq = dist_sq;
					max_dist_index = j;
				}
			}

			if (max_dist_sq > split_thred_sq) {
				cur_mark_index.push_back(max_dist_index);
			}
		}

		if (cur_mark_index.empty()) {
			break;
		}

		mark_index_t.insert(mark_index_t.end(), cur_mark_index.begin(), cur_mark_index.end());
		std::sort(mark_index_t.begin(), mark_index_t.end());
	}

	mark_index = std::move(mark_index_t);
}

bool LineFitting::fittingLine2D(const std::vector<Eigen::Vector2f>& cloud,
	float& k, float& b, bool& k_exist) const {

	const int n = static_cast<int>(cloud.size());

	if (n == 0) {
		k = 0.0f;
		b = 0.0f;
		k_exist = false;
		return false;
	}

	float sum_x = 0.0f;
	float sum_y = 0.0f;
	float sum_xx = 0.0f;
	float sum_xy = 0.0f;

	for (const auto& pt : cloud) {
		const float x = pt.x();
		const float y = pt.y();
		sum_x += x;
		sum_y += y;
		sum_xx += x * x;
		sum_xy += x * y;
	}

	const float mean_x = sum_x / static_cast<float>(n);
	const float var_x = sum_xx / static_cast<float>(n) - mean_x * mean_x;

	if (var_x < 0.001f) {
		k_exist = false;
		b = mean_x;
		return true;
	}

	const float denom = static_cast<float>(n) * sum_xx - sum_x * sum_x;
	k = (static_cast<float>(n) * sum_xy - sum_x * sum_y) / denom;
	b = (sum_xx * sum_y - sum_xy * sum_x) / denom;
	k_exist = true;

	return true;
}

void LineFitting::pointToLineSubpoint(const float point_x, const float point_y,
	const float p1_x, const float p1_y,
	const float p2_x, const float p2_y,
	float& line_pt_x, float& line_pt_y) {
	const float A = point_x - p1_x;
	const float B = point_y - p1_y;

	const float C = p2_x - p1_x;
	const float D = p2_y - p1_y;

	const float denom = C * C + D * D;

	if (std::abs(denom) < 1e-12f) {
		line_pt_x = p1_x;
		line_pt_y = p1_y;
		return;
	}

	const float param = (A * C + B * D) / denom;

	line_pt_x = p1_x + param * C;
	line_pt_y = p1_y + param * D;
}

bool LineFitting::checkDegradationByLineAngles() {

	std::vector<FittedLine> valid_lines;

	for (const auto& line : lines_) {
		if (line.points_.size() >= static_cast<size_t>(line_length_threshold_)) {
			valid_lines.push_back(line);
		}
	}

	if (valid_lines.size() < 2) {
		return true;
	}

	for (size_t i = 0; i < valid_lines.size(); ++i) {
		for (size_t j = i + 1; j < valid_lines.size(); ++j) {
			float angle_i;
			if (valid_lines[i].k_is_exist) {
				angle_i = std::atan(valid_lines[i].k_);
				if (angle_i < 0.0f) {
					angle_i += M_PI;
				}
			}
			else {
				angle_i = M_PI / 2.0f;
			}

			float angle_j;
			if (valid_lines[j].k_is_exist) {
				angle_j = std::atan(valid_lines[j].k_);
				if (angle_j < 0.0f) {
					angle_j += M_PI;
				}
			}
			else {
				angle_j = M_PI / 2.0f;
			}

			float diff = std::fabs(angle_i - angle_j);
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