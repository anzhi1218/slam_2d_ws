#include "linear_fitting.h"
#include <algorithm>
#include <cmath>
#include <iostream>


LineFitting::LineFitting(int min_line_length, int line_length_threshold) :min_line_length_(min_line_length), line_length_threshold_(line_length_threshold) {

}


double LineFitting::distance(const Eigen::Vector3f& point_1, const Eigen::Vector3f& point_2) const {
    const double dx = point_1.x() - point_2.x();
    const double dy = point_1.y() - point_2.y();
    return dx * dx + dy * dy; 
}

std::vector<int> LineFitting::regionQuery(int idx, const std::vector<Eigen::Vector3f>& points) const {
    
	std::vector<int> neighbors;
    
    const double eps2 = 0.2 * 0.2;
    
    for (int i = 0; i < points.size(); ++i) {
        if (distance(points[idx], points[i]) <= eps2) {
            neighbors.push_back(i);
        }
    }
	
    return neighbors;
}

void LineFitting::removePointsNearOrigin(std::vector<Eigen::Vector3f>& cloud, double radius) {

	double r2 = radius * radius;
	std::vector<Eigen::Vector3f> filtered;
	filtered.reserve(cloud.size());

	for (int i = 0; i < cloud.size(); ++i) {
		const Eigen::Vector3f& pt = cloud[i];
		double dist2 = pt.x()*pt.x() + pt.y()*pt.y();
		if (dist2 >= r2) {
			filtered.push_back(pt);
		}
	}

	cloud.swap(filtered);
}

void LineFitting::runLineExtraction(const std::vector<Eigen::Vector3f>& cloud) {

	lines_.clear();
	clusters_.clear();
  
	//1.点云预处理
	std::vector<Eigen::Vector3f> filtered_cloud = cloud;
	const double radius_near_origin = 0.2;
	removePointsNearOrigin(filtered_cloud, radius_near_origin);

	if (filtered_cloud.size() < min_line_length_) {
		return;
	}
    
	//2.聚类
	std::vector<int> labels;
	dbscan(filtered_cloud, labels);
    
	//3.提取聚类
	extractClusters(filtered_cloud, labels);
    

	//4.直线拟合
	for (int i = 0; i < clusters_.size(); ++i) {
		
		std::vector<Eigen::Vector3f>& cluster = clusters_[i];

		std::cout << "LineFitting: Cluster[%d] points= " << cluster.size() << std::endl;

		fitLines(cluster);
	}

    //5.判断是否退化
	is_degradation_ = checkDegradationByLineAngles();
}


void LineFitting::dbscan(const std::vector<Eigen::Vector3f>& points, std::vector<int>& labels) {
    

	const int n = points.size();
	labels.assign(n, -2);
	int clusterId = 0;

	for (int i = 0; i < n; ++i) {

		if (labels[i] != -2) {
			continue;
		}
        
		//查询当前点附近邻域的点
		std::vector<int> neighbors = regionQuery(i, points);
		if (neighbors.size() < 8) { //小于最小聚类的点数

			labels[i] = -1;
			continue;
		}

		labels[i] = clusterId;
		std::vector<int> seedQueue = neighbors;
		seedQueue.erase(std::remove(seedQueue.begin(), seedQueue.end(), i), seedQueue.end());
        
		//扩展簇
		for (int j = 0; j < seedQueue.size(); ++j) {

			const int idx = seedQueue[j];

			if (labels[idx] == -2) {

				labels[idx] = clusterId;
				const std::vector<int> newNeighbors = regionQuery(idx, points);

				if (newNeighbors.size() >= 8) {

					for (int nb : newNeighbors) {
						if (std::find(seedQueue.begin(), seedQueue.end(), nb) == seedQueue.end() && nb != idx) {
							seedQueue.push_back(nb);
						}
					}
				}
			}
			else if (labels[idx] == -1) {

				labels[idx] = clusterId;
			}
		}

		++clusterId;
	}

}


void LineFitting::extractClusters(const std::vector<Eigen::Vector3f>& points,
	
	const std::vector<int>& labels) {
	int max_label = -1;
    
	//找到最大的label
	for (int label : labels) {
		if (label > max_label) {
			max_label = label;
		}
	}

	if (max_label < 0) {
		return;
	}

	clusters_.resize(max_label + 1);
    
	//提取簇
	for (int i = 0; i < points.size(); ++i) {
		const int label = labels[i];
		if (label >= 0) {
			clusters_[label].push_back(points[i]);
		}
	}
}


void LineFitting::fitLines(std::vector<Eigen::Vector3f>& points_cloud) {


	if (points_cloud.size() < min_line_length_) {
		return;
	}

	sortClusterAlongPrincipalAxis(points_cloud);

	std::vector<int> mark_index;
	markIndexDetection(points_cloud, mark_index);

	for (int i = 0; i < mark_index.size() - 1; ++i) {

		const int front_index_id = mark_index[i];
		const int back_index_id = mark_index[i + 1];

		std::vector<Eigen::Vector3f> segment_points;
		segment_points.reserve(back_index_id - front_index_id);
		for (int j = front_index_id; j < back_index_id; ++j) {
			segment_points.push_back(points_cloud[j]);
		}

		if (segment_points.size() < min_line_length_) {
			continue;
		}

		double k = 0.0;
		double b = 0.0;
		bool k_exist = false;
		if (!fittingLine2D(segment_points, k, b, k_exist)) {
			continue;
		}

		FittedLine line;
		line.k_ = static_cast<float>(k);
		line.b_ = static_cast<float>(b);
		line.k_is_exist = k_exist;
		line.points_ = std::move(segment_points);

		lines_.push_back(line);
	}
}

void LineFitting::sortClusterAlongPrincipalAxis(std::vector<Eigen::Vector3f>& cluster) {


	double cx = 0.0;
	double cy = 0.0;

	for (const auto& pt : cluster) {
		cx += static_cast<double>(pt.x());
		cy += static_cast<double>(pt.y());
	}

	cx /= static_cast<double>(cluster.size());
	cy /= static_cast<double>(cluster.size());

	double cxx = 0.0;
	double cxy = 0.0;
	double cyy = 0.0;
	for (const Eigen::Vector3f& pt : cluster) {
		const double dx = static_cast<double>(pt.x()) - cx;
		const double dy = static_cast<double>(pt.y()) - cy;
		cxx += dx * dx;
		cxy += dx * dy;
		cyy += dy * dy;
	}

	const double trace = cxx + cyy;
	const double det = cxx * cyy - cxy * cxy;
	const double disc = std::sqrt(std::max(0.0, trace * trace * 0.25 - det));
	const double lambda1 = trace * 0.5 + disc;

	double vx = cxy;
	double vy = lambda1 - cxx;
	double norm = std::sqrt(vx * vx + vy * vy);

	if (norm < 1e-9) {
		if (cxx >= cyy) {
			vx = 1.0; vy = 0.0;
		}
		else {
			vx = 0.0; vy = 1.0;
		}
	}
	else {
		vx /= norm;
		vy /= norm;
	}

	std::sort(cluster.begin(), cluster.end(), [cx, cy, vx, vy](const Eigen::Vector3f& point1, const Eigen::Vector3f& point2) {

		const double proj1 = (static_cast<double>(point1.x()) - cx) * vx + (static_cast<double>(point1.y()) - cy) * vy;
		const double proj2 = (static_cast<double>(point2.x()) - cx) * vx + (static_cast<double>(point2.y()) - cy) * vy;
		return proj1 < proj2;
	});
}

void LineFitting::markIndexDetection(const std::vector<Eigen::Vector3f>& pointcloud, std::vector<int>& mark_index) const {


	std::vector<int> mark_index_t;
	mark_index_t.push_back(0);
	mark_index_t.push_back(static_cast<int>(pointcloud.size()) - 1);

	while (true) {

		std::vector<int> cur_mark_index;

		for (int i = 0; i < static_cast<int>(mark_index_t.size()) - 1; ++i) {

			const int front_index_id = mark_index_t[i];
			const int back_index_id = mark_index_t[i + 1];


			if (back_index_id <= front_index_id + 1) {
				continue;
			}

			const double p1_x = static_cast<double>(pointcloud[front_index_id].x());
			const double p1_y = static_cast<double>(pointcloud[front_index_id].y());
			const double p2_x = static_cast<double>(pointcloud[back_index_id].x());
			const double p2_y = static_cast<double>(pointcloud[back_index_id].y());

			const double seg_dx = p2_x - p1_x;
			const double seg_dy = p2_y - p1_y;
			const double seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;

			if (seg_len_sq < 1e-12) {
				continue;
			}

			double max_dist2 = 0.0;
			int max_dist_index = front_index_id + 1;

			for (int j = front_index_id + 1; j < back_index_id; ++j) {
				const double point_x = static_cast<double>(pointcloud[j].x());
				const double point_y = static_cast<double>(pointcloud[j].y());

				const double A = point_x - p1_x;
				const double B = point_y - p1_y;
				const double param = (A * seg_dx + B * seg_dy) / seg_len_sq;

				const double line_pt_x = p1_x + param * seg_dx;
				const double line_pt_y = p1_y + param * seg_dy;

				const double dist2 = (line_pt_x - point_x) * (line_pt_x - point_x) + (line_pt_y - point_y) * (line_pt_y - point_y);

				if (dist2 > max_dist2) {
					max_dist2 = dist2;
					max_dist_index = j;
				}
			}

			const double MaxDistThreshold2 = 0.12*0.12;
			if (max_dist2 > MaxDistThreshold2) { 
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

bool LineFitting::fittingLine2D(const std::vector<Eigen::Vector3f>& cloud,
	                            double& k, double& b, bool& k_exist) const {

	const int n = static_cast<int>(cloud.size());

	if (n == 0) {
		k = 0.0;
		b = 0.0;
		k_exist = false;
		return false;
	}

	double sum_x = 0.0;
	double sum_y = 0.0;
	double sum_xx = 0.0;
	double sum_xy = 0.0;
	

	for (int i = 0; i < n; ++i) {
		const double x = static_cast<double>(cloud[i].x());
		const double y = static_cast<double>(cloud[i].y());
		sum_x += x;
		sum_y += y;
		sum_xx += x * x;
		sum_xy += x * y;
	}

	const double mean_x = sum_x / n;

	const double var_x = sum_xx / n - mean_x * mean_x;

	if (var_x < 0.001) {
		k_exist = false;
		b = mean_x;
		return true;
	}

	const double denom = static_cast<double>(n) * sum_xx - sum_x * sum_x;
	
	k = (static_cast<double>(n) * sum_xy - sum_x * sum_y) / denom;
	b = (sum_xx * sum_y - sum_xy * sum_x) / denom;
	k_exist = true;

	return true;
}

bool LineFitting::checkDegradationByLineAngles() {

	std::vector<FittedLine> valid_lines;
	for (int i = 0; i < lines_.size(); ++i) {
		if (lines_[i].points_.size() >= line_length_threshold_) {
			valid_lines.push_back(lines_[i]);
		}
	}

	std::cout << "LineFitting: valid_lines size = " << valid_lines.size() << std::endl;


	if (valid_lines.size() < 2) {
		return true;
	}

	for (int i = 0; i < valid_lines.size(); ++i) {
		for (int j = i + 1; j < valid_lines.size(); ++j) {

			double angle_i;
			if (valid_lines[i].k_is_exist) {
				angle_i = std::atan(valid_lines[i].k_);

				if (angle_i < 0.0) {
					angle_i += M_PI;
				}
			}
			else {
				angle_i = M_PI / 2.0;
			}

			double angle_j;
			if (valid_lines[j].k_is_exist) {
				angle_j = std::atan(valid_lines[j].k_);
				if (angle_j < 0.0) {
					angle_j += M_PI;
				}
			}
			else {
				angle_j = M_PI / 2.0;
			}

			double diff = std::fabs(angle_i - angle_j);
			if (diff > M_PI / 2.0) {
				diff = M_PI - diff;
			}

			if ((diff * 180.0 / M_PI) > 20.0) {
				return false;
			}
		}
	}

	return true;
}