#include "cartographer/mapping_2d/sparse_pose_graph.h"

#include <cmath>
#include <memory>
#include <random>

#include "cartographer/common/lua_parameter_dictionary_test_helpers.h"
#include "cartographer/common/make_unique.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping_2d/range_data_inserter.h"
#include "cartographer/mapping_2d/submaps.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/rigid_transform_test_helpers.h"
#include "cartographer/transform/transform.h"

cartographer::sensor::PointCloud point_cloud_;
std::unique_ptr<cartographer::mapping_2d::Submaps> submaps_;
cartographer::common::ThreadPool thread_pool_(1);
std::unique_ptr<cartographer::mapping_2d::SparsePoseGraph> sparse_pose_graph_;
cartographer::transform::Rigid2d current_pose_;


void SparsePoseGraphInit()
{
	for (float t = 0.f; t < 2.f * M_PI; t += 0.005f) {
		const float r = (std::sin(20.f * t) + 2.f) * std::sin(t + 2.f);
		point_cloud_.emplace_back(r * std::sin(t), r * std::cos(t), 0.f);
	}

	{
		auto parameter_dictionary = cartographer::common::MakeDictionary(R"text(
          return {
            resolution = 0.05,
            half_length = 21.,
            num_range_data = 1,
            output_debug_images = false,
            range_data_inserter = {
              insert_free_space = true,
              hit_probability = 0.53,
              miss_probability = 0.495,
            },
          })text");
		submaps_ = cartographer::common::make_unique<cartographer::mapping_2d::Submaps>(
			cartographer::mapping_2d::CreateSubmapsOptions(parameter_dictionary.get()));
	}

	{
		auto parameter_dictionary = cartographer::common::MakeDictionary(R"text(
          return {
            optimize_every_n_scans = 1000,
            constraint_builder = {
              sampling_ratio = 1.,
              max_constraint_distance = 6.,
              adaptive_voxel_filter = {
                max_length = 1e-2,
                min_num_points = 1000,
                max_range = 50.,
              },
              min_score = 0.5,
              global_localization_min_score = 0.6,
              lower_covariance_eigenvalue_bound = 1e-6,
              log_matches = true,
              fast_correlative_scan_matcher = {
                linear_search_window = 3.,
                angular_search_window = 0.1,
                branch_and_bound_depth = 3,
              },
              ceres_scan_matcher = {
                occupied_space_weight = 20.,
                translation_weight = 10.,
                rotation_weight = 1.,
                covariance_scale = 1.,
                ceres_solver_options = {
                  use_nonmonotonic_steps = true,
                  max_num_iterations = 50,
                  num_threads = 1,
                },
              },
            },
            optimization_problem = {
              acceleration_weight = 1.,
              rotation_weight = 1e2,
              huber_scale = 1.,
              consecutive_scan_translation_penalty_factor = 0.,
              consecutive_scan_rotation_penalty_factor = 0.,
              log_solver_summary = true,
              ceres_solver_options = {
                use_nonmonotonic_steps = false,
                max_num_iterations = 200,
                num_threads = 1,
              },
            },
            max_num_final_iterations = 200,
            global_sampling_ratio = 0.01,
          })text");
		sparse_pose_graph_ = cartographer::common::make_unique< cartographer::mapping_2d::SparsePoseGraph>(
			cartographer::mapping::CreateSparsePoseGraphOptions(parameter_dictionary.get()),
			&thread_pool_);
	}

	current_pose_ = cartographer::transform::Rigid2d::Identity();
}


void MoveRelativeWithNoise(const cartographer::transform::Rigid2d& movement,
	const cartographer::transform::Rigid2d& noise) {
	current_pose_ = current_pose_ * movement;
	const cartographer::sensor::PointCloud new_point_cloud = cartographer::sensor::TransformPointCloud(
		point_cloud_,
		cartographer::transform::Embed3D(current_pose_.inverse().cast<float>()));
	cartographer::kalman_filter::Pose2DCovariance covariance =
		cartographer::kalman_filter::Pose2DCovariance::Identity();
	const cartographer::mapping::Submap* const matching_submap =
		submaps_->Get(submaps_->matching_index());
	std::vector<const cartographer::mapping::Submap*> insertion_submaps;
	for (int insertion_index : submaps_->insertion_indices()) {
		insertion_submaps.push_back(submaps_->Get(insertion_index));
	}
	const cartographer::sensor::RangeData range_data{
		Eigen::Vector3f::Zero(), new_point_cloud,{} };
	const cartographer::transform::Rigid2d pose_estimate = noise * current_pose_;
	submaps_->InsertRangeData(TransformRangeData(
		range_data, cartographer::transform::Embed3D(pose_estimate.cast<float>())));
	sparse_pose_graph_->AddScan(cartographer::common::FromUniversal(0),
		cartographer::transform::Rigid3d::Identity(), range_data,
		pose_estimate, covariance, submaps_.get(),
		matching_submap, insertion_submaps);
}

void MoveRelative(const cartographer::transform::Rigid2d& movement) {
	MoveRelativeWithNoise(movement, cartographer::transform::Rigid2d::Identity());
}


void OverlappingScans()
{
	std::mt19937 rng(0);
	std::uniform_real_distribution<double> distribution(-1., 1.);
	std::vector<cartographer::transform::Rigid2d> ground_truth;
	std::vector<cartographer::transform::Rigid2d> poses;
	for (int i = 0; i != 5; ++i) {
		const double noise_x = 0.1 * distribution(rng);
		const double noise_y = 0.1 * distribution(rng);
		const double noise_orientation = 0.1 * distribution(rng);
		cartographer::transform::Rigid2d noise({ noise_x, noise_y }, noise_orientation);
		MoveRelativeWithNoise(
			cartographer::transform::Rigid2d({ 0.15 * distribution(rng), 0.4 }, 0.), noise);
		ground_truth.emplace_back(current_pose_);
		poses.emplace_back(noise * current_pose_);
	}
	sparse_pose_graph_->RunFinalOptimization();
	const auto nodes = sparse_pose_graph_->GetTrajectoryNodes();
	cartographer::transform::Rigid2d true_movement =
		ground_truth.front().inverse() * ground_truth.back();
	cartographer::transform::Rigid2d movement_before = poses.front().inverse() * poses.back();
	cartographer::transform::Rigid2d error_before = movement_before.inverse() * true_movement;
	cartographer::transform::Rigid3d optimized_movement =
		nodes.front().pose.inverse() * nodes.back().pose;
	cartographer::transform::Rigid2d optimized_error =
		cartographer::transform::Project2D(optimized_movement).inverse() * true_movement;

}
int main()
{
	SparsePoseGraphInit();
	OverlappingScans();
	return 0;
}