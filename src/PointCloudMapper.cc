#include <point_cloud_mapper/PointCloudMapper.h>
//#include <parameter_utils/ParameterUtils.h>
#include <parameter_utils/slamBase.h>

//namespace pu = parameter_utils;

PointCloudMapper::PointCloudMapper()
	: initialized_(false),
	map_updated_(false),
	incremental_unsubscribed_(false),
	map_data_list_(new std::vector<int>)
{
	// Initialize map data container.
	map_data_.reset(new PointCloud);
}

PointCloudMapper::~PointCloudMapper() {
	if (publish_thread_.joinable()) {
		publish_thread_.join();
	}
}

bool PointCloudMapper::Initialize(/*const ros::NodeHandle& n*/) {
	if (!LoadParameters()) {

		return false;
	}
	return true;
}

bool PointCloudMapper::LoadParameters() {
	ParameterReader pd;
	fixed_frame_id_ = pd.getData("fixed");
	map_data_->header.frame_id = fixed_frame_id_;
	octree_resolution_ = atof(pd.getData("octree_resolution").c_str());
	translation_threshold_ = atof(pd.getData("translation_threshold").c_str());
	string szTemp = pd.getData("local_map_integrate_distance");
	if (szTemp == string("NOT_FOUND"))
	{
		cout << "local_map_integrate_distance NOT_FOUND, set it to DBL_MAX" << endl;
		local_map_integrate_distance_ = DBL_MAX;
	}
	else
	{
		local_map_integrate_distance_ = atof(szTemp.c_str());
		if (local_map_integrate_distance_ <= 0.0)
		{
			cout << "local_map_integrate_distance <= 0.0, set it to DBL_MAX" << endl;
			local_map_integrate_distance_ = DBL_MAX;
		}
	}
	cout << "local_map_integrate_distance: " << local_map_integrate_distance_ << endl;
	
	// Initialize the map octree.
	map_octree_.reset(new Octree(octree_resolution_));
	map_octree_->setInputCloud(map_data_);

	initialized_ = true;

	return true;
}

void PointCloudMapper::Reset() {
	map_data_.reset(new PointCloud);
	map_data_->header.frame_id = fixed_frame_id_;
	map_octree_.reset(new Octree(octree_resolution_));
	map_octree_->setInputCloud(map_data_);

	map_data_list_->clear();
	map_data_list_info_ = queue<int>();

	initialized_ = true;
}

void PointCloudMapper::TrimData()
{
	int nMapLong = local_map_integrate_distance_ / translation_threshold_;
	if (map_data_list_info_.size() > nMapLong)
	{
		int nTrimSize = map_data_list_info_.front();
		map_data_list_info_.pop();
		for (unsigned int i = 0; i < nTrimSize; i++)
		{
			map_octree_->deleteVoxelAtPoint(map_data_->at(i));
		}
		map_data_->erase(map_data_->begin(), map_data_->begin()+ nTrimSize);
		cout << "delete point cont: " << nTrimSize << endl;
	}
}

bool PointCloudMapper::InsertPoints(const PointCloud::ConstPtr& points,
	PointCloud* incremental_points) {
	if (!initialized_) {
		return false;
	}

	if (incremental_points == NULL) {
		return false;
	}
	incremental_points->clear();


	if (map_mutex_.try_lock()) 
	{
		int nAddCont = 0;
		for (size_t ii = 0; ii < points->points.size(); ++ii)
		{
			const pcl::POINT_TYPE p = points->points[ii];
			if (!map_octree_->isVoxelOccupiedAtPoint(p))
			{
				map_octree_->addPointToCloud(p, map_data_, map_data_list_);
				incremental_points->push_back(p);
				nAddCont++;
			}
		}
		map_data_list_info_.push(nAddCont);

		TrimData();

		map_mutex_.unlock();
	}
	else
	{

	}

	// Publish the incremental map update.
	incremental_points->header = points->header;
	incremental_points->header.frame_id = fixed_frame_id_;
	//  PublishMapUpdate(*incremental_points);

	map_updated_ = true;
	return true;
}

bool PointCloudMapper::ApproxNearestNeighbors(const PointCloud& points,
	PointCloud* neighbors) {
	if (!initialized_) {
	}

	if (neighbors == NULL) {
	}

	neighbors->points.clear();

	for (size_t ii = 0; ii < points.points.size(); ++ii) {
		float unused = 0.f;
		int result_index = -1;

		map_octree_->approxNearestSearch(points.points[ii], result_index, unused);
		if (result_index >= 0)
			neighbors->push_back(map_data_->points[result_index]);
	}

	return neighbors->points.size() > 0;
}
