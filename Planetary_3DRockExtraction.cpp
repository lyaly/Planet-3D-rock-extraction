// Planetary_3DRockFeatureExtraction.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/statistical_outlier_removal.h>


using namespace std;
using namespace pcl;

struct Rock
{
	Rock()
	{
		pts = NULL;
		normals = NULL;
		min_p = PointXYZ(FLT_MAX, FLT_MAX, FLT_MAX);
		max_p = PointXYZ(FLT_MIN, FLT_MIN, FLT_MIN);
	};
	~Rock()
	{};
	PointCloud<PointXYZ>::Ptr pts;
	PointCloud<Normal>::Ptr normals;
	PointXYZ min_p;
	PointXYZ max_p;
	PointXYZ center_p;
};

 int readLabeldPointCloudfromTXT(const string& fpath, PointCloud<PointXYZL>::Ptr& cloud, PointCloud<Normal>::Ptr& normals)
{
	cout << endl << "  --> Reading file: " << fpath << "..." << endl;
	cloud = PointCloud<PointXYZL>::Ptr(new PointCloud<PointXYZL>);
	normals = PointCloud<Normal>::Ptr(new PointCloud<Normal>);
	ifstream infile(fpath);
	if (!infile.is_open())
	{
		cout << "      filed to open file: " << endl;
		return 0;
	}
	double x, y, z, label, nx, ny, nz;

	while (infile >> x >> y >> z >> label >> nx >> ny >> nz)
	{
		PointXYZL p;
		p.x = x;
		p.y = y;
		p.z = z;
		p.label = int(label);
		cloud->points.push_back(p);

		Normal n;
		n.normal_x = nx;
		n.normal_y = ny;
		n.normal_z = nz;
		normals->points.push_back(n);
	}

	if (cloud->empty() || normals->empty())
	{
		cout << "      The point cloud is empty." << endl;
		return 0;
	}

	return 1;
}

vector<Rock> getRocks(const PointCloud<PointXYZL>::Ptr& cloud, const PointCloud<Normal>::Ptr& normals)
{
	map<int, vector<int>> rocks;
	for (int i = 0; i < cloud->size(); i++)
	{
		int label = cloud->points[i].label;
		//if (label == 0)
		   // continue;

		rocks[label].push_back(i);
	}

	vector<Rock> Rocks;
	for (map<int, vector<int>>::iterator itr = rocks.begin(); itr != rocks.end(); itr++)
	{
		PointCloud<PointXYZ>::Ptr pts_xyz(new  PointCloud<PointXYZ>());
		PointCloud<Normal>::Ptr pts_normal(new  PointCloud<Normal>());
		copyPointCloud(*cloud, itr->second, *pts_xyz);
		copyPointCloud(*normals, itr->second, *pts_normal);
		PointXYZ minp, maxp;
		getMinMax3D(*pts_xyz, minp, maxp);
		Rock a_rock;
		a_rock.pts = pts_xyz;
		a_rock.normals = pts_normal;
		a_rock.min_p = minp;
		a_rock.max_p = maxp;
		a_rock.center_p = PointXYZ((maxp.x + minp.x)/2, (maxp.y + minp.y)/2, (maxp.z + minp.z)/2);

		Rocks.push_back(a_rock);
	}
	return Rocks;
}

int saveRocksAsLabeledPointCloudtoTXT(const vector<Rock>& rocks, const string& fpath)
{
	cout << endl << "  --> Saving file: " << fpath << "..." << endl;
	ofstream outfile(fpath);
	if (!outfile.is_open())
	{
		cout << "      filed to open file: " << endl;
		return 0;
	}
	for (int i = 0; i < rocks.size(); i++)
	{
		Rock r = rocks[i];
		for (int j = 0; j < rocks[i].pts->size(); j++)
		{
			outfile << r.pts->points[j].x << "\t" << r.pts->points[j].y << "\t" << r.pts->points[j].z << "\t" << i + 1 << endl;
		}
	}
	return 1;
}

/********************************************
Function: Separating close-distributed rocks
*********************************************/
vector<Rock> refining_rock(const Rock& a_rock);
int segment_rocks(const vector<Rock>& origin_rocks)
{
	cout << endl << "  -->Refining rock clustering..." << endl;
	int process = 0;
	int psize = origin_rocks.size();
	vector<Rock> refined_rocks;
	for (int i = 0; i < origin_rocks.size(); i++)
	{
		vector<Rock> new_rocks = refining_rock(origin_rocks[i]);
		refined_rocks.insert(refined_rocks.end(), new_rocks.begin(), new_rocks.end());
		int pro = (int((float)i / psize * 10));
		if (pro > process)
		{
			process = pro;
			cout << "   " << process * 10 << "%";
		}
	}
	cout << "   100%" << endl;

// 	PointCloud<PointXYZL>::Ptr out_cloud(new PointCloud<PointXYZL>);
// 	for (int i = 0; i < refined_rocks.size(); i++ )
// 	{
// 		PointCloud<PointXYZL>::Ptr rock_pts(new PointCloud<PointXYZL>);
// 		copyPointCloud(*(refined_rocks[i].pts), *rock_pts);
// 		for (auto p : rock_pts->points)
// 			p.label = i + 1;
// 		out_cloud->insert(out_cloud->end(), rock_pts->begin(), rock_pts->end());
// 	}
	cout << " Savepath of rock point cloud file (.txt):";
	string outfpath;
	cin >> outfpath;
	ofstream out_file(outfpath);
	if (out_file.is_open())
	{
		for (int i = 0; i < refined_rocks.size(); i++ )
		{
			for (int j = 0; j < refined_rocks[i].pts->size(); j++)
			{
				out_file << refined_rocks[i].pts->points[j].x << "\t"
					<< refined_rocks[i].pts->points[j].y << "\t"
					<< refined_rocks[i].pts->points[j].z << "\t"
					<< i + 1 << endl;
			}
		}
	}
	out_file.close();
	cout << "Finished!" << endl;
	return 1;
}

// Refining individual rock
vector<Rock> refining_rock(const Rock& a_rock)
{
	// parameters
	int K_for_normal = 20;
	int K_for_regiongrowing = 15;
	float radius_for_CC = 0.01;
	float CC_threshold = -0.5;
	float RG_angle_threshold = cos(M_PI * 10.0 / 180.0);
	// build index
	search::KdTree<pcl::PointXYZ>::Ptr tree(new search::KdTree<pcl::PointXYZ>());
	tree->setInputCloud(a_rock.pts);

	// compute point density
	float point_density = 0;
	for (auto p : a_rock.pts->points)
	{
		vector<int> nb_indices;
		vector<float> nb_dises;
		tree->nearestKSearch(p, 5, nb_indices, nb_dises);
		float d = (sqrtf(nb_dises[1]) + sqrtf(nb_dises[2]) + +sqrtf(nb_dises[3]) + +sqrtf(nb_dises[4])) / 4;
		point_density += d;
	}
	point_density /= a_rock.pts->size();
	cout << "Average point density is " << point_density << endl;

// 	// compute point normal
// 	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
// 	NormalEstimation<PointXYZ, Normal> normal_estimator;
// 	normal_estimator.setInputCloud(a_rock.pts);
// 	normal_estimator.setSearchMethod(tree);
// 	normal_estimator.setKSearch(K_for_normal);
// 	normal_estimator.compute(*normals);

	PointCloud<Normal>::Ptr normals(new PointCloud<Normal>());
	copyPointCloud(*(a_rock.normals), *normals);

	// compute the concavity at each point - CC(x)
	vector<float> CC(a_rock.pts->points.size());
	for (int i = 0; i < a_rock.pts->size(); i++)
	{
		PointXYZ p = a_rock.pts->points[i];
		Eigen::Vector3f vp(p.x, p.y, p.z);
		Eigen::Vector3f np(normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
		vector<int> nb_indices;
		vector<float> nb_dises;
		tree->radiusSearch(p, radius_for_CC, nb_indices, nb_dises);
		float cc = 0;
		for (int j = 1; j < nb_indices.size(); j++)
		{
			PointXYZ q = a_rock.pts->points[nb_indices[j]];
			Eigen::Vector3f vq(q.x, q.y, q.z);
			Eigen::Vector3f nq(normals->points[nb_indices[j]].normal_x, normals->points[nb_indices[j]].normal_y, normals->points[nb_indices[j]].normal_z);
			float cc_pair = (np - nq).dot(vp - vq) / (vp - vq).norm();
			cc += cc_pair;
		}
		cc /= (nb_indices.size() - 1);
		CC[i] = cc;
	}
	/**************************temp output**************************/
	{
// 		ofstream tmp_file("D:\\Data\\TianWen-1\\Zhurong\\P9\\tmp_CC.txt");
// 		if (tmp_file.is_open())
// 		{
// 			for (int i = 0; i < a_rock.pts->size(); i++)
// 			{
// 				tmp_file << a_rock.pts->points[i].x << "\t" << a_rock.pts->points[i].y << "\t" << a_rock.pts->points[i].z << "\t" << CC[i] << endl;
// 			}
// 		}
// 		tmp_file.close();
	}
	/***********************************************************/
	// find seed (water source)
	vector<int> seed_indices;
	vector<bool> convex_flags(CC.size(), true);
	for (int i = 0; i < CC.size(); i++)
	{
		if (CC[i] < CC_threshold)
		{
			seed_indices.push_back(i);
			convex_flags[i] = false;
		}
	}
	// isolated point cannot be seed
	vector<int> new_seed_indices;
	//vector<float> seed_densities;
	PointCloud<PointXYZ>::Ptr tmp_seeds(new PointCloud<PointXYZ>());
	copyPointCloud(*(a_rock.pts), seed_indices, *tmp_seeds);
	// build index
	search::KdTree<pcl::PointXYZ>::Ptr tree2(new search::KdTree<pcl::PointXYZ>());
	tree2->setInputCloud(tmp_seeds);
	for (int i = 0; i < tmp_seeds->size(); i++)
	{
		PointXYZ p = tmp_seeds->points[i];
		vector<int> nb_indices;
		vector<float> nb_dises;
		tree2->nearestKSearch(p, 5, nb_indices, nb_dises);
		float d = (sqrtf(nb_dises[1]) + sqrtf(nb_dises[2]) + +sqrtf(nb_dises[3]) + +sqrtf(nb_dises[4])) / 4;
		//seed_densities.push_back(d);
		if (d < point_density*1.5)
			new_seed_indices.push_back(seed_indices[i]);
	}
	seed_indices = new_seed_indices;
// 	StatisticalOutlierRemoval<PointXYZ> sor;
// 	sor.setInputCloud(tmp_seeds);
// 	sor.setMeanK(10);
// 	sor.setStddevMulThresh(5);
// 	sor.filter(ids);
// 	vector<int> new_seed_indices;
// 	for (auto i : ids)
// 		new_seed_indices.push_back(seed_indices[i]);
// 	seed_indices = new_seed_indices;


	/**************************temp output**************************/
	{
// 		ofstream tmp_file("D:\\Data\\TianWen-1\\Zhurong\\P9\\tmp_seeds.txt");
// 		if (tmp_file.is_open())
// 		{
// 			for (int i = 0; i < seed_indices.size(); i++)
// 			{
// 				tmp_file << a_rock.pts->points[seed_indices[i]].x << "\t" << a_rock.pts->points[seed_indices[i]].y << "\t" << a_rock.pts->points[seed_indices[i]].z << endl;
// 			}
// 		}
// 		tmp_file.close();
	}

	// Region growing 
	while (!seed_indices.empty())
	{
		PointXYZ seed = a_rock.pts->points[seed_indices[0]];
		Normal normal = normals->points[seed_indices[0]];
		Eigen::Vector3f p_normal(normal.normal_x, normal.normal_y, normal.normal_z);
		vector<int> nb_indices;
		vector<float> nb_dises;
		tree->nearestKSearch(seed, K_for_regiongrowing, nb_indices, nb_dises);
		for (int i = 1; i < nb_indices.size(); i++)
		{
			int nb_idx = nb_indices[i];
			if (convex_flags[nb_idx] == false)
				continue;
			Eigen::Vector3f q_normal(normals->points[nb_idx].normal_x, normals->points[nb_idx].normal_y, normals->points[nb_idx].normal_z);
			float cos_angle = p_normal.dot(q_normal);
			if (cos_angle > RG_angle_threshold && CC[nb_idx] < 0)
			{
				seed_indices.push_back(nb_idx);
				convex_flags[nb_idx] = false;
			}
		}
		seed_indices.erase(seed_indices.begin());
	}

	// rasterization and connectivity analysis
	PointCloud<PointXYZ>::Ptr convex_pts(new PointCloud<PointXYZ>);
	for (int i = 0; i < convex_flags.size(); i++)
	{
		if (convex_flags[i])
			convex_pts->push_back(a_rock.pts->points[i]);
	}
	/**************************temp output**************************/
	{
// 		ofstream tmp_file("D:\\Data\\TianWen-1\\Zhurong\\P9\\tmp_RG.txt");
// 		if (tmp_file.is_open())
// 		{
// 			for (int i = 0; i < convex_pts->size(); i++)
// 			{
// 				tmp_file << convex_pts->points[i].x << "\t" << convex_pts->points[i].y << "\t" << convex_pts->points[i].z << endl;
// 			}
// 		}
// 		tmp_file.close();
	}
	/***********************************************************/

	float min_x = a_rock.min_p.x;
	float min_y = a_rock.min_p.y;
	float resolution = point_density * 2.2;
	int cols = (a_rock.max_p.x - min_x) / resolution + 1;
	int rows = (a_rock.max_p.y - min_y) / resolution + 1;
	vector<vector<int>> grid(cols*rows);
	for (int i = 0; i < convex_pts->size(); i++)
	{
		int c = (convex_pts->points[i].x - min_x) / resolution;
		int r = (convex_pts->points[i].y - min_y) / resolution;
		grid[r*cols + c].push_back(i);
	}
	//Open operation
	vector<bool> binary_grid(cols*rows, false);
	for (int i = 0; i < grid.size(); i++)
	{
		if (grid[i].size() >= 2)
			binary_grid[i] = true;
	}
	// erosion operation
	vector<bool> binary_grid_erosion(cols*rows, true);
	for (int r = 0; r < rows; r++)
	{
		for (int c = 0; c < cols; c++)
		{
			bool exist_false = false;
			for (int r1 = r - 1; r1 <= r + 1; r1++)
			{
				for (int c1 = c - 1; c1 <= c + 1; c1++)
				{
					if (r1 < 0 || c1 < 0 || r1 >= rows || c1 >= cols)
						continue;
					if (r1 == 0 && c1 == 0)
						continue;
					if (binary_grid[r1 * cols + c1] == false)
					{
						exist_false = true;
						break;
					}
				}
				if (exist_false)
					break;
			}
			if (exist_false)
				binary_grid_erosion[r*cols + c] = false;
		}
	}
	//dilation operation
	vector<bool> binary_grid_dilate(cols*rows, false);
	for (int r = 0; r < rows; r++)
	{
		for (int c = 0; c < cols; c++)
		{
			bool exist_true = false;
			for (int r1 = r - 1; r1 <= r + 1; r1++)
			{
				for (int c1 = c - 1; c1 <= c + 1; c1++)
				{
					if (r1 < 0 || c1 < 0 || r1 >= rows || c1 >= cols)
						continue;
					if (r1 == 0 && c1 == 0)
						continue;
					if (binary_grid_erosion[r1 * cols + c1] == true)
					{
						exist_true = true;
						break;
					}
				}
				if (exist_true)
					break;
			}
			if (exist_true)
				binary_grid_dilate[r*cols + c] = true;
		}
	}

	/**************************temp output**************************/
	{
// 		ofstream tmp_file("D:\\Data\\TianWen-1\\Zhurong\\P9\\tmp_Open.txt");
// 		if (tmp_file.is_open())
// 		{
// 			for (int i = 0; i < binary_grid_dilate.size(); i++)
// 			{
// 				if (binary_grid_dilate[i] == true)
// 				{
// 					for (int j = 0; j < grid[i].size(); j++)
// 					{
// 						tmp_file << convex_pts->points[grid[i][j]].x << "\t" << convex_pts->points[grid[i][j]].y << "\t" << convex_pts->points[grid[i][j]].z << endl;
// 					}
// 				}
// 			}
// 
// 		}
// 		tmp_file.close();
	}
	/***********************************************************/

	// connectivity analysis
	vector<vector<int>> grid_clusters;
	vector<bool> grid_flags = binary_grid_dilate;
	while (std::count(grid_flags.begin(), grid_flags.end(), true) > 0)
	{
		//  find seed
		vector<int> seeds;
		for (int i = 0; i < grid_flags.size(); i++)
			if (grid_flags[i])
			{
				seeds.push_back(i);
				break;
			}
		vector<int> cluster;
		while (!seeds.empty())
		{
			if (grid_flags[seeds[0]] == true)
			{
				cluster.push_back(seeds[0]);
				grid_flags[seeds[0]] = false;
				int seed_r = seeds[0] / cols;
				int seed_c = seeds[0] % cols;
				for (int r = seed_r - 1; r <= seed_r + 1; r++)
				{
					for (int c = seed_c - 1; c <= seed_c + 1; c++)
					{
						if (r < 0 || c < 0 || r >= rows || c >= cols)
							continue;
						if (r == 0 && c == 0)
							continue;
						if (grid_flags[r*cols + c] == false)
							continue;
						seeds.push_back(r*cols + c);
					}
				}
			}
			seeds.erase(seeds.begin());
		}
		grid_clusters.push_back(cluster);
	}

	// clustering as rocks
	vector<Rock> new_Rocks(grid_clusters.size());
	for (int i = 0; i < grid_clusters.size(); i++)
	{
		new_Rocks[i].pts = PointCloud<PointXYZ>::Ptr(new  PointCloud<PointXYZ>);
		for (int j = 0; j < grid_clusters[i].size(); j++)
		{
			PointCloud<PointXYZ>::Ptr tmp_pts(new  PointCloud<PointXYZ>());
			copyPointCloud(*convex_pts, grid[grid_clusters[i][j]], *tmp_pts);
			new_Rocks[i].pts->insert(new_Rocks[i].pts->end(), tmp_pts->begin(), tmp_pts->end());
		}
		PointXYZ minp, maxp;
		getMinMax3D(*new_Rocks[i].pts, minp, maxp);
		new_Rocks[i].min_p = minp;
		new_Rocks[i].max_p = maxp;
		new_Rocks[i].center_p = PointXYZ((maxp.x + minp.x) / 2, (maxp.y + minp.y) / 2, (maxp.z + minp.z) / 2);
	}

	return new_Rocks;

}

int main()
{
	cout << "Input rock point cloud file (.txt) filepath: ";
	string infpath;
	cin >> infpath;

	PointCloud<PointXYZL>::Ptr cloud;
	PointCloud<Normal>::Ptr normals;
	if (readLabeldPointCloudfromTXT(infpath, cloud, normals) == 0)
	{
		cout << "Failed to read the file." << endl;
		return 0;
	}

	vector<Rock> rocks = getRocks(cloud, normals);
	segment_rocks(rocks);

	return 1;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
