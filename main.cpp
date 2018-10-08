// Taken from https://github.com/ihmcrobotics/ihmc-open-robotics-software/blob/5f5345ea78f681c1ca815bb1539041b5d0ab54d0/ihmc-sensor-processing/csrc/ransac_schnabel/main.cpp

#include <PointCloud.h>
#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>
#include <CylinderPrimitiveShapeConstructor.h>
#include <SpherePrimitiveShapeConstructor.h>
#include <ConePrimitiveShapeConstructor.h>
#include <TorusPrimitiveShapeConstructor.h>

#include<iostream>



int main()
{
	PointCloud pc;

	// fill or load point cloud from file
	for(int i=0;i<100;i++){
		for(int j=0;j<100;j++){
			pc.push_back(Point(Vec3f(i,j,0)));
			if(fabs((i-50)*(i-50)+(j-50)*(j-50)-50)<5){
				//std::cout << i << "," << j << "\n";
				pc.push_back(Point(Vec3f(i,j,1)));
				pc.push_back(Point(Vec3f(i,j,2)));
				pc.push_back(Point(Vec3f(i,j,3)));
			}
		}
	}
	
	// set the bbox in pc
	pc.setBBox(Vec3f(-100,-100,-100), Vec3f(100,100,100));
	//void calcNormals( float radius, unsigned int kNN = 20, unsigned int maxTries = 100 );
	pc.calcNormals(3);


	std::cout << "added " << pc.size() << " points" << std::endl;

	RansacShapeDetector::Options ransacOptions;
	ransacOptions.m_epsilon = .2f * pc.getScale(); // set distance threshold to .01f of bounding box width
		// NOTE: Internally the distance threshold is taken as 3 * ransacOptions.m_epsilon!!!
	ransacOptions.m_bitmapEpsilon = .02f * pc.getScale(); // set bitmap resolution to .02f of bounding box width
		// NOTE: This threshold is NOT multiplied internally!
	ransacOptions.m_normalThresh = .9f; // this is the cos of the maximal normal deviation
	ransacOptions.m_minSupport = 10; // this is the minimal numer of points required for a primitive
	ransacOptions.m_probability = .001f; // this is the "probability" with which a primitive is overlooked

	RansacShapeDetector detector(ransacOptions); // the detector object

	// set which primitives are to be detected by adding the respective constructors
	detector.Add(new PlanePrimitiveShapeConstructor());
	detector.Add(new CylinderPrimitiveShapeConstructor());
	
	/*
	detector.Add(new SpherePrimitiveShapeConstructor());
	detector.Add(new CylinderPrimitiveShapeConstructor());
	detector.Add(new ConePrimitiveShapeConstructor());
	detector.Add(new TorusPrimitiveShapeConstructor());
	*/

	MiscLib::Vector< std::pair< MiscLib::RefCountPtr< PrimitiveShape >, size_t > > shapes; // stores the detected shapes
	size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes); // run detection
		// returns number of unassigned points
		// the array shapes is filled with pointers to the detected shapes
		// the second element per shapes gives the number of points assigned to that primitive (the support)
		// the points belonging to the first shape (shapes[0]) have been sorted to the end of pc,
		// i.e. into the range [ pc.size() - shapes[0].second, pc.size() )
		// the points of shape i are found in the range
		// [ pc.size() - \sum_{j=0..i} shapes[j].second, pc.size() - \sum_{j=0..i-1} shapes[j].second )

	std::cout << "remaining unassigned points " << remaining << std::endl;
	for(int i=0;i<shapes.size();i++)
	{
		std::string desc;
		shapes[i].first->Description(&desc);
		std::cout << "shape " << i << " consists of " << shapes[i].second << " points, it is a " << desc << std::endl;
	}
}
