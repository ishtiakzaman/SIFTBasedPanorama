// B657 assignment 2 skeleton code
//
// Compile with: "make"
//
// See assignment handout for command line and project specifications.


//Link to the header file
#include "CImg.h"
#include <ctime>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>
#include <Sift.h>

//Use the cimg namespace to access the functions easily
using namespace cimg_library;
using namespace std;

// Transform image using inverse warping
CImg<unsigned char> transform_image(CImg<unsigned char> &input_image, CImg<double> &M,
											int minx=0, int miny=0, int maxx=0, int maxy=0)
{	
	if (minx == 0 && miny == 0 && maxx == 0 && maxy == 0)
	{
		maxx = input_image.width()-1;
		maxy = input_image.height()-1;
	}

	CImg<unsigned char> output_image(maxx-minx+1, maxy-miny+1, 1, 3, 0);
	double w;
	int xsource, ysource;

	M.invert(true);

	for (int x = minx; x <= maxx; ++x)
	{
		for(int y = miny; y <= maxy; ++y)
		{
			w = x*M(0,2)+y*M(1,2)+M(2,2);
			xsource = (int) ((x*M(0,0)+y*M(1,0)+M(2,0)) / w + 0.5);
			ysource = (int) ((x*M(0,1)+y*M(1,1)+M(2,1)) / w + 0.5);

			for (int p = 0; p < 3; ++p)			
				if (!(xsource < 0 || ysource < 0 || xsource >= input_image.width() || ysource >= input_image.height()))
					output_image(x-minx, y-miny, 0, p) = input_image(xsource, ysource, 0, p);			
		}
	}
	return output_image;
}

CImg<double> ransac(vector< pair< pair<int,int>, pair<int,int> > > &matches, int steps, double threshold)
{
	int n = matches.size(), index, max_inliers = -1;
	int x, y, xp, yp;	
	double x1, y1, w;

	CImg<double> A(8, 8, 1, 1, 0), B(1, 8, 1, 1, 0), T(3, 3, 1, 1, 0), T_best;

	// do this #steps times
	while (steps--)
	{

		// Building the matrices for four points
		for (int i = 0; i < 4; ++i)
		{
			// Getting four points at random
			index = rand() % n;
			x = matches[index].first.first;
			y = matches[index].first.second;
			xp = matches[index].second.first;
			yp = matches[index].second.second;

			// Intialize A and B
			A(0,2*i) = x;	A(1,2*i) = y;	A(2,2*i) = 1;	A(6,2*i) = -x*xp;	A(7,2*i) = -y*xp;
			A(3,2*i+1) = x;	A(4,2*i+1) = y;	A(5,2*i+1) = 1;	A(6,2*i+1) = -x*yp;	A(7,2*i+1) = -y*yp;

			B(0,2*i) = xp;
			B(0,2*i+1) = yp;		
		}

		// Solve 8 linear equations with 8 unknown to get Transform value into B
		B.solve(A);

		// Get the B (1x8) matrix values into the tranform matrix T (3x3)
		for (int i = 0; i < 8; ++i)		
			T(i%3, i/3) = B(0,i);
		T(2,2) = 1;

		// Count number of inliers for this transformation
		int inliers = 0;		
		
		for (int i = 0; i < n; ++i)
		{
			x = matches[i].first.first;
			y = matches[i].first.second;
			xp = matches[i].second.first;
			yp = matches[i].second.second;

			w = x*T(0,2)+y*T(1,2)+T(2,2);
			x1 = (x*T(0,0)+y*T(1,0)+T(2,0)) / w;
			y1 = (x*T(0,1)+y*T(1,1)+T(2,1)) / w;

			if (abs(x1-xp) < threshold && abs(y1-yp) < threshold) // This point matches
				++inliers;			
		}

		// if this projection is current best projection
		if (inliers > max_inliers)
		{
			max_inliers = inliers;
			T_best = T;
		}
	}

	return T_best;
}

CImg<unsigned char> create_panaroma(CImg<unsigned char> &image1, CImg<unsigned char> &image2)
{
	double x1, y1, w;
	int x, y;
	vector< pair< pair<int,int>, pair<int,int> > > matches;
	CImg<double> T = ransac(matches, 75, 3.0);

	// Find the boundary we will get after stitching two images
	int minx = 0, miny = 0, maxx = image2.width(), maxy = image2.height();

	// Check for 4 corner points of first image after translation
	for (int i = 0; i < 4; ++i)
	{
		x1 = i%2?image1.width():0;
		y1 = i/2?image1.height():0;
		
		w = x1*T(0,2)+y1*T(1,2)+T(2,2);
		x = (x1*T(0,0)+y1*T(1,0)+T(2,0)) / w + 0.5;
		y = (x1*T(0,1)+y1*T(1,1)+T(2,1)) / w + 0.5;
		if (x < minx) minx = x;
		if (x > maxx) maxx = x;
		if (y < miny) miny = y;
		if (y > maxy) maxy = y;
	}		

	CImg<unsigned char> output_image = transform_image(image1, T, minx, miny, maxx, maxy);
	for (int x = 0; x < image2.width(); ++x)
		for (int y = 0; y < image2.height(); ++y)
			for(int p=0; p<3; p++)
				output_image(x-minx, y-miny, 0, p) = image2(x, y, 0, p);

	return output_image;
}


int main(int argc, char **argv)
{
	try 
	{
		if(argc < 2)
		{
			cout << "Insufficent number of arguments; correct usage:" << endl;
			cout << "    a2-p1 part_id ..." << endl;
			return -1;
		}

		string part = argv[1];
		string inputFile = argv[2];
		

		if(part == "part1")
		{
			string inputfile2 = argv[3];

			CImg<double> input_image(inputFile.c_str());

			// convert image to grayscale
			CImg<double> gray = input_image.get_RGBtoHSI().get_channel(2);
			vector<SiftDescriptor> descriptors = Sift::compute_sift(gray);
			
			cout <<"Desc size"<< descriptors.size();
			cout << "desc 1" << descriptors[1].descriptor[1];
			cout << endl;
			cout << endl;
			for(int i=0; i<descriptors.size(); i++)
			{
				cout << "Descriptor #" << i << ": x=" << descriptors[i].col << " y=" << descriptors[i].row << " descriptor=(";
				for(int l=0; l<128; l++)
					cout << descriptors[i].descriptor[l] << "," ;
				cout << ")" << endl;

				for(int j=0; j<5; j++)
					for(int k=0; k<5; k++)
						if(j==2 || k==2)
							for(int p=0; p<3; p++)								
			                    if(descriptors[i].col+k < input_image.width() && descriptors[i].row+j < input_image.height())
            				        input_image(descriptors[i].col+k, descriptors[i].row+j, 0, p) = 0;

			}
			input_image.get_normalize(0,255).save("sift.png");
			
			CImg<double> input_image1(inputfile2.c_str());
			// convert image to grayscale
			CImg<double> gray1 = input_image1.get_RGBtoHSI().get_channel(2);
			vector<SiftDescriptor> descriptors1 = Sift::compute_sift(gray1);
			
			cout <<"Desc size"<< descriptors1.size();
			cout << "desc 1" << descriptors1[1].descriptor[1];
			cout << endl;
			cout << endl;
			for(int i=0; i<descriptors1.size(); i++)
			{
				cout << "Descriptor #" << i << ": x=" << descriptors1[i].col << " y=" << descriptors1[i].row << " descriptor=(";
				for(int l=0; l<128; l++)
					cout << descriptors1[i].descriptor[l] << "," ;
				cout << ")" << endl;

				for(int j=0; j<5; j++)
					for(int k=0; k<5; k++)
						if(j==2 || k==2)
							for(int p=0; p<3; p++)								
			                    if(descriptors1[i].col+k < input_image1.width() && descriptors1[i].row+j < input_image1.height())
            				        input_image1(descriptors[i].col+k, descriptors[i].row+j, 0, p) = 0;

			}
			input_image1.get_normalize(0,255).save("sift1.png");
			
		}
		else if(part == "part2")
		{
			CImg<unsigned char> input_image(inputFile.c_str());
			
			CImg<double> M(3, 3, 1, 1, 0.0);
			M(0,0) = 0.907; 		M(1,0) = 0.258; 		M(2,0) = -182;
			M(0,1) = -0.153; 		M(1,1) = 1.44; 			M(2,1) = 58;
			M(0,2) = -0.000306; 	M(1,2) = 0.000731; 		M(2,2) = 1;

			CImg<unsigned char> trans_image = transform_image(input_image, M);
			trans_image.save("transformed.png");
		}
		else
			throw std::string("unknown part!");

		// feel free to add more conditions for other parts (e.g. more specific)
		//  parts, for debugging, etc.
	}
	catch(const string &err)
	{
		cerr << "Error: " << err << endl;
	}
}








