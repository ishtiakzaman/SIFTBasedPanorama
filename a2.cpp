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
CImg<unsigned char> transform_image(CImg<unsigned char> &input_image, CImg<double> &M)
{	
	M.invert(true);

	CImg<unsigned char> output_image(input_image.width(), input_image.height(), 1, 3, 0);
	double w;
	int xsource, ysource;

	for (int x = 0; x < output_image.width(); ++x)
	{
		for(int y = 0; y < output_image.height(); ++y)
		{
			w = x*M(0,2)+y*M(1,2)+M(2,2);
			xsource = (int) ((x*M(0,0)+y*M(1,0)+M(2,0)) / w + 0.5);
			ysource = (int) ((x*M(0,1)+y*M(1,1)+M(2,1)) / w + 0.5);

			for (int p = 0; p < 3; ++p)			
				if (!(xsource < 0 || ysource < 0 || xsource >= input_image.width() || ysource >= input_image.height()))
					output_image(x, y, 0, p) = input_image(xsource, ysource, 0, p);			
		}
	}
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








