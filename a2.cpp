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

double matrix_3x3_determinant(vector< vector<double> > &M)
{
	return 	M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1]) +
				M[0][1]*(M[1][2]*M[2][0]-M[1][0]*M[2][2]) + 
					M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0]);
}

vector< vector<double> > matrix_3x3_inverse(vector< vector<double> > &M)
{
	double det = matrix_3x3_determinant(M);
	if ( abs(det) < 0.0000001 )
	{
		cout << "Cannot inverse matrix with determinant zero!!!" << endl;
		exit(1);
	}

	vector< vector<double> > R(3, vector<double>(3));
	R[0][0] = (M[1][1]*M[2][2]-M[1][2]*M[2][1])/det;
	R[0][1] = (M[0][2]*M[2][1]-M[0][1]*M[2][2])/det;
	R[0][2] = (M[0][1]*M[1][2]-M[0][2]*M[1][1])/det;
	R[1][0] = (M[1][2]*M[2][0]-M[1][0]*M[2][2])/det;
	R[1][1] = (M[0][0]*M[2][2]-M[0][2]*M[2][0])/det;
	R[1][2] = (M[0][2]*M[1][0]-M[0][0]*M[1][2])/det;
	R[2][0] = (M[1][0]*M[2][1]-M[1][1]*M[2][0])/det;
	R[2][1] = (M[0][1]*M[2][0]-M[0][0]*M[2][1])/det;
	R[2][2] = (M[0][0]*M[1][1]-M[0][1]*M[1][0])/det;

	return R;
}

CImg<double> transform_image(CImg<double> &input_image, vector< vector<double> > &M)
{	
	vector< vector<double> > R = matrix_3x3_inverse(M);

	CImg<double> output_image(input_image.width(), input_image.height(), 1, 3, 0);
	double w;
	int xsource, ysource;

	for (int x = 0; x < output_image.width(); ++x)
	{
		for(int y = 0; y < output_image.height(); ++y)
		{
			w = x*R[2][0]+y*R[2][1]+R[2][2];
			xsource = int( (x*R[0][0]+y*R[0][1]+R[0][2]) / w);
			ysource = int( (x*R[1][0]+y*R[1][1]+R[1][2]) / w);

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
			// This is just a bit of sample code to get you started, to
			// show how to use the SIFT library.

			CImg<double> input_image(inputFile.c_str());

			// convert image to grayscale
			CImg<double> gray = input_image.get_RGBtoHSI().get_channel(2);
			vector<SiftDescriptor> descriptors = Sift::compute_sift(gray);

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
		}
		else if(part == "part2")
		{
			CImg<double> input_image(inputFile.c_str());

			vector< vector<double> > M(3, vector<double>(3));
			M[0][0] = 0.907; 		M[0][1] = 0.258; 		M[0][2] = -182;
			M[1][0] = -0.153; 		M[1][1] = 1.44; 		M[1][2] = 58;
			M[2][0] = -0.000306; 	M[2][1] = 0.000731; 	M[2][2] = 1;

			CImg<double> trans_image = transform_image(input_image, M);
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








