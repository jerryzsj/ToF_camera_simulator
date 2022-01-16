#include "read_stl.h"

using namespace std;

v3::v3(char* facet)
{
	char f1[4] = {facet[0], facet[1], facet[2], facet[3]};
	char f2[4] = {facet[4], facet[5], facet[6], facet[7]};
	char f3[4] = {facet[8], facet[9], facet[10], facet[11]};

	float xx = *((float*) f1);
	float yy = *((float*) f2);
	float zz = *((float*) f3);

	m_x = double(xx);
	m_y = double(yy);
	m_z = double(zz);
}

tri::tri(v3 p1, v3 p2, v3 p3)
{

	m_p1 = v3(p1.m_x, p1.m_y, p1.m_z); 
	m_p2 = v3(p2.m_x, p2.m_y, p2.m_z); 
	m_p3 = v3(p3.m_x, p3.m_y, p3.m_z);
}

void read_stl(string fname, std::vector<tri>&v)
{
	ifstream myFile(fname.c_str(), ios::in | ios::binary);

	char header_info[80] = "";
	char nTri[4];
	unsigned long nTriLong;

	//read 80 byte header
	if(myFile){
		myFile.read (header_info, 80);
		cout << "header: " << header_info << endl;
	}
	else{
		cout << "ERROR READING FILE" << endl;
	}

	//read 4-byte ulong
	if(myFile) {
		myFile.read (nTri, 4);
		nTriLong = *((unsigned long*)nTri);
		cout << "n Tri: " << nTriLong << endl;
	}
	else {
		cout << "ERROR READING TRIANGEL" << endl;
	}

	//now read in all the triangles
	for(int i = 0; i < int(nTriLong) ; i++)
	{
		char facet[50];
		if(myFile) {
			myFile.read(facet, 50);
			v3 p1(facet+12);
			v3 p2(facet+24);
			v3 p3(facet+36);

			//add a new triangle to the array
			v.push_back(tri(p1, p2, p3));
		}

	}
	return;

}