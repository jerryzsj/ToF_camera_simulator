// #include "read_stl.h"

#include <string>
#include <vector>
#include <iostream>
#include <utility>
#include <fstream>

using namespace std;


int main(int argc, char const *argv[])
{
	string fname = "../stl_models/compresor_cover.stl";
	
	std::vector<char> array;
	array.reserve(10);
	char c = 0;
	while(c != 'x'){
		cin >> c;
		array.push_back(c);
	}
	cout << c << endl;
	cout << array.capacity() << endl;
	cout << array.size() << endl;

	int z =0;
	int &x = z;
	cout << "&x = z " << x << endl;
	int *y;
	// cout << "y before y =&x " << *y << endl;
	y = &x;
	cout << "y after y =&x " << *y << endl;
	x = 2;
	cout << "z after x = 2 "<< z << endl;


	return 0;
}

