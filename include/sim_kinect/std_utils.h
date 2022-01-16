#ifndef STD_UTILS_H_
#define STD_UTILS_H_

#include <iostream>
#include <string>
#include <list>

using namespace std;

const string INTERVAL = "***--------------------------------------***\n";

void cout_100per()
{
	for (int i =0; i<100;i++)
	{
		cout << "-";
	}
	cout << endl;
	return;
}

void cout_1per(int total_iter, int iter)
{
	if (iter/(total_iter/100)==0)
		cout << "*";
	if(iter == total_iter)
		cout << endl;
	return;
}


#endif