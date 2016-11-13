/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "BasicExample.h"

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCollisionShape.h"
//#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

#include "LinearMath/btTransform.h"
#include "LinearMath/btHashMap.h"
#include "LinearMath/btVector3.h"


#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"

#include <vector>
#include <iostream>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <set>
#include <cstdlib>
#include <ctime>m

using namespace std;


set<int> roll(vector<btVector3> vertices){
	DummyGUIHelper noGfx;
	CommonExampleOptions options(&noGfx);
	CommonExampleInterface* example = BasicExampleCreateFunc(options);
	
	example->vertexPoints(vertices);
	example->initPhysics();
	int iteration = 0;
	while(!example->checkDice()){
		example->stepSimulation(1.f/60.f);
		iteration++;
		//this_thread::sleep_for(chrono::milliseconds(1000));
	}

	//cout << "iterations passed: " << iteration << endl;
	//cout << "here" << endl;
	set<int> face = example->checkFace();
	example->exitPhysics();
	delete example;
	return face;
}


//return true if two sets of vertex indices represent the same face
bool faceMatch(set<int> faces1,set<int> faces2){
	set<int> copy = faces2;
	for(int f1:faces1){
		bool found = false;
		for(int f2:faces2){
			found = found || f1 == f2;
		}
		if(!found) return false;
	}
	return true;
}



double getDiscrepancy(vector<double> probs) {
    int n = probs.size();
    vector<double> vals;
    for (int i = 0; i < (1 << n); ++i) {
        double cur_prob = 0;
        for (int k = i, j = 0; k > 0; k >>= 1, ++j) {
            if (k & 1) {
                cur_prob += probs[j];
            }
        }
        vals.push_back(cur_prob);
    }
    double d = 0;
    sort(vals.begin(), vals.end());
    for (int i = 0; i < vals.size() - 1; ++i) {
    	//cout << vals[i] << endl;
        if (vals[i + 1] - vals[i] > d) {
            d = vals[i + 1] - vals[i];
        }
    }
    return d;
}

double calcDiscrepancy(vector<btVector3> vertices,vector<set<int> > faces)
{
	double res = 0;
	int side = 6;
	int roll_count = 10000;
	int i;
	vector<double> probs;
	for(i = 0; i < faces.size(); i++) probs.push_back(0);

	int bad_count = 0;
	//simualte dice rolling
	for(i = 0; i < roll_count; i++){
		set<int> face_landed = roll(vertices);
		int j;
		
		bool matched = false;
		for(j = 0; j < faces.size() ; j++){
			if(faceMatch(faces[j],face_landed)) {
				probs[j]++;
				matched = true;
			}
		}
		if(!matched) {
			++bad_count;
			//cout << "NOT MATCHED" <<endl;
			//cout << face_landed.size() << endl;
		}
	}
	roll_count -= bad_count;
	for(i = 0; i < faces.size(); i++){
		probs[i] = probs[i]/roll_count;
		cout << "calculated probs for each side: " << probs[i] << endl;
	}
	cout << endl;

	return getDiscrepancy(probs);
}

double rand_double1() {
	double x = (double) rand() / RAND_MAX;
	return x;
}

int main(int argc, char* argv[])
{
	srand(time(NULL));
	int simulation_count = 1;
	int i;
	double min_discrepancy = 1;
	
	float l = 1;
	float h = 1.22249;
	float w = 1.44929;

	float best_w,best_h;
	for(i = 0; i < simulation_count; i++){
		//float l = 1;
		//float b = rand_double1() + 1;
		//float h = rand_double1() + 1;

		cout << "please enter l h w" << endl;
		cin>>l>>h>>w;
		float length = l;
		float height = h;//1.5;
		float width = w;//1.75;
		btVector3 v0;
		v0.setX(0.0f);
		v0.setY(0.0f);
		v0.setZ(0.0f);
		btVector3 v1;
		v1.setX(0.0f);
		v1.setY(0.0f);
		v1.setZ(width);
		btVector3 v2;
		v2.setX(length);
		v2.setY(0.0f);
		v2.setZ(width);
		btVector3 v3;
		v3.setX(length);
		v3.setY(0.0f);
		v3.setZ(0.0f);
		btVector3 v4;
		v4.setX(0.0f);
		v4.setY(height);
		v4.setZ(0.0f);
		btVector3 v5;
		v5.setX(0.0f);
		v5.setY(height);
		v5.setZ(width);
		btVector3 v6;
		v6.setX(length);
		v6.setY(height);
		v6.setZ(width);
		btVector3 v7;
		v7.setX(length);
		v7.setY(height);
		v7.setZ(0.0f);

		vector<btVector3> v;
		v.push_back(v0);
		v.push_back(v1);
		v.push_back(v2);
		v.push_back(v3);
		v.push_back(v4);
		v.push_back(v5);
		v.push_back(v6);
		v.push_back(v7);

		vector<set<int> > faces;
		set<int> f1;
		f1.insert(0);
		f1.insert(4);
		f1.insert(7);
		f1.insert(3);
		faces.push_back(f1);
		set<int> f2;
		f2.insert(0);
		f2.insert(1);
		f2.insert(5);
		f2.insert(4);
		faces.push_back(f2);
		set<int> f3;
		f3.insert(0);
		f3.insert(3);
		f3.insert(2);
		f3.insert(1);
		faces.push_back(f3);
		set<int> f4;
		f4.insert(3);
		f4.insert(7);
		f4.insert(6);
		f4.insert(2);
		faces.push_back(f4);
		set<int> f5;
		f5.insert(6);
		f5.insert(7);
		f5.insert(4);
		f5.insert(5);
		faces.push_back(f5);
		set<int> f6;
		f6.insert(6);
		f6.insert(5);
		f6.insert(1);
		f6.insert(2);
		faces.push_back(f6);

		
		double d = calcDiscrepancy(v,faces);

		if(d < min_discrepancy){
			min_discrepancy = d;
			best_h = h;
			best_w = w;
		}
		
		cout << "calculated discrepancy: " << d << endl;
	}
	//cout << "best_h: " << best_h << endl;
	//cout << "best_b: " << best_b << endl;
	return 0;
}