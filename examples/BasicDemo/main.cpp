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
//#include "../Utils/b3Clock.h"

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
#include <ctime>
#include <cmath>

using namespace std;



double rand_double1() {
	double x = (double) rand() / RAND_MAX;
	//cout << "rand_double1 " << x << endl;
	return x;
}

set<int> roll(vector<btVector3> vertices){
	DummyGUIHelper noGfx;
	CommonExampleOptions options(&noGfx);
	CommonExampleInterface* example = BasicExampleCreateFunc(options);
	
	example->vertexPoints(vertices);
	example->initPhysics();
	do{
		example->stepSimulation(1.f/60.f);
	}while(!example->DiceIsStill());

	set<int> face = example->checkFace();
	example->exitPhysics();
	delete example;
	return face;
}


//return true if two sets of vertex indices represent the same face
bool faceMatch(set<int> face1,set<int> face2){
	//if(face1.size() != 4 || face2.size() != 4) return false;
	for(int f1:face1){
		if(face2.find(f1) == face2.end()) return false;
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
        if (vals[i + 1] - vals[i] > d) {
            d = vals[i + 1] - vals[i];
        }
    }
    return d/2;
}

double calcDiscrepancy(vector<btVector3> vertices,vector<set<int> > faces)
{
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
				//cout << "matched with face: " << (j+1) << endl;
				//cout << "=========================" << endl;
			}
		}
		if(!matched) {
			probs[faces.size()-1]++;
			//++bad_count;
			//cout << "NOT MATCHED" <<endl;
			//for(int f:face_landed) cout << (f+1) << ",";
			cout << endl;
		}
	}
	
	//cout << "bad_count: " << bad_count << endl;
	//roll_count -= bad_count;
	for(i = 0; i < faces.size(); i++){
		probs[i] = probs[i]/roll_count;
		cout << "calculated probs for each side: " << probs[i] << endl;
	}

	/*
	vector<double> probs_copy;
	for(i = 0; i < 6; i++) probs_copy.push_back(probs[i]);
	sort(probs_copy.begin(),probs_copy.end(),greater<double>());


	for(i = 0; i < 6; i++){
		int j;
		for(j = 0; j < 6; j++){
			if(abs(probs[j]-probs_copy[i]) < 1e-4) cout << (j+1) << ",";
		}
	}
	cout << endl;
	*/
	return getDiscrepancy(probs);
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

	float height = 0.1;
	int n = 5;
	pair<float, float> pt[6];
	/*{{0.1,0},
				{0.1,M_PI*2.0f/(n-1)},
				{0.1, 2 * M_PI * 2.0f/(n-1)},
				{0.1, 3 * M_PI * 2.0f/(n-1)},
				{0.1, 4 * M_PI * 2.0f/(n-1)}};
	*/

	float best_w,best_h;
	int side_max_counts[6];
	for(i = 0; i <6; i++) side_max_counts[i] = 0;

	for(i = 0; i < simulation_count; i++){
		cout << "please enter h" << endl;
		
		float height = 0.1;
		cin >> height;
		cout << height << endl;
		cout << "enter (r,\\theta) in {meter, degree} for five points" << endl;
		for(int i = 1 ; i < n; ++ i) {
			cin >> pt[i].first >> pt[i].second;
			pt[i].second *= (M_PI/180);
			cout << pt[i].first <<" "<< pt[i].second << endl;
		}

		btVector3 v[6];
		v[0].setX(0.0f);
		v[0].setY(height);
		v[0].setZ(0.0f);

		for (int i = 1 ; i < n; ++ i) {
			float x,y;
			x = pt[i].first * cos(pt[i].second);
			y = pt[i].first * sin(pt[i].second);
			v[i].setX(x);
			v[i].setY(0);
			v[i].setZ(y);
		}
		/*
		double width = 1;
		double height = 1;
		double length = 1;
		btVector3 v[];
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
		


		vector<int> f1;
		f1.push_back(0);
		f1.push_back(4);
		f1.push_back(7);
		f1.push_back(3);
		vector<int> f2;
		f2.push_back(0);
		f2.push_back(1);
		f2.push_back(5);
		f2.push_back(4);
		vector<int> f3;
		f3.push_back(0);
		f3.push_back(3);
		f3.push_back(2);
		f3.push_back(1);
		vector<int> f4;
		f4.push_back(3);
		f4.push_back(7);
		f4.push_back(6);
		f4.push_back(2);
		vector<int> f5;
		f5.push_back(6);
		f5.push_back(7);
		f5.push_back(4);
		f5.push_back(5);
		vector<int> f6;
		f6.push_back(6);
		f6.push_back(5);
		f6.push_back(1);
		f6.push_back(2);
		*/


		
		vector<btVector3> vv;
		for( int i = 0 ; i < n ; ++ i) {
			vv.push_back(v[i]);
		}
		
		vector<set<int> > faces;
		set<int> f1;
		f1.insert(0);
		f1.insert(4);
		f1.insert(5);
		faces.push_back(f1);
		set<int> f2;
		f2.insert(0);
		f2.insert(5);
		f2.insert(1);
		faces.push_back(f2);
		set<int> f3;
		f3.insert(0);
		f3.insert(1);
		f3.insert(2);
		faces.push_back(f3);
		set<int> f4;
		f4.insert(3);
		f4.insert(0);
		f4.insert(2);
		faces.push_back(f4);
		set<int> f5;
		f5.insert(4);
		f5.insert(0);
		f5.insert(3);
		faces.push_back(f5);

		set<int> f6;
		f6.insert(5);
		f6.insert(4);
		f6.insert(3);
		f6.insert(2);
		f6.insert(1);
		faces.push_back(f6);
		
		/*
		vector<set<int> > faces;
		set<int> f1;
		f1.insert(0);
		f1.insert(4);
		f1.insert(1);
		faces.push_back(f1);
		set<int> f3;
		f3.insert(0);
		f3.insert(1);
		f3.insert(2);
		faces.push_back(f3);
		set<int> f4;
		f4.insert(3);
		f4.insert(0);
		f4.insert(2);
		faces.push_back(f4);
		set<int> f5;
		f5.insert(4);
		f5.insert(0);
		f5.insert(3);
		faces.push_back(f5);

		set<int> f6;
		f6.insert(4);
		f6.insert(3);
		f6.insert(2);
		f6.insert(1);
		faces.push_back(f6);
		*/
		double d = calcDiscrepancy(vv,faces);
		/*
		if(d < min_discrepancy){
			min_discrepancy = d;
			best_h = h;
			best_w = w;
		}
		*/
		
		cout << "calculated discrepancy: " << d << endl;
		
	}


	//cout << "best_h: " << best_h << endl;
	//cout << "best_b: " << best_b << endl;
	return 0;
}