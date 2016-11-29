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

//#include "btBulletDynamicsCommon.h"
#define ARRAY_SIZE_Y 5
#define ARRAY_SIZE_X 5
#define ARRAY_SIZE_Z 5

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btQuaternion.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include <vector>
#include <set>
#include <iostream>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <random>
#include <unistd.h>

using namespace std;

struct BasicExample : public CommonRigidBodyBase
{
	BasicExample(struct GUIHelperInterface* helper)
		:CommonRigidBodyBase(helper)
	{
	}
	virtual ~BasicExample(){}
	virtual void initPhysics();
	virtual void renderScene();

	void vertexPoints(vector<btVector3> vertices);
	void printVertices();
	bool DiceIsStill();
	set<int> checkFace();
	btVector3 getCenterOfMass();
	double eps = 1e-7;

	void resetCamera()
	{
		float dist = 2;
		float pitch = 52;
		float yaw = 35;
		float targetPos[3]={0,0,0};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
	}
};





typedef struct {
  int numVerts;
  double norm[3];
  double w;
  int verts[10];
  struct polyhedron *poly;
} FACE;

typedef struct polyhedron {
  int numVerts, numFaces;
  double verts[100][3];
  FACE faces[100];
} POLYHEDRON;
class Mirtich {
	static const int MAX_VERTS =100;     /* maximum number of polyhedral vertices */
	static const int MAX_FACES =100;     /* maximum number of polyhedral faces */
	static const int MAX_POLYGON_SZ =10; /* maximum number of verts per polygonal face */

	static const int X =0;
	static const int Y =1;
	static const int Z =2;

	/*
	   ============================================================================
	   macros
	   ============================================================================
	*/

	#define SQR(x) ((x)*(x))
	#define CUBE(x) ((x)*(x)*(x))

	/*
	   ============================================================================
	   data structures
	   ============================================================================
	*/
	public:
	/*
	   ============================================================================
	   globals
	   ============================================================================
	*/

	int A;   /* alpha */
	int B;   /* beta */
	int C;   /* gamma */

	/* projection integrals */
	double P1, Pa, Pb, Paa, Pab, Pbb, Paaa, Paab, Pabb, Pbbb;

	/* face integrals */
	double Fa, Fb, Fc, Faa, Fbb, Fcc, Faaa, Fbbb, Fccc, Faab, Fbbc, Fcca;

	/* volume integrals */
	double T0, T1[3], T2[3], TP[3];

	POLYHEDRON *p;
	/*
	   ============================================================================
	   read in a polyhedron
	   ============================================================================
	*/

	Mirtich(vector<vector<int> > faces, vector<btVector3> verts)
	{
		//cout<<"constructor"<<endl;
		p = new POLYHEDRON();
		p->numVerts = verts.size();


	  char line[200], *c;
	  int i, j, n;
	  double dx1, dy1, dz1, dx2, dy2, dz2, nx, ny, nz, len;
	  FACE *f;

	  
	  for (i = 0; i < p->numVerts; i++) {
	  	p->verts[i][X] = verts[i].getX();
	  	p->verts[i][Y] = verts[i].getY();
	  	p->verts[i][Z] = verts[i].getZ();
	  }
	  p->numFaces = faces.size();

	  for (i = 0; i < p->numFaces; i++) {
	    f = &p->faces[i];
	    f->poly = p;
	    f->numVerts = faces[i].size();
	    for (j = 0; j < f->numVerts; j++) f->verts[j] = faces[i][j];

	    /* compute face normal and offset w from first 3 vertices */
	    dx1 = p->verts[f->verts[1]][X] - p->verts[f->verts[0]][X];
	    dy1 = p->verts[f->verts[1]][Y] - p->verts[f->verts[0]][Y];
	    dz1 = p->verts[f->verts[1]][Z] - p->verts[f->verts[0]][Z];
	    dx2 = p->verts[f->verts[2]][X] - p->verts[f->verts[1]][X];
	    dy2 = p->verts[f->verts[2]][Y] - p->verts[f->verts[1]][Y];
	    dz2 = p->verts[f->verts[2]][Z] - p->verts[f->verts[1]][Z];
	    nx = dy1 * dz2 - dy2 * dz1;
	    ny = dz1 * dx2 - dz2 * dx1;
	    nz = dx1 * dy2 - dx2 * dy1;
	    len = sqrt(nx * nx + ny * ny + nz * nz);
	    f->norm[X] = nx / len;
	    f->norm[Y] = ny / len;
	    f->norm[Z] = nz / len;
	    f->w = - f->norm[X] * p->verts[f->verts[0]][X]
	           - f->norm[Y] * p->verts[f->verts[0]][Y]
	           - f->norm[Z] * p->verts[f->verts[0]][Z];

	  }

	  /*
	  for (i = 0; i < p->numVerts; i++) {
	  	cout<<p->verts[i][X]<<" ";// = verts[i][X];
	  	cout<<p->verts[i][Y]<<" ";// = verts[i][Y];
	  	cout<<p->verts[i][Z]<<endl;// = verts[i][Z];
	  }
	  cout<<faces.size()<<endl;
	  for(i = 0; i < faces.size(); ++i) {
	  	cout<<faces[i].size()<<" ";
	  	for(int j = 0 ; j < faces[i].size(); ++j) cout<<faces[i][j]<<" ";
	  	cout<<endl;
	  }
	  */
	}

	/*
	   ============================================================================
	   compute mass properties
	   ============================================================================
	*/

	    ~Mirtich() {
	    	//cout<<"destructor"<<endl;
	   		delete p;
	    }
	/* compute various integrations over projection of face */
	void compProjectionIntegrals(FACE *f)
	{
	  double a0, a1, da;
	  double b0, b1, db;
	  double a0_2, a0_3, a0_4, b0_2, b0_3, b0_4;
	  double a1_2, a1_3, b1_2, b1_3;
	  double C1, Ca, Caa, Caaa, Cb, Cbb, Cbbb;
	  double Cab, Kab, Caab, Kaab, Cabb, Kabb;
	  int i;

	  P1 = Pa = Pb = Paa = Pab = Pbb = Paaa = Paab = Pabb = Pbbb = 0.0;

	  for (i = 0; i < f->numVerts; i++) {
	    a0 = f->poly->verts[f->verts[i]][A];
	    b0 = f->poly->verts[f->verts[i]][B];
	    a1 = f->poly->verts[f->verts[(i+1) % f->numVerts]][A];
	    b1 = f->poly->verts[f->verts[(i+1) % f->numVerts]][B];
	    da = a1 - a0;
	    db = b1 - b0;
	    a0_2 = a0 * a0; a0_3 = a0_2 * a0; a0_4 = a0_3 * a0;
	    b0_2 = b0 * b0; b0_3 = b0_2 * b0; b0_4 = b0_3 * b0;
	    a1_2 = a1 * a1; a1_3 = a1_2 * a1; 
	    b1_2 = b1 * b1; b1_3 = b1_2 * b1;

	    C1 = a1 + a0;
	    Ca = a1*C1 + a0_2; Caa = a1*Ca + a0_3; Caaa = a1*Caa + a0_4;
	    Cb = b1*(b1 + b0) + b0_2; Cbb = b1*Cb + b0_3; Cbbb = b1*Cbb + b0_4;
	    Cab = 3*a1_2 + 2*a1*a0 + a0_2; Kab = a1_2 + 2*a1*a0 + 3*a0_2;
	    Caab = a0*Cab + 4*a1_3; Kaab = a1*Kab + 4*a0_3;
	    Cabb = 4*b1_3 + 3*b1_2*b0 + 2*b1*b0_2 + b0_3;
	    Kabb = b1_3 + 2*b1_2*b0 + 3*b1*b0_2 + 4*b0_3;

	    P1 += db*C1;
	    Pa += db*Ca;
	    Paa += db*Caa;
	    Paaa += db*Caaa;
	    Pb += da*Cb;
	    Pbb += da*Cbb;
	    Pbbb += da*Cbbb;
	    Pab += db*(b1*Cab + b0*Kab);
	    Paab += db*(b1*Caab + b0*Kaab);
	    Pabb += da*(a1*Cabb + a0*Kabb);
	  }

	  P1 /= 2.0;
	  Pa /= 6.0;
	  Paa /= 12.0;
	  Paaa /= 20.0;
	  Pb /= -6.0;
	  Pbb /= -12.0;
	  Pbbb /= -20.0;
	  Pab /= 24.0;
	  Paab /= 60.0;
	  Pabb /= -60.0;
	}

	void compFaceIntegrals(FACE *f)
	{
	  double *n, w;
	  double k1, k2, k3, k4;

	  compProjectionIntegrals(f);

	  w = f->w;
	  n = f->norm;
	  k1 = 1 / n[C]; k2 = k1 * k1; k3 = k2 * k1; k4 = k3 * k1;

	  Fa = k1 * Pa;
	  Fb = k1 * Pb;
	  Fc = -k2 * (n[A]*Pa + n[B]*Pb + w*P1);

	  Faa = k1 * Paa;
	  Fbb = k1 * Pbb;
	  Fcc = k3 * (SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb
		 + w*(2*(n[A]*Pa + n[B]*Pb) + w*P1));

	  Faaa = k1 * Paaa;
	  Fbbb = k1 * Pbbb;
	  Fccc = -k4 * (CUBE(n[A])*Paaa + 3*SQR(n[A])*n[B]*Paab 
		   + 3*n[A]*SQR(n[B])*Pabb + CUBE(n[B])*Pbbb
		   + 3*w*(SQR(n[A])*Paa + 2*n[A]*n[B]*Pab + SQR(n[B])*Pbb)
		   + w*w*(3*(n[A]*Pa + n[B]*Pb) + w*P1));

	  Faab = k1 * Paab;
	  Fbbc = -k2 * (n[A]*Pabb + n[B]*Pbbb + w*Pbb);
	  Fcca = k3 * (SQR(n[A])*Paaa + 2*n[A]*n[B]*Paab + SQR(n[B])*Pabb
		 + w*(2*(n[A]*Paa + n[B]*Pab) + w*Pa));
	}

	void compVolumeIntegrals(POLYHEDRON *p)
	{
	  FACE *f;
	  double nx, ny, nz;
	  int i;

	  T0 = T1[X] = T1[Y] = T1[Z] 
	     = T2[X] = T2[Y] = T2[Z] 
	     = TP[X] = TP[Y] = TP[Z] = 0;

	  for (i = 0; i < p->numFaces; i++) {

	    f = &p->faces[i];

	    nx = fabs(f->norm[X]);
	    ny = fabs(f->norm[Y]);
	    nz = fabs(f->norm[Z]);
	    if (nx > ny && nx > nz) C = X;
	    else C = (ny > nz) ? Y : Z;
	    A = (C + 1) % 3;
	    B = (A + 1) % 3;

	    compFaceIntegrals(f);

	    T0 += f->norm[X] * ((A == X) ? Fa : ((B == X) ? Fb : Fc));

	    T1[A] += f->norm[A] * Faa;
	    T1[B] += f->norm[B] * Fbb;
	    T1[C] += f->norm[C] * Fcc;
	    T2[A] += f->norm[A] * Faaa;
	    T2[B] += f->norm[B] * Fbbb;
	    T2[C] += f->norm[C] * Fccc;
	    TP[A] += f->norm[A] * Faab;
	    TP[B] += f->norm[B] * Fbbc;
	    TP[C] += f->norm[C] * Fcca;
	  }

	  T1[X] /= 2; T1[Y] /= 2; T1[Z] /= 2;
	  T2[X] /= 3; T2[Y] /= 3; T2[Z] /= 3;
	  TP[X] /= 2; TP[Y] /= 2; TP[Z] /= 2;
	}


	/*
	   ============================================================================
	   main
	   ============================================================================
	*/


	void calculate_com_and_inertia(btVector3 &iner, btVector3 &com)
	{
	  //cout<<"here"<<std::endl;
	  double density, mass;
	  double r[3];            /* center of mass */
	  double J[3][3];         /* inertia tensor */


	  compVolumeIntegrals(p);

	  /*
	  printf("\nT1 =   %+20.6f\n\n", T0);

	  printf("Tx =   %+20.6f\n", T1[X]);
	  printf("Ty =   %+20.6f\n", T1[Y]);
	  printf("Tz =   %+20.6f\n\n", T1[Z]);
	  
	  printf("Txx =  %+20.6f\n", T2[X]);
	  printf("Tyy =  %+20.6f\n", T2[Y]);
	  printf("Tzz =  %+20.6f\n\n", T2[Z]);

	  printf("Txy =  %+20.6f\n", TP[X]);
	  printf("Tyz =  %+20.6f\n", TP[Y]);
	  printf("Tzx =  %+20.6f\n\n", TP[Z]);
	  */

	  density = 1.0;  /* assume unit density */

	  mass = density * T0;

	  /* compute center of mass */
	  r[X] = T1[X] / T0;
	  r[Y] = T1[Y] / T0;
	  r[Z] = T1[Z] / T0;

	  /* compute inertia tensor */
	  J[X][X] = density * (T2[Y] + T2[Z]);
	  J[Y][Y] = density * (T2[Z] + T2[X]);
	  J[Z][Z] = density * (T2[X] + T2[Y]);
	  J[X][Y] = J[Y][X] = - density * TP[X];
	  J[Y][Z] = J[Z][Y] = - density * TP[Y];
	  J[Z][X] = J[X][Z] = - density * TP[Z];

	  /* translate inertia tensor to center of mass */
	  J[X][X] -= mass * (r[Y]*r[Y] + r[Z]*r[Z]);
	  J[Y][Y] -= mass * (r[Z]*r[Z] + r[X]*r[X]);
	  J[Z][Z] -= mass * (r[X]*r[X] + r[Y]*r[Y]);
	  J[X][Y] = J[Y][X] += mass * r[X] * r[Y]; 
	  J[Y][Z] = J[Z][Y] += mass * r[Y] * r[Z]; 
	  J[Z][X] = J[X][Z] += mass * r[Z] * r[X]; 

	  //printf("center of mass:  (%+12.6f,%+12.6f,%+12.6f)\n\n", r[X], r[Y], r[Z]);
	  com.setX(r[X]);
	  com.setY(r[Y]);
	  com.setZ(r[Z]);
	  //printf("inertia tensor with origin at c.o.m. :\n");
	  //printf("%+15.6f  %+15.6f  %+15.6f\n", J[X][X], J[X][Y], J[X][Z]);
	  //printf("%+15.6f  %+15.6f  %+15.6f\n", J[Y][X], J[Y][Y], J[Y][Z]);
	  //printf("%+15.6f  %+15.6f  %+15.6f\n\n", J[Z][X], J[Z][Y], J[Z][Z]);
	  /*
	  for(int i = 0 ; i < 3; ++ i) {
	  	vector<double> inr;
	  	for(int j = 0 ; j < 3; ++j) {
	  		inr.push_back(J[i][j]);
	  	}
	  	iner.push_back(inr);
	  
	  }
	  */
	  iner.setX(J[X][X]);
	  iner.setY(J[Y][Y]);
	  iner.setZ(J[Z][Z]);

	  //cout << r[X] << "," << r[Y] << "," << r[Z] << endl;
	  //cout << J[X][X] << "," << J[Y][Y] << "," << J[Z][Z] << endl;
	}
};




double rand_double() {
	double x = (double) rand() / RAND_MAX;
	//cout << x * RAND_MAX << endl;
	return x;
}

void BasicExample::initPhysics()
{
	m_guiHelper->setUpAxis(1);

	createEmptyDynamicsWorld();
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe+btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(5000.),btScalar(10.),btScalar(5000.)));
	
	groundShape->initializePolyhedralFeatures();
	//btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,1,0),50);
	
	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,-10,0));

	{
		btScalar mass(0.);
		createRigidBody(mass,groundTransform,groundShape, btVector4(0,0,1,1));
	}


	random_device rand_dev;
	mt19937 generator(rand_dev());
	uniform_real_distribution<double> distr1(-1,1);
	normal_distribution<double> distr2(0,1);

    //create arbitrary shape convex polyhrdron shaped dice
    {	
    	btCompoundShape* compoundShape = new btCompoundShape();
        btConvexHullShape* colShape = new btConvexHullShape();
        m_collisionShapes.push_back((btCollisionShape *)colShape);
        
        int i;
        for(i = 0; i < 8; i++){
        	btVector3 vertex = this->vertices[i];
        	colShape->addPoint(vertex,true);
        }

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);


		//cout << "localInertia: ";
		btVector3 localInertia(0,0,0);

		
		int max_height = 0;
		double height = 1 + rand_double()*max_height;
		startTransform.setOrigin(btVector3(	btScalar(0),
										btScalar(height),
										btScalar(0)));
		
       	btTransform localTransform;
        localTransform.setIdentity();
        localTransform.setOrigin((-1)*getCenterOfMass());
        compoundShape->addChildShape(localTransform, colShape);

        if (isDynamic)
			colShape->calculateLocalInertia(mass,localInertia);

		
	    double w = distr1(generator)*M_PI;
	   	double x = distr2(generator);
	   	double y = distr2(generator);
	    double z = distr2(generator);
	   	btQuaternion quat(btVector3(x,y,z),w);
	   	startTransform.setRotation(quat);
	    this->dice = createRigidBody(mass, startTransform, compoundShape);
    }
        
    int max_speed = 10;
    double max_rad = M_PI*10;
    btVector3 angular = btVector3(distr1(generator),distr1(generator),distr1(generator));
    angular *= max_rad;
	btVector3 speed = btVector3(distr1(generator),distr1(generator),distr1(generator));
	speed *= max_speed;

   	this->dice->setAngularVelocity(angular);
   	this->dice->setLinearVelocity(speed);
	
    //cout << "angular velocity: " << angular.getX() << "," << angular.getY() << "," << angular.getZ() << endl;
    //cout << "velocity: " << speed.getX() << "," << speed.getY() << "," << speed.getZ() << endl;
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);	
}



void BasicExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}


btVector3 BasicExample::getCenterOfMass(){
	vector<vector<int> > faces;
	vector<int> f1;
	f1.push_back(0);
	f1.push_back(4);
	f1.push_back(5);
	vector<int> f2;
	f2.push_back(0);
	f2.push_back(5);
	f2.push_back(1);
	vector<int> f3;
	f3.push_back(0);
	f3.push_back(1);
	f3.push_back(2);
	vector<int> f4;
	f4.push_back(3);
	f4.push_back(0);
	f4.push_back(2);
	vector<int> f5;
	f5.push_back(4);
	f5.push_back(0);
	f5.push_back(3);
	vector<int> f6;
	f6.push_back(5);
	f6.push_back(4);
	f6.push_back(3);
	f6.push_back(2);
	f6.push_back(1);


	faces.push_back(f1);
	faces.push_back(f2);
	faces.push_back(f3);
	faces.push_back(f4);
	faces.push_back(f5);
	faces.push_back(f6);
	
	Mirtich m(faces,this->vertices);
	btVector3 com, iner;
	m.calculate_com_and_inertia(iner, com);
	return com;
}

//pass in vertex locations to create the dice
void BasicExample::vertexPoints(vector<btVector3> vertices){
	this->vertices = vertices;
}

void BasicExample::printVertices(){	
	int i;
	for(i = 0; i < this->vertices.size(); i++){
		btVector3 vertex = vertices[i];
		cout << vertex.x() << "," << vertex.y() << "," << vertex.z() << endl;
	}
}


bool BasicExample::DiceIsStill(){
	const btTransform& transfrom = this->dice->getWorldTransform();
	int i;
	for(i = 0; i < 8; i++){
		btVector3 endPosition = transfrom*this->vertices[i];
		if(endPosition.getY() < -10) return true;
	}
	const btVector3&  v = this->dice->getLinearVelocity();

	return abs(v.x()) < this->eps && abs(v.y()) < this->eps && abs(v.z()) < this->eps;
}

set<int> BasicExample::checkFace(){
	set<int> res;

	const btTransform& transfrom = this->dice->getWorldTransform();

	vector<double> heights;
	int i;
	for(i = 0; i < 8; i++){
		btVector3 endPosition = transfrom*this->vertices[i];
		if(endPosition.getY() < -10) return res;
		heights.push_back(endPosition.getY());
		//cout << endPosition.getX() << "," << endPosition.getY() << "," << endPosition.getZ() << endl;
	}
	
	//find the lowest 4 vertices
	for(i = 0; i < 4; i++){
		double curr_min = 10000000000; 
		int min_vertex = -1;
		int j;
		for(j = 0; j < 8; j++){
			if(res.find(j) != res.end()) continue;
			if(heights[j] < curr_min){
				curr_min = heights[j];
				min_vertex = j;
			}
		}
		res.insert(min_vertex);
	}

	/*
	for(i = 0; i < 8; i++){
		btVector3 endPosition = transfrom*this->vertices[i];
		cout << "old pos: " << this->vertices[i].getY() << ", ";
		cout << "current pos: " << endPosition.getY() << ", " << endl;
	}
	cout << "checkFace" << endl;
	*/

	return res;
}


CommonExampleInterface*    BasicExampleCreateFunc(CommonExampleOptions& options)
{
	return new BasicExample(options.m_guiHelper);

}


B3_STANDALONE_EXAMPLE(BasicExampleCreateFunc)



