// Strategy.cpp : Defines the entry point for the DLL application.
//

#include "stdafx.h"
#include "Strategy.h"
#include <math.h>

BOOL APIENTRY DllMain( HANDLE hModule,
                       DWORD  ul_reason_for_call,
                       LPVOID lpReserved
                     )
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}



const double PI = 3.1415923;

#define MaxVel	125
#define ATENUAR 0.5
static int lastX;
static int lastY;
static int cantFriendBlocked = 0;
int cantEnemyBlocked = 0;


//char myMessage[200]; //big enough???

void PredictBall ( Environment *env );
void Arquero ( Robot *robot, Environment *env );
void NearBound2 ( Robot *robot, double vl, double vr, Environment *env );
void Attack2 ( Robot *robot, Environment *env );
void Defend ( Robot *robot, double x, Environment *env);

void Attack (int robotIndex, Environment *env );
void FollowNearbyOpponent(int attackRobotIndex, int robotIndex, Environment *env);
// just for testing to check whether the &env->opponent works or not
void MoonFollowOpponent (  Robot *robot, OpponentRobot *opponent );

// Calcula la distancia entre dos puntos.
double Distancia(double x0, double y0, double xf, double yf);


void Velocity ( Robot *robot, int vl, int vr );
void Angle ( Robot *robot, int desired_angle);
void Position( Robot *robot, double x, double y );


extern "C" STRATEGY_API void Create ( Environment *env )
{
	// allocate user data and assign to env->userData
	// eg. env->userData = ( void * ) new MyVariables ();
}

extern "C" STRATEGY_API void Destroy ( Environment *env )
{
	// free any user data created in Create ( Environment * )

	// eg. if ( env->userData != NULL ) delete ( MyVariables * ) env->userData;
}


extern "C" STRATEGY_API void Strategy ( Environment *env )
{

	Defend ( &env->home [1], 75, env );
	Defend ( &env->home [2], 65, env );
	FollowNearbyOpponent(4, 3, env );
	Attack ( 4, env );
	//Attack ( 3, env );
	Arquero ( &env->home [0], env );

}

void Attack ( int robotIndex, Environment *env )
{
	Robot *robot = &env->home[robotIndex];


	//Velocity (robot, 127, 127);
	//Angle (robot, 45);
	//bool isCloseOponent = false;
	int diffX = 0;
	int diffY = 0;
	double dist = 0;

	//Se bloquea con oponentes
	/**
	for (int i = 0; i < 5; i++)
	{
		dist = Distancia(env->opponent[i].pos.x, env->opponent[i].pos.y, env->home[robotIndex].pos.x, env->home[robotIndex].pos.y);
		if (dist < 5)
		{
			cantEnemyBlocked++;
			//isCloseOponent = true;
			break;
		}
	}
	
	//Se bloquea con compañeros
	for (int j = 0; j < 5; j++)
	{
		if (robotIndex != j) {
			//diffX = env->home[j].pos.x - robot->pos.x;
			//diffY =  env->home[j].pos.y - robot->pos.y;
			dist = Distancia(env->opponent[j].pos.x, env->opponent[j].pos.y, env->home[robotIndex].pos.x, env->home[robotIndex].pos.y);
			if (dist < 5)
			{
				cantFriendBlocked++;
				//isCloseOponent = true;
				break;
			}
		}

	}
	

	//Retrocede por oponentes
	if (cantEnemyBlocked > 0)
	{
		Velocity(robot, -30, -130);
		Angle(robot, 90);
		cantEnemyBlocked = 0;

	}else if (cantFriendBlocked > 0)
	{
		Velocity(robot, -30, -130);
		Angle(robot, 90);
		cantFriendBlocked = 0;
	}else
	{
		PredictBall ( env );
		Position(robot, env->predictedBall.pos.x, env->predictedBall.pos.y);
	}
	
	*/


	if(lastX == robot->pos.x && lastY == robot->pos.y)
	{
		cantFriendBlocked++;
	}
	else
	{
		PredictBall ( env );
		Position(robot, env->predictedBall.pos.x, env->predictedBall.pos.y);
	}

	if(cantFriendBlocked>5)
	{
	
		Velocity(robot, -125, -125);
		Angle(robot, 90);
		cantFriendBlocked = 0 ;
	}else
	{
		PredictBall ( env );
		Position(robot, env->predictedBall.pos.x, env->predictedBall.pos.y);
	}




	lastX = robot->pos.x;
	lastY = robot->pos.y;
	//Retrocede por compañeros

	



}

void FollowNearbyOpponent(int attackRobotIndex, int robotIndex, Environment *env)
{
	Robot *robot = &env->home[robotIndex];
	int masCerca, i;
	double distMin, dist;

	masCerca = 1;
	distMin = Distancia(env->opponent[1].pos.x, env->opponent[1].pos.y, env->home[attackRobotIndex].pos.x, env->home[attackRobotIndex].pos.y);
	for (i = 2; i < 5; i++)
	{
		dist = Distancia(env->opponent[i].pos.x, env->opponent[i].pos.y, env->home[attackRobotIndex].pos.x, env->home[attackRobotIndex].pos.y);
		if (dist < distMin)
		{
			masCerca = i;
			distMin = dist;
		}
	}


	Position(robot, env->opponent[masCerca].pos.x, env->opponent[masCerca].pos.y);
}

void Defend(Robot *robot, double x, Environment *env)
{
	PredictBall ( env );
	Position(robot, x, env->predictedBall.pos.y);
}

void Velocity ( Robot *robot, int vl, int vr )
{
	robot->velocityLeft = vl;
	robot->velocityRight = vr;
}

void Angle ( Robot *robot, int desired_angle)
{
	int theta_e, vl, vr;
	theta_e = desired_angle - (int)robot->rotation;

	while (theta_e > 180) theta_e -= 360;
	while (theta_e < -180) theta_e += 360;

	if (theta_e < -90) theta_e += 180;

	else if (theta_e > 90) theta_e -= 180;

	if (abs(theta_e) > 50)
	{
		vl = (int)(-9. / 90.0 * (double) theta_e);
		vr = (int)(9. / 90.0 * (double)theta_e);
	}
	else if (abs(theta_e) > 20)
	{
		vl = (int)(-11.0 / 90.0 * (double)theta_e);
		vr = (int)(11.0 / 90.0 * (double)theta_e);
	}
	Velocity (robot, vl, vr);
}

void Position( Robot *robot, double x, double y )
{
	int desired_angle = 0, theta_e = 0, d_angle = 0, vl, vr, vc = 70;

	double dx, dy, d_e, Ka = 10.0 / 90.0;
	dx = x - robot->pos.x;
	dy = y - robot->pos.y;

	d_e = sqrt(dx * dx + dy * dy);
	if (dx == 0 && dy == 0)
		desired_angle = 90;
	else
		desired_angle = (int)(180. / PI * atan2((double)(dy), (double)(dx)));
	theta_e = desired_angle - (int)robot->rotation;

	while (theta_e > 180) theta_e -= 360;
	while (theta_e < -180) theta_e += 360;

	if (d_e > 100.)
		Ka = 17. / 90.;
	else if (d_e > 50)
		Ka = 19. / 90.;
	else if (d_e > 30)
		Ka = 21. / 90.;
	else if (d_e > 20)
		Ka = 23. / 90.;
	else
		Ka = 25. / 90.;

	if (theta_e > 95 || theta_e < -95)
	{
		theta_e += 180;

		if (theta_e > 180)
			theta_e -= 360;
		if (theta_e > 80)
			theta_e = 80;
		if (theta_e < -80)
			theta_e = -80;
		if (d_e < 5.0 && abs(theta_e) < 40)
			Ka = 0.1;
		vr = (int)(-vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) + Ka * theta_e);
		vl = (int)(-vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) - Ka * theta_e);
	}

	else if (theta_e < 85 && theta_e > -85)
	{
		if (d_e < 5.0 && abs(theta_e) < 40)
			Ka = 0.1;
		vr = (int)( vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) + Ka * theta_e);
		vl = (int)( vc * (1.0 / (1.0 + exp(-3.0 * d_e)) - 0.3) - Ka * theta_e);
	}

	else
	{
		vr = (int)(+.17 * theta_e);
		vl = (int)(-.17 * theta_e);
	}

	Velocity(robot, vl, vr);
}


void PredictBall ( Environment *env )
{
	double dx = env->currentBall.pos.x - env->lastBall.pos.x;
	double dy = env->currentBall.pos.y - env->lastBall.pos.y;
	env->predictedBall.pos.x = env->currentBall.pos.x + dx;
	env->predictedBall.pos.y = env->currentBall.pos.y + dy;

}


void NearBound2 ( Robot *robot, double vl, double vr, Environment *env )
{
	//Vector3D t = env->currentBall.pos;

	Vector3D a = robot->pos;
	double r = robot->rotation;

	if ( a.y > env->fieldBounds.top - 15 && r > 45 && r < 130 )
	{
		if ( vl > 0 )
			vl /= 3;
		if ( vr > 0 )
			vr /= 3;
	}

	if ( a.y < env->fieldBounds.bottom + 15 && r < -45 && r > -130 )
	{
		if ( vl > 0 ) vl /= 3;
		if ( vr > 0 ) vr /= 3;
	}

	if ( a.x > env->fieldBounds.right - 10 )
	{
		if ( vl > 0 )
			vl /= 2;
		if ( vr > 0 )
			vr /= 2;
	}

	if ( a.x < env->fieldBounds.left + 10 )
	{
		if ( vl > 0 )
			vl /= 2;
		if ( vr > 0 )
			vr /= 2;
	}

	robot->velocityLeft = vl;
	robot->velocityRight = vr;
}

void Arquero( Robot *robot, Environment *env )
// Funcion principal de los movimientos del arquero
{
	double yball = env->predictedBall.pos.y;
	double r = robot->rotation;
	while (r > 180)
		r -= 360;
	while (r < -180)
		r += 360;


	if (fabs(fabs(r) - 90) < 5)
	{
		// Estoy apuntando mas o menos hacia arriba o abajo => avanzo o retrocedo
		if (fabs(robot->pos.y - yball) < 1)
		{
			robot->velocityLeft = 0;
			robot->velocityRight = 0;
		}
		else
		{
			if (robot->pos.y < yball)
			{
				if (robot->pos.y > GTOPY)
				{
					robot->velocityLeft = 0;
					robot->velocityRight = 0;
				}
				else
				{
					if (r > 0)		// Mira hacia arriba
					{
						robot->velocityLeft = MaxVel;
						robot->velocityRight = MaxVel;
					}
					else
					{
						robot->velocityLeft = -MaxVel;
						robot->velocityRight = -MaxVel;
					}
				}
			}
			else
			{
				if (robot->pos.y < GBOTY)
				{
					robot->velocityLeft = 0;
					robot->velocityRight = 0;
				}
				else
				{
					if (r > 0)		// Mira hacia arriba
					{
						robot->velocityLeft = -MaxVel;
						robot->velocityRight = -MaxVel;
					}
					else
					{
						robot->velocityLeft = MaxVel;
						robot->velocityRight = MaxVel;
					}
				}
			}
		}
	}
	else
	{
		// Ajusto orientacion
		if (r > 0)   // Miro para arriba
		{
			robot->velocityLeft = -(90 - r) * ATENUAR;
			robot->velocityRight = (90 - r) * ATENUAR;
		}
		else
		{
			robot->velocityLeft = (r + 90) * ATENUAR;
			robot->velocityRight = -(r + 90) * ATENUAR;
		}
	}

}


double Distancia(double x0, double y0, double xf, double yf)
{
	return sqrt((xf - x0) * (xf - x0) + (yf - y0) * (yf - y0));
}


